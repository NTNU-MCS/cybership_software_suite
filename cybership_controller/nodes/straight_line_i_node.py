#!/usr/bin/env python3
"""
Straight-line maneuvering guidance + model-based DP tracking controller.

Wraps StraightLineGuidance and DPTrackingController into a ROS 2 node.

Interfaces
----------
Subscriptions:
    measurement/odom        nav_msgs/Odometry       vessel pose + body-frame twist

Publications:
    command/force/manual    geometry_msgs/Wrench    body-frame force demand [Fx, Fy, Mz]

Services:
    guidance/straight_line/set_line   std_srvs/Empty   read params, call set_line(), start timer
    guidance/straight_line/stop       std_srvs/Empty   cancel timer, publish zero wrench

Parameters
----------
    guidance.lam            float   0.01    path regularisation (λ ≪ 1)
    guidance.k              float   8.0     tanh ramp steepness
    guidance.v_d            float   0.0     desired speed [m/s]
    guidance.psi_ref_deg    float   0.0     heading reference [deg] — converted to rad
    guidance.sigma          int     1       smooth acceleration {0=step, 1=tanh}
    guidance.align_heading  bool    False   override psi_ref with path bearing
    guidance.use_vessel_pos bool    True    use current odom as p_start
    guidance.p_start_x      float   None    start x [m]  (ignored if use_vessel_pos; reset to None on stop)
    guidance.p_start_y      float   None    start y [m]  (ignored if use_vessel_pos; reset to None on stop)
    guidance.p_end_x        float   None    end x [m]   (must be set before calling set_line; reset to None on stop)
    guidance.p_end_y        float   None    end y [m]   (must be set before calling set_line; reset to None on stop)
    vessel.length           float   1.0     ship length [m]
    vessel.beam             float   0.3     ship beam [m]
    vessel.draft            float   0.1     ship draft [m]
    control.kp              float[] [10,10,5]   diagonal Kp [surge, sway, yaw]
    control.kd              float[] [5,5,2]     diagonal Kd
    control.ki              float[] [0,0,0]     diagonal Ki (integral gains; reset on set_line)
    control.frequency       float   20.0    control loop rate [Hz]
    control.max_force       float   5.0     saturation limit for surge and sway [N]
    control.max_torque      float   2.0     saturation limit for yaw [Nm]
"""
import json
import math

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.msg import ParameterDescriptor, SetParametersResult
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Wrench, Vector3
from std_msgs.msg import Float64, String
from std_srvs.srv import Empty
from tf_transformations import euler_from_quaternion
from shoeboxpy.model3dof import Shoebox

from cybership_controller.guidance.straight_line import StraightLineGuidance
from cybership_controller.position.tracking_controller import DPTrackingController

class StraightLineGuidanceNode(Node):

    def __init__(self):
        super().__init__('straight_line_guidance', allow_undeclared_parameters=True)

        # ---- guidance parameters ----
        self.declare_parameter('guidance.lam', 0.01)
        self.declare_parameter('guidance.k', 10.0)
        self.declare_parameter('guidance.v_d', 0.0)
        self.declare_parameter('guidance.psi_ref_deg', 0.0)
        self.declare_parameter('guidance.sigma', 1)
        self.declare_parameter('guidance.align_heading', False)
        self.declare_parameter('guidance.use_vessel_pos', True)
        _dyn = ParameterDescriptor(dynamic_typing=True)
        self.declare_parameter('guidance.p_start_x', None, _dyn)
        self.declare_parameter('guidance.p_start_y', None, _dyn)
        self.declare_parameter('guidance.p_end_x', None, _dyn)
        self.declare_parameter('guidance.p_end_y', None, _dyn)

        # ---- vessel model parameters ----
        self.declare_parameter('vessel.length', 1.0)
        self.declare_parameter('vessel.beam', 0.3)
        self.declare_parameter('vessel.draft', 0.05)

        # ---- controller gains ----
        self.declare_parameter('control.kp', [0.7, 0.7, 0.7])
        self.declare_parameter('control.kd', [0.4, 0.5, 0.2])
        self.declare_parameter('control.ki', [0.1, 0.1, 0.1])
        self.declare_parameter('control.frequency', 20.0)
        self.declare_parameter('control.max_force', 5.0)
        self.declare_parameter('control.max_torque', 2.0)

        # ---- instantiate guidance and controller ----
        self.guidance = StraightLineGuidance(
            lam=self.get_parameter('guidance.lam').value,
            k=self.get_parameter('guidance.k').value,
        )
        freq = self.get_parameter('control.frequency').value
        self.controller = DPTrackingController(
            model=self._build_model(),
            Kp=np.diag(self.get_parameter('control.kp').value),
            Kd=np.diag(self.get_parameter('control.kd').value),
            Ki=np.diag(self.get_parameter('control.ki').value),
        )

        # ---- subscribers ----
        self._odom_sub = self.create_subscription(
            Odometry, 'measurement/odom', self._odom_callback, 10)

        # ---- publishers ----
        self._force_pub = self.create_publisher(
            Wrench, 'control/force/command/manual', 10)
        self._pub_path_length = self.create_publisher(
            Float64, 'guidance/straight_line/path_length', 10)
        self._pub_bearing = self.create_publisher(
            Float64, 'guidance/straight_line/bearing', 10)
        self._pub_theta = self.create_publisher(
            Float64, 'guidance/straight_line/theta', 10)
        self._pub_state = self.create_publisher(
            String, 'guidance/straight_line/state', 10)
        self._pub_tracking_error = self.create_publisher(
            Vector3, 'guidance/straight_line/tracking_error', 10)

        # ---- services ----
        self._set_line_srv = self.create_service(
            Empty, 'guidance/straight_line/set_line', self._set_line_callback)
        self._stop_srv = self.create_service(
            Empty, 'guidance/straight_line/stop', self._stop_callback)

        self._control_timer = None
        self._dt = 1.0 / freq

        self.add_on_set_parameters_callback(self._on_set_parameters)

        self.get_logger().info('straight_line_guidance node ready')

    # ------------------------------------------------------------------

    def _on_set_parameters(self, params):
        for p in params:
            if p.name == 'control.kp':
                self.controller.Kp = np.diag(p.value)
                self.get_logger().info(f'Kp updated: {p.value}')
            elif p.name == 'control.kd':
                self.controller.Kd = np.diag(p.value)
                self.get_logger().info(f'Kd updated: {p.value}')
            elif p.name == 'control.ki':
                self.controller.Ki = np.diag(p.value)
                self.get_logger().info(f'Ki updated: {p.value}')
        return SetParametersResult(successful=True)

    def _build_model(self) -> Shoebox:
        return Shoebox(
            L=self.get_parameter('vessel.length').value,
            B=self.get_parameter('vessel.beam').value,
            T=self.get_parameter('vessel.draft').value,
        )

    def _odom_callback(self, msg: Odometry):
        q = msg.pose.pose.orientation
        _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
        eta = np.array([
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            yaw,
        ])
        # twist in odom is expressed in the body frame (ROS + NED convention)
        nu = np.array([
            msg.twist.twist.linear.x,
            msg.twist.twist.linear.y,
            msg.twist.twist.angular.z,
        ])
        self.guidance.eta_signal = eta
        self.controller.eta_signal = eta
        self.controller.nu_signal = nu

    def _set_line_callback(self, _request, response):
        # Snapshot all guidance parameters at the moment "Set line / start" is pressed.
        self.guidance.lam   = float(self.get_parameter('guidance.lam').value)
        self.guidance.k     = float(self.get_parameter('guidance.k').value)
        self.guidance.v_d   = float(self.get_parameter('guidance.v_d').value)
        self.guidance.sigma = int(self.get_parameter('guidance.sigma').value)
        self.guidance.psi_ref = math.radians(
            float(self.get_parameter('guidance.psi_ref_deg').value))

        p_end_x = self.get_parameter('guidance.p_end_x').value if self.has_parameter('guidance.p_end_x') else None
        p_end_y = self.get_parameter('guidance.p_end_y').value if self.has_parameter('guidance.p_end_y') else None

        if p_end_x is None and p_end_y is None:
            self.get_logger().warn('p_end not configured — ignoring set_line call.')
            return response

        if p_end_x is None or p_end_y is None:
            # Single-axis offset mode: the non-None value is a relative offset;
            # the missing axis is snapped from the vessel's current position.
            vessel_pos = self.guidance.eta_signal[:2]
            if p_end_y is None:
                p_end = [vessel_pos[0] + p_end_x, vessel_pos[1]]
            else:
                p_end = [vessel_pos[0], vessel_pos[1] + p_end_y]
            p_start = None  # always start from current vessel position
        else:
            # Both axes set: treat as absolute world coordinates (existing behaviour).
            p_end = [p_end_x, p_end_y]
            if self.get_parameter('guidance.use_vessel_pos').value:
                p_start = None
            else:
                p_start_x = self.get_parameter('guidance.p_start_x').value if self.has_parameter('guidance.p_start_x') else None
                p_start_y = self.get_parameter('guidance.p_start_y').value if self.has_parameter('guidance.p_start_y') else None
                if p_start_x is None or p_start_y is None:
                    self.get_logger().warn('p_start not configured and use_vessel_pos=False — ignoring set_line call.')
                    return response
                p_start = [p_start_x, p_start_y]

        self.guidance.set_line(p_end=p_end, p_start=p_start)
        self.controller.reset()

        # (re)start the control loop
        if self._control_timer is not None:
            self._control_timer.cancel()
        self._control_timer = self.create_timer(self._dt, self._control_callback)

        bearing_deg = math.degrees(math.atan2(
            self.guidance.p1[1] - self.guidance.p0[1],
            self.guidance.p1[0] - self.guidance.p0[0],
        ))
        self.get_logger().info(
            f'Line set — p_start={list(self.guidance.p0)}  p_end={list(self.guidance.p1)}  '
            f'length={self.guidance._len:.3f} m  bearing={bearing_deg:.1f}°  '
            f'v_d={self.guidance.v_d:.3f} m/s  sigma={self.guidance.sigma}')
        return response

    def _stop_callback(self, _request, response):
        if self._control_timer is not None:
            self._control_timer.cancel()
            self._control_timer = None
        self.guidance.active = False
        self.set_parameters([
            Parameter('guidance.p_end_x'),
            Parameter('guidance.p_end_y'),
            Parameter('guidance.p_start_x'),
            Parameter('guidance.p_start_y'),
        ])
        self._force_pub.publish(Wrench())
        self._pub_state.publish(String(data='idle'))
        self.get_logger().info('Guidance stopped.')
        return response

    def _control_callback(self):
        if not self.guidance.active:
            return

        # sigma, v_d, and psi_ref can be changed live from the UI without
        # restarting the guidance loop.
        self.guidance.sigma = int(self.get_parameter('guidance.sigma').value)
        self.guidance.v_d = float(self.get_parameter('guidance.v_d').value)

        if self.get_parameter('guidance.align_heading').value:
            # keep psi_ref locked to the path bearing
            self.guidance.psi_ref = math.atan2(
                self.guidance.p1[1] - self.guidance.p0[1],
                self.guidance.p1[0] - self.guidance.p0[0],
            )
        else:
            self.guidance.psi_ref = math.radians(
                float(self.get_parameter('guidance.psi_ref_deg').value))

        eta_d, eta_d_dot, eta_d_ddot = self.guidance.step(self._dt)

        self.controller.eta_d_signal = eta_d
        self.controller.eta_d_dot_signal = eta_d_dot
        self.controller.eta_d_ddot_signal = eta_d_ddot

        tau = self.controller.compute_tau(dt=self._dt)

        max_f = float(self.get_parameter('control.max_force').value)
        max_t = float(self.get_parameter('control.max_torque').value)
        tau = np.array([
            np.clip(tau[0], -max_f, max_f),
            np.clip(tau[1], -max_f, max_f),
            np.clip(tau[2], -max_t, max_t),
        ])

        wrench = Wrench()
        wrench.force.x = float(tau[0])
        wrench.force.y = float(tau[1])
        wrench.torque.z = float(tau[2])
        self._force_pub.publish(wrench)

        # ---- tracking error (actual − reference) ----
        eta = self.controller.eta_signal
        e_x   = float(eta[0] - eta_d[0])
        e_y   = float(eta[1] - eta_d[1])
        e_psi = float(DPTrackingController.wrap(eta[2] - eta_d[2]))
        self._pub_tracking_error.publish(Vector3(x=e_x, y=e_y, z=e_psi))

        # ---- live status ----
        bearing = math.atan2(
            self.guidance.p1[1] - self.guidance.p0[1],
            self.guidance.p1[0] - self.guidance.p0[0],
        ) if self.guidance._len > 1e-9 else 0.0
        self._pub_path_length.publish(Float64(data=self.guidance._len))
        self._pub_bearing.publish(Float64(data=bearing))
        self._pub_theta.publish(Float64(data=self.guidance.theta))
        self._pub_state.publish(String(data='running'))


def main(args=None):
    rclpy.init(args=args)
    node = StraightLineGuidanceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down on SIGINT')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
