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
    guidance.p_start_x      float   0.0     start x [m]  (ignored if use_vessel_pos)
    guidance.p_start_y      float   0.0     start y [m]  (ignored if use_vessel_pos)
    guidance.p_end_x        float   3.0     end x [m]
    guidance.p_end_y        float   2.0     end y [m]
    vessel.length           float   1.0     ship length [m]
    vessel.beam             float   0.3     ship beam [m]
    vessel.draft            float   0.1     ship draft [m]
    control.kp              float[] [10,10,5]   diagonal Kp [surge, sway, yaw]
    control.kd              float[] [5,5,2]     diagonal Kd
    control.frequency       float   20.0    control loop rate [Hz]
    control.max_force       float   5.0     saturation limit for surge and sway [N]
    control.max_torque      float   2.0     saturation limit for yaw [Nm]
"""
import json
import math

import numpy as np
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Wrench, Vector3
from std_msgs.msg import Float64, String
from std_srvs.srv import Empty
from tf_transformations import euler_from_quaternion
from shoeboxpy.model3dof import Shoebox

from cybership_controller.guidance.straight_line import (
    DPTrackingController,
    StraightLineGuidance,
)


class StraightLineGuidanceNode(Node):

    def __init__(self):
        super().__init__('straight_line_guidance')

        # ---- guidance parameters ----
        self.declare_parameter('guidance.lam', 0.01)
        self.declare_parameter('guidance.k', 8.0)
        self.declare_parameter('guidance.v_d', 0.0)
        self.declare_parameter('guidance.psi_ref_deg', 0.0)
        self.declare_parameter('guidance.sigma', 1)
        self.declare_parameter('guidance.align_heading', False)
        self.declare_parameter('guidance.use_vessel_pos', True)
        self.declare_parameter('guidance.p_start_x', 0.0)
        self.declare_parameter('guidance.p_start_y', 0.0)
        self.declare_parameter('guidance.p_end_x', 3.0)
        self.declare_parameter('guidance.p_end_y', 2.0)

        # ---- vessel model parameters ----
        self.declare_parameter('vessel.length', 1.0)
        self.declare_parameter('vessel.beam', 0.3)
        self.declare_parameter('vessel.draft', 0.05)

        # ---- controller gains ----
        self.declare_parameter('control.kp', [1.0, 1.0, 1.0])
        self.declare_parameter('control.kd', [0.4, 0.5, 0.2])
        self.declare_parameter('control.frequency', 20.0)
        self.declare_parameter('control.max_force', 5.0)
        self.declare_parameter('control.max_torque', 2.0)

        # ---- instantiate guidance and controller ----
        self.guidance = StraightLineGuidance(
            lam=self.get_parameter('guidance.lam').value,
            k=self.get_parameter('guidance.k').value,
        )
        self.controller = DPTrackingController(
            model=self._build_model(),
            Kp=np.diag(self.get_parameter('control.kp').value),
            Kd=np.diag(self.get_parameter('control.kd').value),
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
        self._dt = 1.0 / self.get_parameter('control.frequency').value
        self._arrived = False

        self.get_logger().info('straight_line_guidance node ready')

    # ------------------------------------------------------------------

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

        p_end = [
            self.get_parameter('guidance.p_end_x').value,
            self.get_parameter('guidance.p_end_y').value,
        ]
        if self.get_parameter('guidance.use_vessel_pos').value:
            p_start = None   # set_line() will use eta_signal[:2]
        else:
            p_start = [
                self.get_parameter('guidance.p_start_x').value,
                self.get_parameter('guidance.p_start_y').value,
            ]

        self.guidance.set_line(p_end=p_end, p_start=p_start)
        self._arrived = False

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
        self._arrived = False
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

        if self._arrived:
            # Hold at the endpoint with zero velocity/acceleration reference.
            eta_d = np.array([self.guidance.p1[0], self.guidance.p1[1], self.guidance.psi_ref])
            eta_d_dot  = np.zeros(3)
            eta_d_ddot = np.zeros(3)
        else:
            eta_d, eta_d_dot, eta_d_ddot = self.guidance.step(self._dt)
            if self.guidance.arrived:
                self._arrived = True
                self.get_logger().info('Arrived at end point — holding position.')

        self.controller.eta_d_signal = eta_d
        self.controller.eta_d_dot_signal = eta_d_dot
        self.controller.eta_d_ddot_signal = eta_d_ddot

        tau = self.controller.compute_tau()

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
        self._pub_state.publish(String(data='holding' if self._arrived else 'running'))


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
