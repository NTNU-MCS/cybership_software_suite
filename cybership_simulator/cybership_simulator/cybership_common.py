#!/usr/bin/env python3

import numpy as np
import rclpy

from rclpy.node import Node
import rclpy.clock
import rclpy.logging
import rcl_interfaces

import cybership_interfaces.srv

from std_srvs.srv import Empty
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import (
    Wrench,
    TransformStamped,
    WrenchStamped,
    PoseStamped,
    PoseWithCovarianceStamped,
    TwistWithCovarianceStamped,
)

from tf2_ros import TransformBroadcaster
from cybership_simulator.common_tools.math_tools import *

from rosgraph_msgs.msg import Clock


class Simulator(Node):
    """
    The enterprise object represents the C/S Enterprise and contains the necessary
    kinematics and dynamics of the ship, as well as the operations required to
    "move" the ship over one time-step
    """

    ### Main data of the C/S Enterprise. Do not touch ###
    _M = np.array(
        [[16.11, 0.0, 0.0], [0.0, 24.11, 0.5291], [0.0, 0.5291, 2.7600]], dtype=float
    )  # Inertia matrix
    _X = np.array(
        [-0.6555, 0.3545, -3.787, 0.0, -2.443, 0.0], dtype=float
    )  # Hydro surge [Xu, Xuu, Xuuu, Xv, Xvv, Xvvv]
    _Y = np.array(
        [-1.3443, -2.776, -65.91, -7.25, -3.45, 0.0, -0.805, -0.845], dtype=float
    )  # Hydro sway [Yv, Yvv, Yvvv, Yr, Yrr, Yrrr, Yrv, Yvr]
    _N = np.array(
        [0.0, -0.2088, 0.0, -1.9, -0.75, 0.0, 0.130, 0.080], dtype=float
    )  # Hydro yaw [Nv, Nvv, Nvvv, Nr, Nrr, Nrrr, Nrv, Nvr]
    _A = np.array([-2, -10, -0, -1], dtype=float)  # Added mass [Xu, Yv, Yr, Nr]
    _m = 14.11  # Rigid body mass
    _Iz = 1.7600  # Inertial moment
    _xg = 0.0375  # Center of gravity
    lx1 = 0.3875
    lx2 = -0.4574
    lx3 = -0.4574
    ly1 = 0
    ly2 = -0.055
    ly3 = 0.055

    _K = np.diag([2.629, 1.03, 1.03])  # K-matrix

    def __init__(self, *args, **kwargs):
        super().__init__(
            node_name="cybership_simulator",
            allow_undeclared_parameters=True,
        )

        self.eta = np.zeros((3, 1), dtype=float)
        self.nu = np.zeros((3, 1), dtype=float)
        self.tau = np.zeros((3, 1), dtype=float)
        self.nu_dot = np.zeros((3, 1), dtype=float)
        self.eta_dot = np.zeros((3, 1), dtype=float)
        self.u = np.zeros(5, dtype=float)

        self.D = np.zeros((3, 3), dtype=float)
        self.C = np.zeros((3, 3), dtype=float)

        self._declare_parameters()
        self._read_parameters()

        self.odom = Odometry()  # Msg to be published

        self.tauMsg = Float64MultiArray()

        # TF2 Broadcast
        self.tf_broadcaster = TransformBroadcaster(self, 1)

        self.publisher_odom = self.create_publisher(Odometry, "measurement/odom", 1)
        self.publisher_pose = self.create_publisher(
            PoseWithCovarianceStamped, "measurement/pose", 1
        )
        self.subscriber_tunnel_thruster = self.create_subscription(
            Wrench, "thruster/tunnel/command", self.cb_tunnel_thruster, 10
        )
        self.subscriber_starboard_thruster = self.create_subscription(
            Wrench, "thruster/starboard/command", self.cb_starboard_thruster, 10
        )
        self.subcriber_port_thruster = self.create_subscription(
            Wrench, "thruster/port/command", self.cb_port_thruster, 10
        )

        self.publisher_tunnel_thruster = self.create_publisher(
            WrenchStamped, "thruster/tunnel/issued", 1
        )
        self.publisher_starboard_thruster = self.create_publisher(
            WrenchStamped, "thruster/starboard/issued", 1
        )
        self.publisher_port_thruster = self.create_publisher(
            WrenchStamped, "thruster/port/issued", 1
        )
        self.publisher_allocated = self.create_publisher(WrenchStamped, "allocated", 1)

        self.reset_service = self.create_service(Empty, "simulator/reset", self.reset_simulation)

        self.reset_service2 = self.create_service(
            cybership_interfaces.srv.ResetSimulator, "simulator/reset_pose", self.reset_simulator2
        )

        self.dt = 0.01

        self._loop_rate = self.create_rate((1.0 / self.dt), self.get_clock())

        self.timer = self.create_timer(self.dt, self.iterate)

    def reset_simulation(self, request, response):
        self.eta = np.zeros((3, 1), dtype=float)
        self.nu = np.zeros((3, 1), dtype=float)
        self.tau = np.zeros((3, 1), dtype=float)
        self.nu_dot = np.zeros((3, 1), dtype=float)
        self.eta_dot = np.zeros((3, 1), dtype=float)
        self.u = np.zeros(5, dtype=float)

        self.D = np.zeros((3, 3), dtype=float)
        self.C = np.zeros((3, 3), dtype=float)

        self._read_parameters()

        return response

    def reset_simulator2(self, request, response):
        x = request.pose.x
        y = request.pose.y
        yaw = request.pose.theta

        self.eta = np.array([[x], [y], [yaw]], dtype=float)
        self.nu = np.zeros((3, 1), dtype=float)
        self.tau = np.zeros((3, 1), dtype=float)
        self.nu_dot = np.zeros((3, 1), dtype=float)
        self.eta_dot = np.zeros((3, 1), dtype=float)
        self.u = np.zeros(5, dtype=float)

        self.D = np.zeros((3, 3), dtype=float)
        self.C = np.zeros((3, 3), dtype=float)

        # self._read_parameters()

        response.success = True
        return response

    def _declare_parameters(self):
        self.declare_parameter(
            "initial_conditions.position.x", rclpy.Parameter.Type.DOUBLE
        )
        self.declare_parameter(
            "initial_conditions.position.y", rclpy.Parameter.Type.DOUBLE
        )
        self.declare_parameter(
            "initial_conditions.position.yaw", rclpy.Parameter.Type.DOUBLE
        )
        self.declare_parameter(
            "initial_conditions.velocity.surge", rclpy.Parameter.Type.DOUBLE
        )
        self.declare_parameter(
            "initial_conditions.velocity.sway", rclpy.Parameter.Type.DOUBLE
        )
        self.declare_parameter(
            "initial_conditions.velocity.yaw_rate", rclpy.Parameter.Type.DOUBLE
        )

    def _read_parameters(self):
        self.eta[0] = (
            self.get_parameter("initial_conditions.position.x")
            .get_parameter_value()
            .double_value
        )
        self.eta[1] = (
            self.get_parameter("initial_conditions.position.y")
            .get_parameter_value()
            .double_value
        )
        self.eta[2] = (
            self.get_parameter("initial_conditions.position.yaw")
            .get_parameter_value()
            .double_value
        )

        self.nu[0] = (
            self.get_parameter("initial_conditions.velocity.surge")
            .get_parameter_value()
            .double_value
        )
        self.nu[1] = (
            self.get_parameter("initial_conditions.velocity.sway")
            .get_parameter_value()
            .double_value
        )
        self.nu[2] = (
            self.get_parameter("initial_conditions.velocity.yaw_rate")
            .get_parameter_value()
            .double_value
        )

    def _update_parameters(self):
        pass

    def publish_odom(self):
        self.nav_msg()
        self.publisher_odom.publish(self.odom)

    # def publish_tau(self):
    #     tau = Float64MultiArray()
    #     tau.data.fromlist(self.tau.flatten().tolist())
    #     self.pub_tau.publish(tau)

    def publish_clock(self):

        self.t += self.dt
        msg = Clock()
        msg.clock.sec = int(self.t)
        msg.clock.nanosec = int((self.t - msg.clock.sec) * 1e9)
        self.publisher_clock.publish(msg)

    def set_D(self):
        u = self.nu[0]
        v = self.nu[1]
        r = self.nu[2]
        d11 = (-self._X[0] - self._X[1] * np.abs(u) - self._X[2] * (u**2))[0]
        d22 = (-self._Y[0] - self._Y[1] * np.abs(v) - self._Y[2] * (v**2))[0]
        d33 = (-self._N[3] - self._N[7] * np.abs(v) - self._N[5] * (r**2))[0]
        d23 = (
            -self._Y[3]
            - self._Y[7] * np.abs(v)
            - self._Y[4] * np.abs(r)
            - self._Y[5] * (r**2)
        )[0]
        d32 = (
            -self._N[0]
            - self._N[2] * np.abs(v)
            - self._N[3] * (v**2)
            - self._N[6] * np.abs(r)
        )[0]
        new_D = np.array([[d11, 0, 0], [0, d22, d23], [0, d32, d33]])
        self.D = new_D

    def set_C(self):
        u = self.nu[0]
        v = self.nu[1]
        r = self.nu[2]
        c13 = ((-self._m * self._xg + self._A[2]) * r + (-self._m + self._A[1]) * v)[0]
        c23 = ((self._m - self._A[0]) * u)[0]
        new_C = np.array([[0, 0, c13], [0, 0, c23], [-c13, -c23, 0]])
        self.C = new_C

    def set_tau(self, u):

        u_t = np.transpose(np.take(u, [0, 1, 2])[np.newaxis])
        alpha = np.take(u, [3, 4])
        c1 = 0
        c2 = np.cos(alpha[0])
        c3 = np.cos(alpha[1])
        s1 = 1
        s2 = np.sin(alpha[0])
        s3 = np.sin(alpha[1])
        B = np.array(
            [
                [c1, c2, c3],
                [s1, s2, s3],
                [
                    self.lx1 * s1 - self.ly1 * c1,
                    self.lx2 * s2 - self.ly2 * c2,
                    self.lx3 * s3 - self.ly3 * c3,
                ],
            ]
        )
        new_tau = (B @ self._K) @ u_t
        self.tau = new_tau

        allocated = WrenchStamped()
        allocated.header.frame_id = "base_link"
        allocated.header.stamp = self.get_clock().now().to_msg()
        allocated.wrench.force.x = self.tau[0, 0]
        allocated.wrench.force.y = self.tau[1, 0]
        allocated.wrench.torque.z = self.tau[2, 0]
        self.publisher_allocated.publish(allocated)

    def set_eta(self):
        psi = self.eta[2]
        R = Rzyx(psi)
        self.eta_dot = np.dot(R, self.nu)
        self.eta = self.eta + self.dt * self.eta_dot
        self.eta[2] = rad2pipi(self.eta[2])  # Wrap the angle

    def set_nu(self):
        A = self._M
        b = self.tau - np.dot((self.C + self.D), self.nu)
        self.nu_dot = np.linalg.solve(A, b)
        self.nu = self.nu + self.dt * self.nu_dot  # Integration, forward euler

    def get_tau(self):
        return self.tau

    def get_eta(self):
        return self.eta

    def get_nu(self):
        return self.nu

    def nav_msg(self):
        """
        Computes the Odometry message of the ship
        """
        quat = yaw2quat(self.eta[2][0])

        self.odom.header.frame_id = "world"
        self.odom.header.stamp = self.get_clock().now().to_msg()

        self.odom.child_frame_id = "base_link"
        self.odom.pose.pose.position.x = self.eta[0].item()
        self.odom.pose.pose.position.y = self.eta[1].item()
        self.odom.pose.pose.position.z = 0.0
        self.odom.pose.pose.orientation.w = quat[0].item()
        self.odom.pose.pose.orientation.x = quat[1].item()
        self.odom.pose.pose.orientation.y = quat[2].item()
        self.odom.pose.pose.orientation.z = quat[3].item()

        self.odom.twist.twist.linear.x = self.nu[0].item()
        self.odom.twist.twist.linear.y = self.nu[1].item()
        self.odom.twist.twist.linear.z = 0.0
        self.odom.twist.twist.angular.x = 0.0
        self.odom.twist.twist.angular.y = 0.0
        self.odom.twist.twist.angular.z = self.nu[2].item()

        self.publisher_odom.publish(self.odom)

        pose_msg = PoseWithCovarianceStamped()
        pose_msg.header = self.odom.header

        pose_msg.pose = self.odom.pose
        self.publisher_pose.publish(pose_msg)

    def publish_tf(self):
        t = TransformStamped()

        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "world"
        t.child_frame_id = "base_link"
        t.transform.translation.x = self.odom.pose.pose.position.x
        t.transform.translation.y = self.odom.pose.pose.position.y
        t.transform.translation.z = self.odom.pose.pose.position.z
        t.transform.rotation.x = self.odom.pose.pose.orientation.x
        t.transform.rotation.y = self.odom.pose.pose.orientation.y
        t.transform.rotation.z = self.odom.pose.pose.orientation.z
        t.transform.rotation.w = self.odom.pose.pose.orientation.w

        self.tf_broadcaster.sendTransform(t)

    def get_odom(self):
        return self.odom

    def cb_tunnel_thruster(self, msg):
        self.u[0] = msg.force.x

        issued = WrenchStamped()
        issued.header.frame_id = "bow_tunnel_thruster_link"
        issued.header.stamp = self.get_clock().now().to_msg()
        issued.wrench.force.x = msg.force.x
        self.publisher_tunnel_thruster.publish(issued)

    def cb_starboard_thruster(self, msg):
        self.u[1] = np.linalg.norm(np.array((msg.force.y, msg.force.x)))
        self.u[3] = np.arctan2(msg.force.y, msg.force.x)

        issued = WrenchStamped()
        issued.header.frame_id = "stern_starboard_thruster_link"
        issued.header.stamp = self.get_clock().now().to_msg()
        issued.wrench.force.x = msg.force.x
        issued.wrench.force.y = msg.force.y
        self.publisher_starboard_thruster.publish(issued)

    def cb_port_thruster(self, msg):
        self.u[2] = np.linalg.norm(np.array((msg.force.y, msg.force.x)))
        self.u[4] = np.arctan2(msg.force.y, msg.force.x)

        issued = WrenchStamped()
        issued.header.frame_id = "stern_port_thruster_link"
        issued.header.stamp = self.get_clock().now().to_msg()
        issued.wrench.force.x = msg.force.x
        issued.wrench.force.y = msg.force.y
        self.publisher_port_thruster.publish(issued)

    # Move the ship
    def iterate(self):
        # self.publish_clock()
        self.set_C()  # Coreolis matrix
        self.set_D()  # Compute damping matrix
        self.set_tau(self.u)  # Compute the force vector
        # self.publish_tau()   # Publish the tau, this is needed for the Observer :)
        self.set_nu()  # Compute the velocity
        self.set_eta()  # Compute the position
        self.publish_odom()  # Publish the new position
        self.publish_tf()


def main(args=None):

    rclpy.init(args=args)
    simulator = Simulator()

    rclpy.spin(simulator)

    rclpy.shutdown()


if __name__ == "__main__":
    main()
