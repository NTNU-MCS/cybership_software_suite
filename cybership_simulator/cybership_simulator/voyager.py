# /usr/bin/env python3
# -*- coding: utf-8 -*-

import shoeboxpy
import shoeboxpy.model6dof
import skadipy.actuator
import skadipy.allocator

from scipy.spatial.transform import Rotation as R
import rclpy
from rclpy.node import Node
from rclpy.clock import clock
import numpy as np

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
from rosgraph_msgs.msg import Clock
from cybership_simulator.common_tools.math_tools import *


class Simulator(Node):

    def __init__(self, *args, **kwargs):
        super().__init__(
            node_name="voyager_simulator",
            allow_undeclared_parameters=True,
        )

        self.vessel = shoeboxpy.model6dof.Shoebox(
            L=1.0, B=0.3, T=0.08, GM_theta=0.2, GM_phi=0.2
        )

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

        self.reset_service = self.create_service(
            Empty, "simulator/reset", self.reset_simulation
        )

        self.reset_pose = self.create_service(
            cybership_interfaces.srv.ResetSimulator,
            "simulator/reset_pose",
            self.reset_pose,
        )

        self.dt = 0.01

        self._loop_rate = self.create_rate((1.0 / self.dt), self.get_clock())

        self.timer = self.create_timer(self.dt, self.iterate)

    def _init_allocator(self):

        tunnel = skadipy.actuator.Fixed(
            position=skadipy.toolbox.Point([0.3875, 0.0, -0.01]),
            orientation=skadipy.toolbox.Quaternion(
                axis=(0.0, 0.0, 1.0), radians=np.pi / 2.0
            ),
            extra_attributes={
                "rate_limit": 1.0,
                "saturation_limit": 1.0,
                "name": "tunnel",
            },
        )
        port_azimuth = skadipy.actuator.Azimuth(
            position=skadipy.toolbox.Point([-0.4574, -0.055, -0.1]),
            extra_attributes={
                "rate_limit": 1.0,
                "saturation_limit": 1.0,
                "reference_angle": np.pi / 4.0,
                "name": "port_azimuth",
            },
        )
        starboard_azimuth = skadipy.actuator.Azimuth(
            position=skadipy.toolbox.Point([-0.4547, 0.055, -0.1]),
            extra_attributes={
                "rate_limit": 1.0,
                "saturation_limit": 1.0,
                "reference_angle": -np.pi / 4.0,
                "name": "starboard_azimuth",
            },
        )

        actuators = [
            tunnel,
            port_azimuth,
            starboard_azimuth,
        ]
        dofs = [
            skadipy.allocator._base.ForceTorqueComponent.X,
            skadipy.allocator._base.ForceTorqueComponent.Y,
            skadipy.allocator._base.ForceTorqueComponent.Z,
            skadipy.allocator._base.ForceTorqueComponent.K,
            skadipy.allocator._base.ForceTorqueComponent.M,
            skadipy.allocator._base.ForceTorqueComponent.N,
        ]

        self.allocator = skadipy.allocator.PseudoInverse(
            actuators=actuators, force_torque_components=dofs
        )

        self.allocator.compute_configuration_matrix()

    def reset_simulation(self, request, response):

        self._read_parameters()

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
        """
        Computes the Odometry message of the ship
        """
        eta, nu = self.vessel.get_states()

        rot = R.from_euler("xyz", eta[3:5], degrees=False)
        quat = rot.as_quat()

        self.odom.header.frame_id = "world"
        self.odom.header.stamp = self.get_clock().now().to_msg()

        self.odom.child_frame_id = "base_link"
        self.odom.pose.pose.position.x = eta[0].item()
        self.odom.pose.pose.position.y = eta[1].item()
        self.odom.pose.pose.position.z = eta[2].item()
        self.odom.pose.pose.orientation.w = quat[0].item()
        self.odom.pose.pose.orientation.x = quat[1].item()
        self.odom.pose.pose.orientation.y = quat[2].item()
        self.odom.pose.pose.orientation.z = quat[3].item()

        self.odom.twist.twist.linear.x = nu[0].item()
        self.odom.twist.twist.linear.y = nu[1].item()
        self.odom.twist.twist.linear.z = nu[2].item()
        self.odom.twist.twist.angular.x = nu[3].item()
        self.odom.twist.twist.angular.y = nu[4].item()
        self.odom.twist.twist.angular.z = nu[5].item()

        self.publisher_odom.publish(self.odom)

        pose_msg = PoseWithCovarianceStamped()
        pose_msg.header = self.odom.header

        pose_msg.pose = self.odom.pose
        self.publisher_pose.publish(pose_msg)

    def publish_clock(self):

        self.t += self.dt
        msg = Clock()
        msg.clock.sec = int(self.t)
        msg.clock.nanosec = int((self.t - msg.clock.sec) * 1e9)
        self.publisher_clock.publish(msg)

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
        self.u[1] = np.clip(msg.force.x, -1.0, 1.0)
        self.u[2] = np.clip(msg.force.y, -1.0, 1.0)

        issued = WrenchStamped()
        issued.header.frame_id = "stern_starboard_thruster_link"
        issued.header.stamp = self.get_clock().now().to_msg()
        issued.wrench.force.x = msg.force.x
        issued.wrench.force.y = msg.force.y
        self.publisher_starboard_thruster.publish(issued)

    def cb_port_thruster(self, msg):
        self.u[3] = np.clip(msg.force.x, -1.0, 1.0)
        self.u[4] = np.clip(msg.force.y, -1.0, 1.0)

        issued = WrenchStamped()
        issued.header.frame_id = "stern_port_thruster_link"
        issued.header.stamp = self.get_clock().now().to_msg()
        issued.wrench.force.x = msg.force.x
        issued.wrench.force.y = msg.force.y
        self.publisher_port_thruster.publish(issued)

    # Move the ship
    def iterate(self):

        # compute the force
        tau = self.allocator._b_matrix @ self.u

        self.vessel.step(
            dt=self.dt,
            tau=tau,
        )

        self.publish_odom()  # Publish the new position
        self.publish_tf()


def main(args=None):

    rclpy.init(args=args)
    simulator = Simulator()

    rclpy.spin(simulator)

    rclpy.shutdown()


if __name__ == "__main__":
    main()
