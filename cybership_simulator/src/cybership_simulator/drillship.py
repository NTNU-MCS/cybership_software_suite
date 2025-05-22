#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from cybership_simulator.base import BaseSimulator
import numpy as np
import shoeboxpy.model6dof
import skadipy
import rclpy
import geometry_msgs.msg

class DrillshipSimulator(BaseSimulator):
    """
    Concrete simulator for the Drillship vessel.
    Defines vessel geometry, thruster configuration, and topic handling for thruster commands.
    """

    def __init__(self):
        super().__init__(node_name="drillship_simulator")

    def _create_vessel(self):
        return shoeboxpy.model6dof.Shoebox(
            L=3.0, B=0.4, T=0.10, GM_theta=0.02, GM_phi=0.02,
            eta0=np.array([0.0, 0.0, 0.0, 0.2, 0.2, 0.0]),
        )

    def _init_allocator(self):

        bow_port_azimuth = skadipy.actuator.Azimuth(
            position=skadipy.toolbox.Point([0.9344, -0.11, -0.1]),
        )
        bow_center_azimuth = skadipy.actuator.Azimuth(
            position=skadipy.toolbox.Point([1.0678, 0.0, -0.01]),
        )
        bow_starboard_azimuth = skadipy.actuator.Azimuth(
            position=skadipy.toolbox.Point([0.9344, 0.11, -0.1]),
        )

        aft_port_azimuth = skadipy.actuator.Azimuth(
            position=skadipy.toolbox.Point([-0.9911, -0.1644, -0.1]),
        )
        aft_center_azimuth = skadipy.actuator.Azimuth(
            position=skadipy.toolbox.Point([-1.1644, 0.0, -0.01]),
        )
        aft_starboard_azimuth = skadipy.actuator.Azimuth(
            position=skadipy.toolbox.Point([-0.9911, 0.1644, -0.1]),
        )

        actuators = [
            bow_port_azimuth,
            bow_center_azimuth,
            bow_starboard_azimuth,
            aft_port_azimuth,
            aft_center_azimuth,
            aft_starboard_azimuth
        ]

        dofs = [
            skadipy.allocator._base.ForceTorqueComponent.X,
            skadipy.allocator._base.ForceTorqueComponent.Y,
            skadipy.allocator._base.ForceTorqueComponent.Z,
            skadipy.allocator._base.ForceTorqueComponent.K,
            skadipy.allocator._base.ForceTorqueComponent.M,
            skadipy.allocator._base.ForceTorqueComponent.N
        ]
        return skadipy.allocator.PseudoInverse(actuators=actuators, force_torque_components=dofs)

    def setup_thrusters(self):
        """
        Sets up thruster publishers/subscriptions for Drillship.
        In this example, Drillship uses three thruster groups with a combined command vector of dimension 5.
        """
        # Initialize the thruster command vector (5 elements)
        self.u = np.zeros((12, 1))


        self.subscriber_bow_port_thruster = self.create_subscription(
            geometry_msgs.msg.Wrench, "thruster/bow_port/command", self.cb_bow_port_thruster, 10
        )
        self.subscriber_bow_center_thruster = self.create_subscription(
            geometry_msgs.msg.Wrench, "thruster/bow_center/command", self.cb_bow_center_thruster, 10
        )
        self.subscriber_bow_starboard_thruster = self.create_subscription(
            geometry_msgs.msg.Wrench, "thruster/bow_starboard/command", self.cb_bow_starboard_thruster, 10
        )
        self.subscriber_aft_port_thruster = self.create_subscription(
            geometry_msgs.msg.Wrench, "thruster/aft_port/command", self.cb_aft_port_thruster, 10
        )
        self.subscriber_aft_center_thruster = self.create_subscription(
            geometry_msgs.msg.Wrench, "thruster/aft_center/command", self.cb_aft_center_thruster, 10
        )
        self.subscriber_aft_starboard_thruster = self.create_subscription(
            geometry_msgs.msg.Wrench, "thruster/aft_starboard/command", self.cb_aft_starboard_thruster, 10
        )

        self.publisher_bow_port_thruster = self.create_publisher(
            geometry_msgs.msg.WrenchStamped, "thruster/bow_port/issued", 1
        )
        self.publisher_bow_center_thruster = self.create_publisher(
            geometry_msgs.msg.WrenchStamped, "thruster/bow_center/issued", 1
        )
        self.publisher_bow_starboard_thruster = self.create_publisher(
            geometry_msgs.msg.WrenchStamped, "thruster/bow_starboard/issued", 1
        )
        self.publisher_aft_port_thruster = self.create_publisher(
            geometry_msgs.msg.WrenchStamped, "thruster/aft_port/issued", 1
        )
        self.publisher_aft_center_thruster = self.create_publisher(
            geometry_msgs.msg.WrenchStamped, "thruster/aft_center/issued", 1
        )
        self.publisher_aft_starboard_thruster = self.create_publisher(
            geometry_msgs.msg.WrenchStamped, "thruster/aft_starboard/issued", 1
        )


    def cb_bow_port_thruster(self, msg: geometry_msgs.msg.Wrench):
        self.u[0] = np.clip(msg.force.x, -1.0, 1.0)
        self.u[1] = np.clip(msg.force.y, -1.0, 1.0)
        issued = geometry_msgs.msg.WrenchStamped()
        issued.header.frame_id = "bow_port_thruster_link"
        issued.header.stamp = self.get_clock().now().to_msg()
        issued.wrench.force.x = msg.force.x
        issued.wrench.force.y = msg.force.y
        self.publisher_bow_port_thruster.publish(issued)

    def cb_bow_center_thruster(self, msg: geometry_msgs.msg.Wrench):
        self.u[2] = np.clip(msg.force.x, -1.0, 1.0)
        self.u[3] = np.clip(msg.force.y, -1.0, 1.0)
        issued = geometry_msgs.msg.WrenchStamped()
        issued.header.frame_id = "bow_center_thruster_link"
        issued.header.stamp = self.get_clock().now().to_msg()
        issued.wrench.force.x = msg.force.x
        issued.wrench.force.y = msg.force.y
        self.publisher_bow_center_thruster.publish(issued)

    def cb_bow_starboard_thruster(self, msg: geometry_msgs.msg.Wrench):
        self.u[4] = np.clip(msg.force.x, -1.0, 1.0)
        self.u[5] = np.clip(msg.force.y, -1.0, 1.0)
        issued = geometry_msgs.msg.WrenchStamped()
        issued.header.frame_id = "bow_starboard_thruster_link"
        issued.header.stamp = self.get_clock().now().to_msg()
        issued.wrench.force.x = msg.force.x
        issued.wrench.force.y = msg.force.y
        self.publisher_bow_starboard_thruster.publish(issued)

    def cb_aft_port_thruster(self, msg: geometry_msgs.msg.Wrench):
        self.u[6] = np.clip(msg.force.x, -1.0, 1.0)
        self.u[7] = np.clip(msg.force.y, -1.0, 1.0)
        issued = geometry_msgs.msg.WrenchStamped()
        issued.header.frame_id = "aft_port_thruster_link"
        issued.header.stamp = self.get_clock().now().to_msg()
        issued.wrench.force.x = msg.force.x
        issued.wrench.force.y = msg.force.y
        self.publisher_aft_port_thruster.publish(issued)

    def cb_aft_center_thruster(self, msg: geometry_msgs.msg.Wrench):
        self.u[8] = np.clip(msg.force.x, -1.0, 1.0)
        self.u[9] = np.clip(msg.force.y, -1.0, 1.0)
        issued = geometry_msgs.msg.WrenchStamped()
        issued.header.frame_id = "aft_center_thruster_link"
        issued.header.stamp = self.get_clock().now().to_msg()
        issued.wrench.force.x = msg.force.x
        issued.wrench.force.y = msg.force.y
        self.publisher_aft_center_thruster.publish(issued)

    def cb_aft_starboard_thruster(self, msg: geometry_msgs.msg.Wrench):
        self.u[10] = np.clip(msg.force.x, -1.0, 1.0)
        self.u[11] = np.clip(msg.force.y, -1.0, 1.0)
        issued = geometry_msgs.msg.WrenchStamped()
        issued.header.frame_id = "aft_starboard_thruster_link"
        issued.header.stamp = self.get_clock().now().to_msg()
        issued.wrench.force.x = msg.force.x
        issued.wrench.force.y = msg.force.y
        self.publisher_aft_starboard_thruster.publish(issued)


def main(args=None):
    rclpy.init(args=args)
    simulator = DrillshipSimulator()
    rclpy.spin(simulator)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
