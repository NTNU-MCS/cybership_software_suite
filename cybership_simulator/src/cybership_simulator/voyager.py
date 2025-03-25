#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from cybership_simulator.base import BaseSimulator
import numpy as np
import shoeboxpy.model6dof
import skadipy
import rclpy
import geometry_msgs.msg

class VoyagerSimulator(BaseSimulator):
    """
    Concrete simulator for the Voyager vessel.
    Defines vessel geometry, thruster configuration, and topic handling for thruster commands.
    """

    def __init__(self):
        super().__init__(node_name="voyager_simulator")

    def _create_vessel(self):
        return shoeboxpy.model6dof.Shoebox(
            L=1.0, B=0.3, T=0.08, GM_theta=0.02, GM_phi=0.02,
            eta0=np.array([0.0, 0.0, 0.0, 0.2, 0.2, 0.0]),
        )

    def _init_allocator(self):
        tunnel = skadipy.actuator.Fixed(
            position=skadipy.toolbox.Point([0.3875, 0.0, -0.01]),
            orientation=skadipy.toolbox.Quaternion(
                axis=(0.0, 0.0, 1.0), radians=np.pi / 2.0
            )
        )
        port_azimuth = skadipy.actuator.Azimuth(
            position=skadipy.toolbox.Point([-0.4574, -0.055, -0.1]),
        )
        starboard_azimuth = skadipy.actuator.Azimuth(
            position=skadipy.toolbox.Point([-0.4547, 0.055, -0.1]),
        )
        actuators = [tunnel, port_azimuth, starboard_azimuth]
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
        Sets up thruster publishers/subscriptions for Voyager.
        In this example, Voyager uses three thruster groups with a combined command vector of dimension 5.
        """
        # Initialize the thruster command vector (5 elements)
        self.u = np.zeros((5, 1))

        # Create subscriptions for thruster commands
        self.subscriber_tunnel_thruster = self.create_subscription(
            geometry_msgs.msg.Wrench, "thruster/tunnel/command", self.cb_tunnel_thruster, 10
        )
        self.subscriber_starboard_thruster = self.create_subscription(
            geometry_msgs.msg.Wrench, "thruster/starboard/command", self.cb_starboard_thruster, 10
        )
        self.subscriber_port_thruster = self.create_subscription(
            geometry_msgs.msg.Wrench, "thruster/port/command", self.cb_port_thruster, 10
        )

        # Create publishers to issue thruster commands (if needed)
        self.publisher_tunnel_thruster = self.create_publisher(
            geometry_msgs.msg.WrenchStamped, "thruster/tunnel/issued", 1
        )
        self.publisher_starboard_thruster = self.create_publisher(
            geometry_msgs.msg.WrenchStamped, "thruster/starboard/issued", 1
        )
        self.publisher_port_thruster = self.create_publisher(
            geometry_msgs.msg.WrenchStamped, "thruster/port/issued", 1
        )

    # Thruster command callbacks
    def cb_tunnel_thruster(self, msg: geometry_msgs.msg.Wrench):
        self.u[0] = msg.force.x

        if np.linalg.norm(self.u[0]) < 0.1:
            self.u[0] = 0.0

        issued = geometry_msgs.msg.WrenchStamped()
        issued.header.frame_id = "bow_tunnel_thruster_link"
        issued.header.stamp = self.get_clock().now().to_msg()
        issued.wrench.force.x = msg.force.x
        self.publisher_tunnel_thruster.publish(issued)

    def cb_starboard_thruster(self, msg: geometry_msgs.msg.Wrench):
        self.u[1] = np.clip(msg.force.x, -1.0, 1.0)
        self.u[2] = np.clip(msg.force.y, -1.0, 1.0)

        # Apply a deadband to the thruster commands
        if np.linalg.norm(self.u[1:3]) < 0.1:
            self.u[1] = 0.0
            self.u[2] = 0.0

        issued = geometry_msgs.msg.WrenchStamped()
        issued.header.frame_id = "stern_starboard_thruster_link"
        issued.header.stamp = self.get_clock().now().to_msg()
        issued.wrench.force.x = msg.force.x
        issued.wrench.force.y = msg.force.y
        self.publisher_starboard_thruster.publish(issued)

    def cb_port_thruster(self, msg: geometry_msgs.msg.Wrench):
        self.u[3] = np.clip(msg.force.x, -1.0, 1.0)
        self.u[4] = np.clip(msg.force.y, -1.0, 1.0)

        if np.linalg.norm(self.u[3:5]) < 0.1:
            self.u[3] = 0.0
            self.u[4] = 0.0

        issued = geometry_msgs.msg.WrenchStamped()
        issued.header.frame_id = "stern_port_thruster_link"
        issued.header.stamp = self.get_clock().now().to_msg()
        issued.wrench.force.x = msg.force.x
        issued.wrench.force.y = msg.force.y
        self.publisher_port_thruster.publish(issued)

# ----------------------------------------------------------------------------
# Main entry point
# ----------------------------------------------------------------------------

def main(args=None):
    rclpy.init(args=args)
    simulator = VoyagerSimulator()
    rclpy.spin(simulator)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
