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
            eta0=self.eta0.flatten(),
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
        # This order is important when unpacking the command vector
        actuators = [
            tunnel,
            port_azimuth,
            starboard_azimuth
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
        if np.linalg.norm(self.u[0]) < 0.05:
            self.u[0] = 0.0
        issued = geometry_msgs.msg.WrenchStamped()
        issued.header.frame_id = self._frame("bow_tunnel_thruster_link")
        issued.header.stamp = self.get_clock().now().to_msg()
        issued.wrench.force.x = msg.force.x
        self.publisher_tunnel_thruster.publish(issued)

    def cb_port_thruster(self, msg: geometry_msgs.msg.Wrench):
        fx = np.clip(msg.force.x, -1.0, 1.0)
        fy = np.clip(msg.force.y, -1.0, 1.0)
        # Apply a deadband to the thruster commands
        if np.linalg.norm([fx, fy]) < 0.1:
            fx = 0.0
            fy = 0.0
        self.u[1] = fx
        self.u[2] = fy
        issued = geometry_msgs.msg.WrenchStamped()
        issued.header.frame_id = self._frame("stern_port_thruster_link")
        issued.header.stamp = self.get_clock().now().to_msg()
        issued.wrench.force.x = fx
        issued.wrench.force.y = fy
        self.publisher_port_thruster.publish(issued)

    def cb_starboard_thruster(self, msg: geometry_msgs.msg.Wrench):
        fx = np.clip(msg.force.x, -1.0, 1.0)
        fy = np.clip(msg.force.y, -1.0, 1.0)
        # Apply a deadband to the thruster commands
        if np.linalg.norm([fx, fy]) < 0.1:
            fx = 0.0
            fy = 0.0
        self.u[3] = fx
        self.u[4] = fy
        issued = geometry_msgs.msg.WrenchStamped()
        issued.header.frame_id = self._frame("stern_starboard_thruster_link")
        issued.header.stamp = self.get_clock().now().to_msg()
        issued.wrench.force.x = fx
        issued.wrench.force.y = fy
        self.publisher_starboard_thruster.publish(issued)

# ----------------------------------------------------------------------------
# Main entry point
# ----------------------------------------------------------------------------


def main(args=None):
    rclpy.init(args=args)

    simulator = VoyagerSimulator()

    try:
        rclpy.spin(simulator)
    except KeyboardInterrupt:
        pass
    finally:
        # Clean shutdown
        simulator.destroy_node()
        rclpy.shutdown()

    # # Create a multithreaded executor
    # executor = rclpy.executors.MultiThreadedExecutor()
    # executor.add_node(simulator)

    # try:
    #     executor.spin()
    # finally:
    #     # Clean shutdown
    #     simulator.destroy_node()
    #     rclpy.shutdown()


if __name__ == "__main__":
    main()
