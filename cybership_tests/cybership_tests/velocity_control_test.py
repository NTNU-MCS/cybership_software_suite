#!/usr/bin/env python3

# ----------------------------------------------------------------------------
# This code is part of the Cybership Software Suite
# Created By: Emir Cem Gezer
# Created Date: 2025-03-26
# Copyright (C) 2025: NTNU, Trondheim
# Licensed under GPL-3.0-or-later
# ---------------------------------------------------------------------------

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import numpy as np
import argparse
import math
import time

class VelocityCommandPublisher(Node):
    """
    Node to publish test velocity commands to test the velocity controller.
    Publishes Twist messages with configurable patterns.
    """
    def __init__(self, test_type='constant', frequency=10.0):
        super().__init__("velocity_test_node")

        # Publisher for velocity commands
        self.vel_pub = self.create_publisher(
            Twist,
            '/voyager/control/velocity/command',
            10
        )

        # Set up timer for publishing velocity commands
        self.timer_period = 1.0 / frequency  # in seconds
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

        # Test configuration
        self.test_type = test_type
        self.start_time = time.time()
        self.duration = 0.0

        # Test parameters
        self.amplitude = {
            'surge': 0.1,  # m/s
            'sway': 0.0,   # m/s
            'yaw': 0.0,    # rad/s
        }
        self.frequency = {
            'surge': 0.1,  # Hz
            'sway': 0.05,  # Hz
            'yaw': 0.05,    # Hz
        }

        self.get_logger().info(f"Starting velocity test with pattern: {self.test_type}")
        self.get_logger().info(f"Publishing at {frequency} Hz")

    def timer_callback(self):
        """Publish velocity commands according to the selected test pattern"""
        msg = Twist()
        self.duration = time.time() - self.start_time

        if self.test_type == 'constant':
            # Constant velocity command
            msg.linear.x = self.amplitude['surge']
            msg.linear.y = self.amplitude['sway']
            msg.angular.z = self.amplitude['yaw']

        elif self.test_type == 'sine':
            # Sinusoidal velocity commands
            msg.linear.x = self.amplitude['surge'] * math.sin(2 * math.pi * self.frequency['surge'] * self.duration)
            msg.linear.y = self.amplitude['sway'] * math.sin(2 * math.pi * self.frequency['sway'] * self.duration)
            msg.angular.z = self.amplitude['yaw'] * math.sin(2 * math.pi * self.frequency['yaw'] * self.duration)

        elif self.test_type == 'step':
            # Step input (alternating between positive, zero, and negative values)
            period_surge = 1.0 / self.frequency['surge']
            period_sway = 1.0 / self.frequency['sway']
            period_yaw = 1.0 / self.frequency['yaw']

            # For each DOF: divide the period into 4 parts: +mag, 0, -mag, 0
            cycle_phase_surge = (self.duration % (4 * period_surge)) // period_surge
            cycle_phase_sway = (self.duration % (4 * period_sway)) // period_sway
            cycle_phase_yaw = (self.duration % (4 * period_yaw)) // period_yaw

            # Map phases to values: 0->+mag, 1->0, 2->-mag, 3->0
            surge_values = [self.amplitude['surge'], 0.0, -self.amplitude['surge'], 0.0]
            sway_values = [self.amplitude['sway'], 0.0, -self.amplitude['sway'], 0.0]
            yaw_values = [self.amplitude['yaw'], 0.0, -self.amplitude['yaw'], 0.0]

            msg.linear.x = surge_values[int(cycle_phase_surge)]
            msg.linear.y = sway_values[int(cycle_phase_sway)]
            msg.angular.z = yaw_values[int(cycle_phase_yaw)]

        elif self.test_type == 'circle':
            # Circular motion (constant sway velocity and yaw rate)
            msg.linear.x = self.amplitude['surge']
            msg.linear.y = 0.0
            msg.angular.z = self.amplitude['yaw']

        # Publish the message
        self.vel_pub.publish(msg)
        self.get_logger().debug(f"Published velocity: [{msg.linear.x:.2f}, {msg.linear.y:.2f}, {msg.angular.z:.2f}]")


def main(args=None):
    """Main function to initialize and run the velocity command publisher."""
    parser = argparse.ArgumentParser(description='Test the velocity controller with various patterns.')
    parser.add_argument('--pattern', type=str, default='step',
                        choices=['constant', 'sine', 'step', 'circle'],
                        help='Velocity command pattern (default: constant)')
    parser.add_argument('--frequency', type=float, default=10.0,
                        help='Publishing frequency in Hz (default: 10.0)')

    # Parse command line arguments before ROS initialization
    parsed_args, remaining_args = parser.parse_known_args(args)

    rclpy.init(args=remaining_args)
    node = VelocityCommandPublisher(test_type=parsed_args.pattern,
                                   frequency=parsed_args.frequency)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard interrupt, shutting down...")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
