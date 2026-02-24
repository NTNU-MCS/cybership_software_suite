#!/usr/bin/env python3
"""
ROS2 Action Server for LOS Guidance using cybership_interfaces/LOSGuidance
"""
import math
import rclpy

from cybership_controller.guidance.ros_action_server_force_ctrl import LOSGuidanceROS
from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor


def main(args=None):
    rclpy.init(args=args)
    node = LOSGuidanceROS()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()
    executor.shutdown()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
