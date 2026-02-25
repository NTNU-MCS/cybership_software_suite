#!/usr/bin/env python3
"""
ROS2 Action Server for LOS Guidance using cybership_interfaces/LOSGuidance
"""
import math
import rclpy

# from cybership_controller.guidance.ros_action_server_force_ctrl import LOSGuidanceROS
from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor

from cybership_controller.guidance.ros_action_server_consolidated import *

def main(args=None):
    rclpy.init(args=args)
    node = LOSGuidanceForceCtrROS()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
