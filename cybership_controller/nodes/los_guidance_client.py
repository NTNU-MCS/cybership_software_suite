#!/usr/bin/env python3
"""
ROS2 Action Client for LOS Guidance
"""
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor

import numpy as np

from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from cybership_interfaces.action import LOSGuidance
from cybership_controller.guidance.ros_action_client import LOSGuidanceClient
import time

def main(args=None):
    rclpy.init(args=args)
    client = LOSGuidanceClient()
    # Example waypoints for testing
    while rclpy.ok():
        example_waypoints = [
            (0.0, 0.0),
            (np.random.normal(6.0, 0.5), 0.0),
            (np.random.normal(6.0, 0.5),np.random.normal(6.0, 0.5)),
            (0.0, np.random.normal(6.0, 0.5)),
        ]
        client.send_goal(example_waypoints)
        time.sleep(5.0)
        client.get_logger().info('Requesting goal cancellation...')
    # Use MultiThreadedExecutor to handle feedback continuously
    executor = MultiThreadedExecutor()
    executor.add_node(client)
    executor.spin()
    executor.shutdown()
    client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
