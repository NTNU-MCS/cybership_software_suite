#!/usr/bin/env python3
"""
ROS2 Action Client for LOS Guidance
"""
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor

from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from cybership_interfaces.action import LOSGuidance

class LOSGuidanceClient(Node):
    def __init__(self):
        super().__init__('los_guidance_client')
        self._action_client = ActionClient(self, LOSGuidance, 'los_guidance')

    def send_goal(self, waypoints):
        goal_msg = LOSGuidance.Goal()
        path_msg = Path()
        path_msg.header.frame_id = 'world'
        path_msg.header.stamp = self.get_clock().now().to_msg()
        for x, y in waypoints:
            pose = PoseStamped()
            pose.header.frame_id = 'world'
            pose.header.stamp = path_msg.header.stamp
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = 0.0
            pose.pose.orientation.w = 1.0
            path_msg.poses.append(pose)
        goal_msg.path = path_msg

        self.get_logger().info('Waiting for action server...')
        if not self._action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Action server not available, shutting down')
            rclpy.shutdown()
            return

        self.get_logger().info('Sending goal...')
        send_goal_future = self._action_client.send_goal_async(
            goal_msg, feedback_callback=self.feedback_callback)
        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return
        self.get_logger().info('Goal accepted :)')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.get_result_callback)

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        heading_deg = feedback.heading * 180.0 / 3.141592653589793
        vel = feedback.vel_cmd
        self.get_logger().info(
            f'Feedback: Heading={heading_deg:.1f}°, Vel=[{vel.x:.2f}, {vel.y:.2f}, {vel.z:.2f}]')

    def get_result_callback(self, future):
        result = future.result().result
        success = result.success
        self.get_logger().info(f'Result: success={success}')
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    client = LOSGuidanceClient()
    # Example waypoints for testing
    example_waypoints = [
        (5.0, 5.0),
        (-5.0, 5.0),
        (-5.0, -5.0),
        (5.0, -5.0)
    ]
    client.send_goal(example_waypoints)
    # Use MultiThreadedExecutor to handle feedback continuously
    executor = MultiThreadedExecutor()
    executor.add_node(client)
    executor.spin()
    executor.shutdown()
    client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
