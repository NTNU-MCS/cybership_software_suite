#!/usr/bin/env python3

import rclpy
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
import math
import time
import numpy as np


def quaternion_from_yaw(yaw):
    """
    Convert a yaw angle (in radians) into a quaternion.
    """
    # Since roll and pitch are assumed to be 0:
    qx = 0.0
    qy = 0.0
    qz = math.sin(yaw / 2.0)
    qw = math.cos(yaw / 2.0)
    return [qx, qy, qz, qw]


def feedback_callback(feedback_msg):
    feedback = feedback_msg.feedback
    print(f"Feedback: remaining distance = {feedback.distance_remaining:.2f}")


def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node("point_sequencer", namespace="voyager")
    action_client = ActionClient(node, NavigateToPose, "navigate_to_pose")

    # Define the sequence of target points as (x, y, yaw)
    target_points = np.array(
        [
            [0, 0, 0],
            [1, 0, 0],
            [1, 1, 0],
            [1, 1, math.pi / 4],
            [0, 1, math.pi / 4],
            [0, 0, 0],
        ],
        dtype=float,
    )
    # Shift the points to the right and down by 1.5 meters and scale by 3
    target_points[:, 0:2] *= 3
    target_points[:, 0:2] -= 1.5

    node.get_logger().info("Waiting for action server...")
    if not action_client.wait_for_server(timeout_sec=10.0):
        node.get_logger().error("Action server not available!")
        rclpy.shutdown()
        return

    for x, y, yaw in target_points:
        node.get_logger().info(f"Sending goal: x={x}, y={y}, yaw={yaw:.2f}")
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.frame_id = (
            "world"  # Use the correct frame for your application
        )
        goal_msg.pose.header.stamp = node.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.position.z = 0.0

        q = quaternion_from_yaw(yaw)
        goal_msg.pose.pose.orientation.x = q[0]
        goal_msg.pose.pose.orientation.y = q[1]
        goal_msg.pose.pose.orientation.z = q[2]
        goal_msg.pose.pose.orientation.w = q[3]

        # Send the goal with a feedback callback.
        future_goal = action_client.send_goal_async(
            goal_msg, feedback_callback=feedback_callback
        )
        rclpy.spin_until_future_complete(node, future_goal)
        goal_handle = future_goal.result()

        if not goal_handle.accepted:
            node.get_logger().info("Goal rejected")
            continue

        node.get_logger().info("Goal accepted, waiting for result...")
        future_result = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(node, future_result)
        result = future_result.result().result
        node.get_logger().info(f"Result: {result}")
        # Optionally wait a moment before sending the next goal.
        time.sleep(10.0)

    node.get_logger().info("All goals completed. Shutting down.")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
