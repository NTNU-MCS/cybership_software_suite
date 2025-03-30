#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient

class NavigateToPoseClient(Node):
    def __init__(self):
        super().__init__('navigate_to_pose_client', namespace='voyager')
        # Create action client to /voyager/navigate_to_pose
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        # Subscribe to the /goal_pose topic
        self.create_subscription(PoseStamped, '/goal_pose', self.goal_pose_callback, 10)
        # Variable to hold the current goal handle
        self._goal_handle = None

    def goal_pose_callback(self, msg: PoseStamped):
        self.get_logger().info('Received new goal pose message.')

        # If there's an active goal, cancel it
        if self._goal_handle is not None:
            cancel_future = self._goal_handle.cancel_goal_async()
            cancel_future.add_done_callback(lambda fut: self.get_logger().info('Cancelled current goal.'))
            self._goal_handle = None

        # Wait for the action server to be available before sending a goal
        if not self._action_client.wait_for_server(timeout_sec=1.0):
            self._
            self.get_logger().error('Action server not available!')
            return

        # Prepare the goal message
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = msg

        # Send goal asynchronously with a feedback callback
        send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected by the action server.')
            return
        self.get_logger().info('Goal accepted, waiting for result...')
        self._goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.get_result_callback)

    def feedback_callback(self, feedback_msg):
        # Log feedback from the action server
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Feedback: {feedback.distance_remaining} meters remaining.')

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Action result: {result}')
        self._goal_handle = None

def main(args=None):
    rclpy.init(args=args)
    node = NavigateToPoseClient()
    # Use a MultiThreadedExecutor so callbacks can run concurrently
    executor = rclpy.executors.MultiThreadedExecutor()
    rclpy.spin(node, executor)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
