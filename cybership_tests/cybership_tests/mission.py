#!/usr/bin/env python3


import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool
from topic_tools_interfaces.srv import MuxSelect
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
import math
import time
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from tf_transformations import quaternion_from_euler
import numpy as np


class ActionRunner(Node):
    def __init__(self):
        # Node namespace set at initialization; service and topic names are relative
        super().__init__('action_runner', namespace='cybership')
        # Define the mission actions

        self.rng = np.random.default_rng()

        self.actions = [
            {
                'action': 'pose',
                'parameters': {'x': -2.0, 'y': -2.0, 'yaw': np.pi / 4},
                'duration': 10.0,
            },
            {
                'action': 'velocity',
                'parameters': {
                    # Sample linear and angular components each publish
                    'linear_x': lambda: self.rng.uniform(0.4, 0.6),
                    'linear_y': lambda: self.rng.uniform(-0.1, 0.1),
                    'angular_z': lambda: self.rng.uniform(-0.1, 0.1)
                },
                'duration': 30.0,
            },
            {
                'action': 'wait',
                'duration': 5.0,
            },
            {
                'action': 'pose',
                'parameters': {'x': 2.0, 'y': 2.0, 'yaw': 5 * np.pi / 4},
                'duration': 10.0,
            },
            {
                'action': 'velocity',
                'parameters': {
                    # Sample linear and angular components each publish
                    'linear_x': lambda: self.rng.uniform(0.4, 0.6),
                    'linear_y': lambda: self.rng.uniform(-0.1, 0.1),
                    'angular_z': lambda: self.rng.uniform(-0.1, 0.1)
                },
                'duration': 30.0,
            },
            {
                'action': 'wait',
                'duration': 5.0,
            }
        ]
        self.repeat = 5

        # Publisher for velocity controller input
        self.vel_pub = self.create_publisher(
            Twist, 'control/velocity/command', 10)

        # Mux select service client (relative to node namespace)
        self.mux_client = self.create_client(MuxSelect, 'force_mux/select')
        self.get_logger().info('Waiting for mux select service...')
        self.mux_client.wait_for_service()
        # Initialize odometry storage
        self.current_odom = None
        # Subscribe to odometry to monitor vehicle pose
        self.create_subscription(
            Odometry, 'measurement/odom', self.odom_callback, 10)
        self.get_logger().info('Subscribed to measurement/odom')
        # Action client for pose navigation (relative to node namespace)
        self.pose_action_client = ActionClient(
            self, NavigateToPose, 'navigate_to_pose')
        self.get_logger().info('Waiting for NavigateToPose action server...')
        self.pose_action_client.wait_for_server()

        # Enable/disable service clients (service names to be set later)
        POSE_ENABLE_SERVICE = 'position_controller/change_state'
        VELOCITY_ENABLE_SERVICE = 'velocity_controller/change_state'

        if POSE_ENABLE_SERVICE:
            self.pose_enable_client = self.create_client(
                SetBool, POSE_ENABLE_SERVICE)
            self.pose_enable_client.wait_for_service()
        else:
            self.pose_enable_client = None

        if VELOCITY_ENABLE_SERVICE:
            self.vel_enable_client = self.create_client(
                SetBool, VELOCITY_ENABLE_SERVICE)
            self.vel_enable_client.wait_for_service()
        else:
            self.vel_enable_client = None

    def call_mux(self, topic: str):
        req = MuxSelect.Request()
        req.topic = topic
        future = self.mux_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        if future.result():
            self.get_logger().info(f'Mux switched to {topic}')
        else:
            self.get_logger().error(f'Failed to switch mux to {topic}')

    def set_enable(self, client, enable: bool, name: str):
        if client is None:
            return
        req = SetBool.Request()
        req.data = enable
        future = client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        if future.result().success:
            self.get_logger().info(f'Service {name} set enable={enable}')
        else:
            self.get_logger().error(f'Failed to set {name} enable={enable}')

    def send_pose_goal(self, x: float, y: float, yaw: float):
        """
        Send a NavigateToPose action goal with given x, y, yaw.
        """
        # Build goal message
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'world'
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = 0.0
        # Orientation quaternion
        quat = quaternion_from_euler(0.0, 0.0, yaw)
        msg.pose.orientation.x = quat[0]
        msg.pose.orientation.y = quat[1]
        msg.pose.orientation.z = quat[2]
        msg.pose.orientation.w = quat[3]

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = msg
        self.get_logger().info(
            f'Sending NavigateToPose goal: x={x}, y={y}, yaw={yaw}')
        # Send goal
        goal_future = self.pose_action_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, goal_future)
        goal_handle = goal_future.result()
        if not goal_handle.accepted:
            self.get_logger().error('NavigateToPose goal was rejected')
            return
        # Wait for result
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result().result
        self.get_logger().info('NavigateToPose action completed')

    def run_pose_action(self, params, duration):
        """Execute a pose action in a loop, monitoring odometry and abort conditions."""
        # Activate pose controller and send goal
        self.call_mux('control/force/command/position')
        self.set_enable(self.pose_enable_client, True, 'pose')
        self.send_pose_goal(params['x'], params['y'], params['yaw'])
        # Monitor until duration expires
        start = self.get_clock().now().nanoseconds / 1e9
        while rclpy.ok() and (self.get_clock().now().nanoseconds / 1e9 - start) < duration:
            if self.current_odom:
                pos = self.current_odom.pose.pose.position
                self.get_logger().debug(
                    f'Pose action - current pos: ({pos.x:.2f}, {pos.y:.2f})')
            rclpy.spin_once(self, timeout_sec=0.1)
        # Deactivate pose controller
        self.set_enable(self.pose_enable_client, False, 'pose')

    def run_velocity_action(self, params, duration):
        """Execute a velocity action in a loop, sending velocity commands continuously."""
        # Activate velocity controller
        self.call_mux('control/force/command/velocity')
        self.set_enable(self.vel_enable_client, True, 'velocity')
        # Loop and publish, sampling parameters if callable
        msg = Twist()
        lx = params['linear_x']() if callable(params.get(
            'linear_x')) else params.get('linear_x', 0.0)
        ly = params['linear_y']() if callable(params.get(
            'linear_y')) else params.get('linear_y', 0.0)
        az = params['angular_z']() if callable(params.get(
            'angular_z')) else params.get('angular_z', 0.0)
        self.get_logger().info(
            f'Velocity action (u={lx:.2f}, v={ly:.2f}, r={az:.2f})')
        start = self.get_clock().now().nanoseconds / 1e9
        while rclpy.ok() and (self.get_clock().now().nanoseconds / 1e9 - start) < duration:
            # Sample velocity parameters if provided as callables

            msg.linear.x = lx
            msg.linear.y = ly
            msg.angular.z = az
            self.vel_pub.publish(msg)
            if self.current_odom:
                pos = self.current_odom.pose.pose.position

            rclpy.spin_once(self, timeout_sec=0.1)
        # Deactivate velocity controller
        self.set_enable(self.vel_enable_client, False, 'velocity')

    def run_wait_action(self, duration):
        """Simple wait loop, still processing callbacks."""
        start = self.get_clock().now().nanoseconds / 1e9
        while rclpy.ok() and (self.get_clock().now().nanoseconds / 1e9 - start) < duration:
            rclpy.spin_once(self, timeout_sec=0.1)

    def execute_actions(self):

        for _ in range(self.repeat):

            for action in self.actions:
                act = action.get('action')
                params = action.get('parameters', {})
                duration = action.get('duration', 0.0)
                # Disable both controllers
                self.set_enable(self.pose_enable_client, False, 'pose')
                self.set_enable(self.vel_enable_client, False, 'velocity')
                # Execute action-specific loop
                if act == 'pose':
                    self.get_logger().info(
                        f'Starting pose action for {duration}s')
                    self.run_pose_action(params, duration)
                elif act == 'velocity':
                    self.get_logger().info(
                        f'Starting velocity action for {duration}s')
                    self.run_velocity_action(params, duration)
                elif act == 'wait':
                    self.get_logger().info(f'Running wait for {duration}s')
                    self.run_wait_action(duration)
                else:
                    self.get_logger().warn(f'Unknown action: {act}')

    def odom_callback(self, msg):
        # Store and log the current vehicle pose from odometry data
        self.current_odom = msg


def main(args=None):
    rclpy.init(args=args)
    node = ActionRunner()
    node.execute_actions()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
