#!/usr/bin/env python3


import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool
from topic_tools_interfaces.srv import MuxSelect, MuxAdd
from geometry_msgs.msg import PoseStamped, Twist, Wrench
from nav_msgs.msg import Odometry
import math
import time
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from tf_transformations import quaternion_from_euler
import numpy as np
from typing import Any, Dict, Optional, List

# Topic and service name constants
TOPIC_FORCE_CONSTANT: str = 'control/force/command/constant'
TOPIC_VELOCITY_COMMAND: str = 'control/velocity/command'
TOPIC_ODOM: str = 'measurement/odom'
TOPIC_POSE_GOAL: str = 'navigate_to_pose'
TOPIC_FORCE_POSITION: str = 'control/force/command/position'
TOPIC_FORCE_VELOCITY: str = 'control/force/command/velocity'
TOPIC_FORCE_BASE: str = 'control/force/command'

SERVICE_MUX_ADD: str = 'force_mux/add'
SERVICE_MUX_SELECT: str = 'force_mux/select'
SERVICE_POSE_ENABLE: str = 'position_controller/change_state'
SERVICE_VEL_ENABLE: str = 'velocity_controller/change_state'


class ActionRunner(Node):
    """
    ActionRunner executes a predefined sequence of navigation, velocity, force, and wait actions
    on a ROS2 node, monitoring odometry for abort conditions.

    Attributes:
        repeat (int): Number of times to repeat the full action sequence.
        current_odom (Odometry): Latest odometry message for pose feedback.
    """
    def __init__(self):
        """
        Initialize the ActionRunner node, including publishers, subscribers, action and service clients.

        Sets up:
            - Mux for force control topics
            - Publishers for velocity (Twist) and force (Wrench) commands
            - Subscriber for odometry (Odometry)
            - ActionClient for NavigateToPose
            - Service clients for enabling/disabling pose and velocity controllers
        """
        super().__init__('action_runner', namespace='cybership')
        # Define the list of mission actions
        self.add_mux(TOPIC_FORCE_CONSTANT)

        self.repeat = 500

        self.vel_pub = self.create_publisher(
            Twist, TOPIC_VELOCITY_COMMAND, 10)
        self.force_pub = self.create_publisher(
            Wrench, TOPIC_FORCE_CONSTANT, 10)

        # Initialize storage for odometry messages
        self.current_odom = None
        # Subscribe to '/measurement/odom' topic for odometry feedback
        self.create_subscription(
            Odometry, TOPIC_ODOM, self.odom_callback, 10)
        self.get_logger().info('Subscribed to measurement/odom')

        # Action client for pose navigation (relative to node namespace)
        self.pose_action_client = ActionClient(
            self, NavigateToPose, TOPIC_POSE_GOAL)
        self.get_logger().info('Waiting for NavigateToPose action server...')
        self.pose_action_client.wait_for_server()

        # Enable/disable service clients (service names to be set later)
        POSE_ENABLE_SERVICE = SERVICE_POSE_ENABLE
        VELOCITY_ENABLE_SERVICE = SERVICE_VEL_ENABLE

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

        self.rng = np.random.default_rng()

        self.wait_until_ready()

        # Default abort lands back at origin after timeout
        self.default_abort_action = {
            'name': 'default_abort',
            'action': self.run_pose_action,
            'parameters': {'x': 0.0, 'y': 0.0, 'yaw': 0.0, 'duration': 60.0, 'grace': 15.0},
        }

        self.actions = [
            {
                'name': 'pose_neg_diag',
                'action': self.run_pose_action,
                'parameters': {
                    'x': -2.0,
                    'y': -2.0,
                    'yaw': np.pi / 4,
                    'duration': 60.0
                },
            },
            {
                'name': 'velocity_random_1',
                'action': self.run_velocity_action,
                'parameters': {
                    # Sample linear and angular components each publish
                    'linear_x': lambda: self.rng.uniform(0.2, 0.4),
                    'linear_y': lambda: self.rng.uniform(-0.1, 0.1),
                    'angular_z': lambda: self.rng.uniform(-0.1, 0.1),
                    'duration': 30.0,
                    'margin': 0.1  # Allow some margin for early success
                },
                'abort_action': {
                    'name': 'velocity_random_1_abort',
                    'action': self.run_pose_action,
                    'parameters': {
                        'x': 2.0,
                        'y': 2.0,
                        'yaw': 5 * np.pi / 4,
                        'duration': 60.0,
                        'grace': 15.0
                    }
                }
            },
            {
                'name': 'force_random_1',
                'action': self.run_force_action,
                'parameters': {
                    'force_x': lambda: self.rng.uniform(0.2, 0.8),
                    'force_y': lambda: self.rng.uniform(-0.5, 0.5),
                    'torque_z': lambda: self.rng.uniform(-0.1, 0.1),
                    'duration': 30.0,
                },
                'abort_action': {
                    'name': 'force_random_1_abort',
                    'action': self.run_pose_action,
                    'parameters': {
                        'x': 2.0,
                        'y': 2.0,
                        'yaw': 5 * np.pi / 4,
                        'duration': 60.0,
                        'grace': 15.0
                    }
                }
            },
            {
                'name': 'pose_pos_diag',
                'action': self.run_pose_action,
                'parameters': {
                    'x': 2.0,
                    'y': 2.0,
                    'yaw': 5 * np.pi / 4,
                    'duration': 60.0
                },
            },
            {
                'name': 'velocity_random_2',
                'action': self.run_velocity_action,
                'parameters': {
                    # Sample linear and angular components each publish
                    'linear_x': lambda: self.rng.uniform(0.2, 0.4),
                    'linear_y': lambda: self.rng.uniform(-0.1, 0.1),
                    'angular_z': lambda: self.rng.uniform(-0.1, 0.1),
                    'duration': 30.0,
                    'margin': 0.1  # Allow some margin for early success
                },
                'abort_action': {
                    'name': 'velocity_random_2_abort',
                    'action': self.run_pose_action,
                    'parameters': {
                        'x': -2.0,
                        'y': -2.0,
                        'yaw': np.pi / 4,
                        'duration': 60.0,
                        'grace': 15.0,
                    }
                }
            },
            {
                'name': 'force_random_2',
                'action': self.run_force_action,
                'parameters': {
                    'force_x': lambda: self.rng.uniform(0.2, 0.8),
                    'force_y': lambda: self.rng.uniform(-0.5, 0.5),
                    'torque_z': lambda: self.rng.uniform(-0.1, 0.1),
                    'duration': 30.0,
                },
                'abort_action': {
                    'name': 'force_random_2_abort',
                    'action': self.run_pose_action,
                    'parameters': {
                        'x': -2.0,
                        'y': -2.0,
                        'yaw': np.pi / 4,
                        'duration': 60.0,
                        'grace': 15.0
                    }
                }
            },
            {
                'name': 'wait',
                'action': self.run_wait_action,
                'parameters': {'duration': 10.0}
            }
        ]

    def wait_until_ready(self) -> None:
        """
        Block until the first odometry message is received.

        Ensures the vehicle has a valid initial pose before performing actions.
        """
        while rclpy.ok() and self.current_odom is None:
            self.get_logger().info('Waiting for initial odometry data...')
            rclpy.spin_once(self, timeout_sec=0.5)
        self.get_logger().info('Initial odometry data received.')

    def add_mux(self, topic: str) -> None:
        """
        Add a topic to the force multiplexer using MuxAdd service.

        Args:
            topic (str): ROS topic name to add to the mux.
        """
        req = MuxAdd.Request()
        mux_add_client = self.create_client(MuxAdd, SERVICE_MUX_ADD)
        req.topic = topic
        future = mux_add_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        # MuxAdd returns a result with a success flag
        if hasattr(future.result(), 'success') and future.result().success:
            self.get_logger().info(f'Mux added topic {topic}')
        else:
            self.get_logger().error(f'Failed to add mux topic {topic}')

    def abort_condition(self) -> bool:
        """
        Determine if the mission should abort based on odometry limits.

        Returns:
            bool: True if |x| or |y| position exceeds 2.5 meters, False otherwise.
        """
        return abs(self.current_odom.pose.pose.position.x) > 2.5 or abs(self.current_odom.pose.pose.position.y) > 2.5

    def call_mux(self, topic: str) -> None:
        """
        Select the active force control topic on the mux.

        Args:
            topic (str): ROS topic name to select for force commands.
        """
        mux_client = self.create_client(MuxSelect, SERVICE_MUX_SELECT)
        mux_client.wait_for_service()
        self.get_logger().info(f'Calling mux select for topic {topic}')
        req = MuxSelect.Request()
        req.topic = topic
        future = mux_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        if future.result():
            self.get_logger().info(f'Mux switched to {topic}')
        else:
            self.get_logger().error(f'Failed to switch mux to {topic}')

    def set_enable(self, client: Optional[Any], enable: bool, name: str) -> None:
        """
        Enable or disable a controller via SetBool service.

        Args:
            client (rclpy.client.Client): Service client for SetBool.
            enable (bool): True to enable, False to disable.
            name (str): Human-readable name of the controller.
        """
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

    def _pose_feedback_callback(self, feedback_msg: Any) -> None:
        """
        Callback to log remaining distance from a NavigateToPose action.

        Args:
            feedback_msg: Action feedback message containing distance_remaining (float, meters).
        """
        fb = feedback_msg.feedback
        # nav2 NavigateToPose feedback has distance_remaining
        dist = getattr(fb, 'distance_remaining', None)
        if dist is not None:
            self.get_logger().info(f'    NavigateToPose feedback: distance_remaining={dist:.2f}')
        else:
            self.get_logger().info('    NavigateToPose feedback received')

    def run_pose_action(self, **params: Any) -> None:
        """
        Navigate to a target pose and monitor for timeouts or abort conditions.

        Sends a NavigateToPose goal and cancels if duration elapsed or abort condition met.

        Args:
            x (float): Target x-position in meters.
            y (float): Target y-position in meters.
            yaw (float): Target yaw angle in radians.
            duration (float): Max navigation time in seconds.
            grace (float): Grace period before starting abort checks in seconds.

        Raises:
            RuntimeError: If abort_condition() returns True after grace period.
        """
        # Activate pose controller via mux & service
        self.call_mux(TOPIC_FORCE_POSITION)
        self.set_enable(self.pose_enable_client, True, 'pose')

        # Extract parameters
        x = params.get('x', 0.0)
        y = params.get('y', 0.0)
        yaw = params.get('yaw', 0.0)
        duration = params.get('duration', 0.0)
        grace = params.get('grace', 0.0)
        self.get_logger().info(
            f'Pose action start (x={x:.2f}, y={y:.2f}, yaw={yaw:.2f}), '
            f'duration={duration:.1f}s, grace={grace:.1f}s'
        )

        # Build goal message
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'world'
        msg.pose.position.x = x
        msg.pose.position.y = y
        q = quaternion_from_euler(0.0, 0.0, yaw)
        msg.pose.orientation.x = q[0]
        msg.pose.orientation.y = q[1]
        msg.pose.orientation.z = q[2]
        msg.pose.orientation.w = q[3]
        goal = NavigateToPose.Goal()
        goal.pose = msg

        # Send goal with feedback callback
        goal_fut = self.pose_action_client.send_goal_async(
            goal, feedback_callback=self._pose_feedback_callback
        )
        rclpy.spin_until_future_complete(self, goal_fut)
        handle = goal_fut.result()
        if not handle.accepted:
            self.get_logger().error('NavigateToPose goal rejected')
            self.set_enable(self.pose_enable_client, False, 'pose')
            return

        self.get_logger().info('NavigateToPose goal accepted, monitoring execution')

        # Wait for result, checking abort & timeout
        result_fut = handle.get_result_async()
        start = self.get_clock().now().nanoseconds / 1e9
        elapsed = 0.0
        while rclpy.ok():
            elapsed = (self.get_clock().now().nanoseconds / 1e9) - start

            if elapsed >= duration:
                self.get_logger().info('NavigateToPose action completed due to duration')
                cancel_fut = handle.cancel_goal_async()
                rclpy.spin_until_future_complete(self, cancel_fut)
                self.set_enable(self.pose_enable_client, False, 'pose')
                break

            if result_fut.done():
                self.get_logger().info('NavigateToPose action completed')
                break

            if elapsed >= grace and self.abort_condition():

                cancel_fut = handle.cancel_goal_async()
                rclpy.spin_until_future_complete(self, cancel_fut)
                self.set_enable(self.pose_enable_client, False, 'pose')
                raise RuntimeError('Abort condition met during pose action, aborting')

            rclpy.spin_once(self, timeout_sec=0.5)

        # Deactivate pose controller
        self.set_enable(self.pose_enable_client, False, 'pose')

    def run_velocity_action(self, **params: Any) -> None:
        """
        Publish constant velocity commands for a given duration, with optional early stop.

        Args:
            linear_x (float): Desired linear velocity in x (m/s).
            linear_y (float): Desired linear velocity in y (m/s).
            angular_z (float): Desired angular velocity around z (rad/s).
            duration (float): Total time to publish commands (seconds).
            margin (float, optional): Early stop threshold for actual vs commanded velocity (m/s).
            grace (float): Grace period before starting abort checks (seconds).

        Raises:
            RuntimeError: If abort_condition() returns True after grace period.
        """
        # Activate velocity controller
        self.call_mux(TOPIC_FORCE_VELOCITY)
        self.set_enable(self.vel_enable_client, True, 'velocity')

        # Prepare and log command
        msg = Twist()
        lx = params.get('linear_x', 0.0)
        ly = params.get('linear_y', 0.0)
        az = params.get('angular_z', 0.0)
        margin = params.get('margin', None)
        self.get_logger().info('    '
            f'Velocity action (u={lx:.4f}, v={ly:.4f}, r={az:.4f})'
                               + (f', margin={margin:.4f}' if margin is not None else ''))

        # Extract duration, grace period and start time
        duration = params.get('duration', 0.0)
        grace = params.get('grace', 0.0)
        start = self.get_clock().now().nanoseconds / 1e9
        elapsed = 0.0

        while rclpy.ok() and elapsed < duration:
            elapsed = self.get_clock().now().nanoseconds / 1e9 - start
            # Publish the commanded velocity
            msg.linear.x = lx
            msg.linear.y = ly
            msg.angular.z = az
            self.vel_pub.publish(msg)

            # Feedback: log current odometry state
            if self.current_odom:
                pos = self.current_odom.pose.pose.position
                vel = self.current_odom.twist.twist.linear
                self.get_logger().info(
                    '    '
                    f'Velocity feedback: pos=({pos.x:.2f}, {pos.y:.2f}), '
                    f'vel=({vel.x:.2f}, {vel.y:.2f})'
                )

            # margin-based early success
            if margin is not None:
                act = self.current_odom.twist.twist
                if (abs(act.linear.x - lx) < margin and
                        abs(act.linear.y - ly) < margin):
                    self.get_logger().info('Velocity within margin, stopping early')
                    break

            # Abort check after grace period
            if self.current_odom and elapsed >= grace and self.abort_condition():
                raise RuntimeError('Abort condition met during velocity action, aborting')

            rclpy.spin_once(self, timeout_sec=0.5)
            # actual sleep to enforce loop period
            time.sleep(0.5)

        # Deactivate velocity controller
        self.set_enable(self.vel_enable_client, False, 'velocity')

    def run_wait_action(self, **params: Any) -> None:
        """
        Wait for a specified duration while processing ROS callbacks.

        This loop periodically checks for an abort condition after an optional grace period
        and logs odometry feedback during the wait.

        Args:
            duration (float): Total wait time in seconds.
            grace (float): Grace period before starting abort checks in seconds.

        Raises:
            RuntimeError: If abort_condition() returns True after grace period.
        """
        duration = params.get('duration', 0.0)
        grace = params.get('grace', 0.0)
        start = self.get_clock().now().nanoseconds / 1e9

        self.get_logger().info(f'Waiting for {duration:.4f} seconds')
        elapsed = 0.0
        while rclpy.ok() and elapsed < duration:
            elapsed = self.get_clock().now().nanoseconds / 1e9 - start

            if elapsed >= grace and self.abort_condition():
                raise RuntimeError(
                    'Abort condition met during wait action, aborting')

            rclpy.spin_once(self, timeout_sec=0.5)
            time.sleep(0.5)

            if self.current_odom:
                pos = self.current_odom.pose.pose.position
                self.get_logger().info(
                    f'Wait feedback: pos=({pos.x:.2f}, {pos.y:.2f}), '
                    f'elapsed={elapsed:.2f}s'
                )

    def run_force_action(self, **params: Any) -> None:
        """
        Apply a constant force to the vehicle for a specified duration.

        This loop publishes a Wrench message at fixed intervals and checks for abort
        conditions after an optional grace period. Odometry feedback is logged each cycle.

        Args:
            force_x (float): Force in x-direction in Newtons.
            force_y (float): Force in y-direction in Newtons.
            torque_z (float): Torque about z-axis in Newton-meters.
            duration (float): Total time to apply force in seconds.
            grace (float): Grace period before starting abort checks in seconds.

        Raises:
            RuntimeError: If abort_condition() returns True after grace period.
        """
        self.call_mux(TOPIC_FORCE_CONSTANT)
        msg = Wrench()

        duration = params.get('duration', 0.0)
        grace = params.get('grace', 0.0)

        start = self.get_clock().now().nanoseconds / 1e9
        elapsed = 0.0
        while rclpy.ok() and elapsed < duration:
            elapsed = self.get_clock().now().nanoseconds / 1e9 - start

            msg.force.x = params.get('force_x', 0.0)
            msg.force.y = params.get('force_y', 0.0)
            msg.torque.z = params.get('torque_z', 0.0)
            self.force_pub.publish(msg)

            if self.current_odom:
                pos = self.current_odom.pose.pose.position
                self.get_logger().info(
                    f'Force feedback: pos=({pos.x:.2f}, {pos.y:.2f})'
                )

            if elapsed >= grace and self.abort_condition():
                raise RuntimeError(
                    'Abort condition met during force action, aborting')

            rclpy.spin_once(self, timeout_sec=0.5)
            time.sleep(0.5)

    def execute_action(self, action: Dict[str, Any]) -> None:
        """
        Execute one mission action and resolve its parameters.

        Normalizes callable parameters, disables other controllers, and invokes the action.

        Args:
            action (dict): Action descriptor with keys 'action' (callable) and 'parameters' (dict).
        """
        if not isinstance(action, dict):
            self.get_logger().error(f'Invalid action format: {action}')
            return

        runner = action.get('action')
        params = action.get('parameters', {})

        self.set_enable(self.pose_enable_client, False, 'pose')
        self.set_enable(self.vel_enable_client, False, 'velocity')

        if not callable(runner):
            self.get_logger().warn(f'Unknown action: {runner}')
            return

        # Resolve any callable parameters before running
        resolved_params = {k: v() if callable(v) else v for k, v in params.items()}

        runner(**resolved_params)

    def execute_actions(self) -> None:
        """
        Loop through the action sequence `repeat` times, handling aborts.

        For each action, calls execute_action and on RuntimeError runs an abort_action.
        """
        for _ in range(self.repeat):

            for action in self.actions:

                try:
                    self.execute_action(action)
                except RuntimeError as e:
                    abort_action = action.get('abort_action', self.default_abort_action)
                    self.get_logger().warn(f'Error executing action {action["name"]}: {e}')
                    self.get_logger().warn(f'Executing abort action: {abort_action["name"]}')
                    try:
                        self.execute_action(abort_action)
                    except RuntimeError as abort_e:
                        self.get_logger().error(f'Abort action failed too!: {abort_e}')
                        self.set_enable(self.pose_enable_client, False, 'pose')
                        self.set_enable(self.vel_enable_client, False, 'velocity')
                        self.call_mux(TOPIC_FORCE_BASE)
                        exit(1)


    def odom_callback(self, msg):
        """
        Callback for Odometry subscription; stores the latest message.

        Args:
            msg (Odometry): Received odometry message containing pose and twist.
        """
        self.current_odom = msg


def main(args: Optional[List[str]] = None) -> None:
    rclpy.init(args=args)
    node = ActionRunner()
    node.execute_actions()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
