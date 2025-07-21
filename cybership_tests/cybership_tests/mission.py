#!/usr/bin/env python3


import rclpy
import rclpy.client
from rclpy.node import Node
from std_srvs.srv import SetBool
from topic_tools_interfaces.srv import MuxSelect, MuxAdd
from geometry_msgs.msg import PoseStamped, Twist, Wrench
from std_msgs.msg import Bool
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from tf_transformations import quaternion_from_euler
import numpy as np
from typing import Any, Dict, Optional, List
from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor


# Topic and service name constants
TOPIC_FORCE_CONSTANT: str = "control/force/command/constant"
TOPIC_VELOCITY_COMMAND: str = "control/velocity/command"
TOPIC_ODOM: str = "measurement/odom"
TOPIC_POSE_GOAL: str = "navigate_to_pose"
TOPIC_FORCE_POSITION: str = "control/force/command/position"
TOPIC_FORCE_VELOCITY: str = "control/force/command/velocity"
TOPIC_FORCE_BASE: str = "control/force/command"

SERVICE_MUX_ADD: str = "force_mux/add"
SERVICE_MUX_SELECT: str = "force_mux/select"
SERVICE_POSE_ENABLE: str = "position_controller/change_state"
SERVICE_VEL_ENABLE: str = "velocity_controller/change_state"

TOPIC_EXPERIMENT_ODOM: str = "experiment/odom"
TOPIC_EXPERIMENT_FORCE: str = "experiment/force"
TOPIC_EXPERIMENT_FLAG: str = "experiment/flag"


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
        super().__init__("action_runner", namespace="voyager")
        # Private executor for service calls
        # self._executor.add_node(self)
        # Define the list of mission actions
        self.add_mux(TOPIC_FORCE_CONSTANT)

        self.repeat = 500

        self.vel_pub = self.create_publisher(Twist, TOPIC_VELOCITY_COMMAND, 10)
        self.force_pub = self.create_publisher(Wrench, TOPIC_FORCE_CONSTANT, 10)

        self.experiment_odom_pub = self.create_publisher(
            Odometry, TOPIC_EXPERIMENT_ODOM, 10
        )
        self.experiment_force_pub = self.create_publisher(
            Wrench, TOPIC_EXPERIMENT_FORCE, 10
        )
        self.experiment_flag_pub = self.create_publisher(
            Bool, TOPIC_EXPERIMENT_FLAG, 10
        )

        self.current_odom = None
        self.create_subscription(Odometry, TOPIC_ODOM, self.odom_callback, 10)

        # Action client for pose navigation (relative to node namespace)
        self.pose_action_client = ActionClient(self, NavigateToPose, TOPIC_POSE_GOAL)
        self.get_logger().info(f"Waiting for {self.pose_action_client._action_name} action server...")
        self.pose_action_client.wait_for_server()

        self.pose_enable_client = self.create_client(SetBool, SERVICE_POSE_ENABLE)
        self.get_logger().info(f"Waiting for  server {self.pose_enable_client.service_name}...")
        self.pose_enable_client.wait_for_service()

        self.vel_enable_client = self.create_client(SetBool, SERVICE_VEL_ENABLE)
        self.get_logger().info(f"Waiting for  server {self.vel_enable_client.service_name}...")
        self.vel_enable_client.wait_for_service()

        self.rng = np.random.default_rng()

        self.wait_until_ready()

        # Experiment publishing state
        self.experiment_enabled: bool = False
        self._last_force: Optional[Wrench] = None
        self.create_timer(0.1, self._experiment_publisher)


        # Default abort lands back at origin after timeout
        self.default_abort_action = {
            "name": "default_abort",
            "action": self.run_pose_action,
            "parameters": {
                "x": 0.0,
                "y": 0.0,
                "yaw": 0.0,
                "duration": 60.0,
                "grace": 15.0,
            },
        }

        self.actions = [
            {
                "name": "pose_neg_diag",
                "action": self.run_pose_action,
                "parameters": {
                    "x": -2.0,
                    "y": -2.0,
                    "yaw": np.pi / 4,
                    "duration": 60.0,
                },
            },
            {
                "name": "velocity_random_1",
                "action": self.run_velocity_action,
                "parameters": {
                    # Sample linear and angular components each publish
                    "linear_x": lambda: self.rng.uniform(0.4, 0.8),
                    "linear_y": lambda: self.rng.uniform(-0.1, 0.1),
                    "angular_z": lambda: self.rng.uniform(-0.01, 0.01),
                    "duration": 30.0,
                    "margin": 0.1,  # Allow some margin for early success
                },
                "abort_action": {
                    "name": "velocity_random_1_abort",
                    "action": self.run_pose_action,
                    "parameters": {
                        "x": 2.0,
                        "y": 2.0,
                        "yaw": 5 * np.pi / 4,
                        "duration": 60.0,
                        "grace": 15.0,
                    },
                },
            },
            {
                "name": "force_random_1",
                "action": self.run_force_action,
                "parameters": {
                    "force_x": lambda: self.rng.uniform(0.2, 0.8),
                    "force_y": lambda: self.rng.uniform(-0.5, 0.5),
                    "torque_z": lambda: self.rng.uniform(-0.1, 0.1),
                    "duration": 30.0,
                    "experiment": True,  # Enable experiment flag
                    "continuous_sampling": True,  # Sample force continuously
                    "frequency": 1.0,  # Hz
                },
                "abort_action": {
                    "name": "force_random_1_abort",
                    "action": self.run_pose_action,
                    "parameters": {
                        "x": 2.0,
                        "y": 2.0,
                        "yaw": 5 * np.pi / 4,
                        "duration": 60.0,
                        "grace": 15.0,
                    },
                },
            },
            {
                "name": "pose_pos_diag",
                "action": self.run_pose_action,
                "parameters": {
                    "x": 2.0,
                    "y": 2.0,
                    "yaw": 5 * np.pi / 4,
                    "duration": 60.0,
                },
            },
            {
                "name": "velocity_random_2",
                "action": self.run_velocity_action,
                "parameters": {
                    # Sample linear and angular components each publish
                    "linear_x": lambda: self.rng.uniform(0.2, 0.8),
                    "linear_y": lambda: self.rng.uniform(-0.1, 0.1),
                    "angular_z": lambda: self.rng.uniform(-0.01, 0.01),
                    "duration": 30.0,
                    "margin": 0.1,  # Allow some margin for early success
                },
                "abort_action": {
                    "name": "velocity_random_2_abort",
                    "action": self.run_pose_action,
                    "parameters": {
                        "x": -2.0,
                        "y": -2.0,
                        "yaw": np.pi / 4,
                        "duration": 60.0,
                        "grace": 15.0,
                    },
                },
            },
            {
                "name": "force_random_2",
                "action": self.run_force_action,
                "parameters": {
                    "force_x": lambda: self.rng.uniform(0.2, 0.8),
                    "force_y": lambda: self.rng.uniform(-0.5, 0.5),
                    "torque_z": lambda: self.rng.uniform(-0.1, 0.1),
                    "duration": 30.0,
                    "experiment": True,  # Enable experiment flag
                    "continuous_sampling": True,  # Sample force continuously
                    "frequency": 1.0,  # Hz
                },
                "abort_action": {
                    "name": "force_random_2_abort",
                    "action": self.run_pose_action,
                    "parameters": {
                        "x": -2.0,
                        "y": -2.0,
                        "yaw": np.pi / 4,
                        "duration": 60.0,
                        "grace": 15.0,
                    },
                },
            },
            {
                "name": "wait",
                "action": self.run_wait_action,
                "parameters": {"duration": 10.0},
            },
        ]

    def wait_until_ready(self) -> None:
        """
        Block until the first odometry message is received.

        Ensures the vehicle has a valid initial pose before performing actions.
        """
        while rclpy.ok() and self.current_odom is None:
            self.get_logger().info("Waiting for initial odometry data...")
            # process callbacks without blocking executor
            rclpy.spin_once(self, timeout_sec=0.1)
        self.get_logger().info("Initial odometry data received.")

    def _experiment_publisher(self) -> None:
        """
        Background thread publishes experiment flag, and when enabled, odometry and force data.
        """
        flag_msg = Bool()
        flag_msg.data = self.experiment_enabled
        self.experiment_flag_pub.publish(flag_msg)
        if self.experiment_enabled:
            if self.current_odom:
                self.experiment_odom_pub.publish(self.current_odom)
            if self._last_force:
                self.experiment_force_pub.publish(self._last_force)


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
        if future.result().success:
            self.get_logger().info(f"Mux added topic {topic}")
        else:
            self.get_logger().error(f"Failed to add mux topic {topic}")

    def abort_condition(self) -> bool:
        """
        Determine if the mission should abort based on odometry limits.

        Returns:
            bool: True if |x| or |y| position exceeds 2.5 meters, False otherwise.
        """
        return (
            abs(self.current_odom.pose.pose.position.x) > 2.5
            or abs(self.current_odom.pose.pose.position.y) > 2.5
        )

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
            self.get_logger().info(f"Mux switched to {topic}")
        else:
            self.get_logger().error(f"Failed to switch mux to {topic}")

    def set_enable(self, client: rclpy.client.Client, enable: bool) -> None:
        """
        Enable or disable a controller via SetBool service.

        Args:
            client (rclpy.client.Client): Service client for SetBool.
            enable (bool): True to enable, False to disable.
        """
        self.get_logger().info(f"Setting {client.service_name} enable={enable}")
        req = SetBool.Request()
        req.data = enable
        future = client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        if future.result().success:
            self.get_logger().info(f'Service {client.service_name} set enable={enable}')
        else:
            self.get_logger().error(f'Failed to set {client.service_name} enable={enable}')

    def _pose_feedback_callback(self, feedback_msg: Any) -> None:
        """
        Callback to log remaining distance from a NavigateToPose action.

        Args:
            feedback_msg: Action feedback message containing distance_remaining (float, meters).
        """
        fb = feedback_msg.feedback
        # nav2 NavigateToPose feedback has distance_remaining
        dist = getattr(fb, "distance_remaining", None)
        if dist is not None:
            self.get_logger().info(
                f"    NavigateToPose feedback: distance_remaining={dist:.2f}"
            )
        else:
            self.get_logger().info("    NavigateToPose feedback received")

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
        resolved_params = {k: v() if callable(v) else v for k, v in params.items()}
        # Activate pose controller via mux & service
        self.call_mux(TOPIC_FORCE_POSITION)

        self.set_enable(self.pose_enable_client, True)

        # Extract parameters
        x = resolved_params.get("x", 0.0)
        y = resolved_params.get("y", 0.0)
        yaw = resolved_params.get("yaw", 0.0)
        duration = resolved_params.get("duration", 0.0)
        grace = resolved_params.get("grace", 0.0)
        self.get_logger().info(
            f"Pose action start (x={x:.2f}, y={y:.2f}, yaw={yaw:.2f}), "
            f"duration={duration:.1f}s, grace={grace:.1f}s"
        )

        # Build goal message
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "world"
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
            self.get_logger().error("NavigateToPose goal rejected")
            self.set_enable(self.pose_enable_client, False)
            return

        self.get_logger().info("NavigateToPose goal accepted, monitoring execution")

        # Wait for result, checking abort & timeout
        result_fut = handle.get_result_async()
        start = self.get_clock().now().nanoseconds / 1e9
        elapsed = 0.0
        while rclpy.ok():
            elapsed = (self.get_clock().now().nanoseconds / 1e9) - start

            if elapsed >= duration:
                self.get_logger().info(
                    "NavigateToPose action completed due to duration"
                )
                cancel_fut = handle.cancel_goal_async()
                rclpy.spin_until_future_complete(self, cancel_fut)
                self.set_enable(self.pose_enable_client, False)
                break

            if result_fut.done():
                self.get_logger().info("NavigateToPose action completed")
                break

            if elapsed >= grace and self.abort_condition():

                cancel_fut = handle.cancel_goal_async()
                rclpy.spin_until_future_complete(self, cancel_fut)
                self.set_enable(self.pose_enable_client, False)
                raise RuntimeError('Abort condition met during pose action, aborting')

            rclpy.spin_once(self, timeout_sec=0.5)

        # Deactivate pose controller
        self.set_enable(self.pose_enable_client, False)

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
        resolved_params = {k: v() if callable(v) else v for k, v in params.items()}
        # Activate velocity controller
        self.call_mux(TOPIC_FORCE_VELOCITY)
        self.set_enable(self.vel_enable_client, True)

        # Prepare and log command
        msg = Twist()
        lx = resolved_params.get("linear_x", 0.0)
        ly = resolved_params.get("linear_y", 0.0)
        az = resolved_params.get("angular_z", 0.0)
        margin = resolved_params.get("margin", None)
        self.get_logger().info(
            "    "
            f"Velocity action (u={lx:.4f}, v={ly:.4f}, r={az:.4f})"
            + (f", margin={margin:.4f}" if margin is not None else "")
        )

        # Extract duration, grace period and start time
        duration = resolved_params.get("duration", 0.0)
        grace = resolved_params.get("grace", 0.0)
        start = self.get_clock().now().nanoseconds / 1e9
        elapsed = 0.0
        # Loop at ~10 Hz, processing callbacks without blocking
        period = 0.1

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
                lvel = self.current_odom.twist.twist.linear
                avel = self.current_odom.twist.twist.angular
                self.get_logger().info(
                    "    "
                    f"Velocity feedback: "
                    f"vel=(vx={lvel.x:.2f}, vy={lvel.y:.2f}, vr={avel.z:.2f}), "
                    f"cmd=(u={lx:.2f}, v={ly:.2f}, r={az:.2f})"
                )

            # margin-based early success
            if margin is not None:
                act = self.current_odom.twist.twist
                if abs(act.linear.x - lx) < margin and abs(act.linear.y - ly) < margin:
                    self.get_logger().info("Velocity within margin, stopping early")
                    break

            # Abort check after grace period
            if self.current_odom and elapsed >= grace and self.abort_condition():
                raise RuntimeError(
                    "Abort condition met during velocity action, aborting"
                )

            # enforce loop rate and allow callbacks
            rclpy.spin_once(self, timeout_sec=period)

        # Deactivate velocity controller
        self.set_enable(self.vel_enable_client, False)

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
        resolved_params = {k: v() if callable(v) else v for k, v in params.items()}
        duration = resolved_params.get("duration", 0.0)
        grace = resolved_params.get("grace", 0.0)
        start = self.get_clock().now().nanoseconds / 1e9

        self.get_logger().info(f"Waiting for {duration:.4f} seconds")
        elapsed = 0.0
        # Loop at ~2 Hz, processing callbacks without blocking
        period = 0.5
        while rclpy.ok() and elapsed < duration:
            elapsed = self.get_clock().now().nanoseconds / 1e9 - start

            if elapsed >= grace and self.abort_condition():
                raise RuntimeError("Abort condition met during wait action, aborting")

            # allow callbacks and enforce loop timing
            rclpy.spin_once(self, timeout_sec=period)

            if self.current_odom:
                pos = self.current_odom.pose.pose.position
                self.get_logger().info(
                    f"Wait feedback: pos=({pos.x:.2f}, {pos.y:.2f}), "
                    f"elapsed={elapsed:.2f}s"
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

        resolved_params = {k: v() if callable(v) else v for k, v in params.items()}

        self.call_mux(TOPIC_FORCE_CONSTANT)

        msg = Wrench()

        duration = params.get("duration", 0.0)
        grace = params.get("grace", 0.0)

        continuous_sampling = params.get("continuous_sampling", False)
        frequency = params.get("frequency", 1.0)  # Hz

        start = self.get_clock().now().nanoseconds / 1e9
        elapsed = 0.0

        # Loop at specified frequency (or 2 Hz), processing callbacks without blocking
        period = 1.0 / (frequency if continuous_sampling else 2.0)

        while rclpy.ok() and elapsed < duration:
            elapsed = self.get_clock().now().nanoseconds / 1e9 - start

            msg.force.x = resolved_params.get("force_x", 0.0)
            msg.force.y = resolved_params.get("force_y", 0.0)
            msg.torque.z = resolved_params.get("torque_z", 0.0)

            if continuous_sampling:
                # Sample force parameters at each iteration
                msg.force.x = params.get("force_x", 0.0)()
                msg.force.y = params.get("force_y", 0.0)()
                msg.torque.z = params.get("torque_z", 0.0)()

            # record last force for experiment publishing
            self._last_force = msg
            self.force_pub.publish(msg)

            if self.current_odom:
                pos = self.current_odom.pose.pose.position
                self.get_logger().info(
                    f"Force feedback: (fx={msg.force.x:.2f}, "
                    f"fy={msg.force.y:.2f}, tz={msg.torque.z:.2f}),"
                    f" pos=({pos.x:.2f}, {pos.y:.2f}), "
                    f"elapsed={elapsed:.2f}s"
                )

            if elapsed >= grace and self.abort_condition():
                raise RuntimeError("Abort condition met during force action, aborting")

            # allow callbacks and enforce publish rate
            rclpy.spin_once(self)

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

        self.set_enable(self.pose_enable_client, False)
        self.set_enable(self.vel_enable_client, False)

        if not callable(runner):
            self.get_logger().warn(f'Unknown action: {runner}')
            return

        self.get_logger().info(f"Executing action: {action['name']}")

        # Handle experiment flag without resolving other parameters
        self.experiment_enabled = bool(params.get("experiment", False))
        runner(**params)

        # Disable experiment after action completes
        self.experiment_enabled = False

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

if __name__ == "__main__":
    main()
