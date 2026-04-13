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
from tf_transformations import quaternion_from_euler, euler_from_quaternion
import numpy as np
from typing import Any, Dict, Optional, List
from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor
from abc import ABC, abstractmethod
import time
import threading
import copy
import math


# Topic and service name constants
TOPIC_FORCE_CONSTANT: str = "control/force/command/constant"
TOPIC_VELOCITY_COMMAND: str = "control/velocity/command"
TOPIC_ODOM: str = "measurement/odom"
TOPIC_POSE_GOAL: str = "navigate_to_pose"
TOPIC_FORCE_POSITION: str = "control/force/command/position"
TOPIC_FORCE_VELOCITY: str = "control/force/command/velocity"
TOPIC_FORCE_BASE: str = "control/force/command"
TOPIC_FORCE_ZERO: str = "control/force/command/zero"

SERVICE_MUX_ADD: str = "force_mux/add"
SERVICE_MUX_SELECT: str = "force_mux/select"
SERVICE_POSE_ENABLE: str = "position_controller/change_state"
SERVICE_VEL_ENABLE: str = "velocity_controller/change_state"

TOPIC_EXPERIMENT_ODOM: str = "experiment/odom"
TOPIC_EXPERIMENT_FORCE: str = "experiment/force"
TOPIC_EXPERIMENT_FLAG: str = "experiment/flag"

# Abort/recovery thresholds (meters)
ABORT_LIMIT: float = 3.5
RECOVER_LIMIT: float = 3.0
ABORT_LIMIT_X_MIN = -5.0
ABORT_LIMIT_X_MAX = 4.5
ABORT_LIMIT_Y_MIN = -2.5
ABORT_LIMIT_Y_MAX = 3.0

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
        super().__init__("action_runner", namespace="drillship")

        self._log_last_time: Dict[str, float] = {}
        self._first_odom_event = threading.Event()

        # Persistent clients (avoid re-creating clients per call)
        self.mux_add_client = self.create_client(MuxAdd, SERVICE_MUX_ADD)
        self.mux_select_client = self.create_client(MuxSelect, SERVICE_MUX_SELECT)

        # Define mission settings
        # (left as constants to keep this file standalone/minimal)
        self.repeat = 500

        self.vel_pub = self.create_publisher(Twist, TOPIC_VELOCITY_COMMAND, 10)
        self.force_pub = self.create_publisher(Wrench, TOPIC_FORCE_CONSTANT, 10)
        self.force_zero_pub = self.create_publisher(Wrench, TOPIC_FORCE_ZERO, 10)

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

        self.pose_enable_client = self.create_client(SetBool, SERVICE_POSE_ENABLE)

        self.vel_enable_client = self.create_client(SetBool, SERVICE_VEL_ENABLE)

        self.rng = np.random.default_rng()

        # Ensure mux services exist before using them
        self._wait_for_service(self.mux_add_client, SERVICE_MUX_ADD)
        self._wait_for_service(self.mux_select_client, SERVICE_MUX_SELECT)

        # NOTE: Do not call mux services (call_async) from __init__.
        # Service responses require an executor to be spinning.
        self._mux_topics_to_add = [TOPIC_FORCE_CONSTANT, TOPIC_FORCE_ZERO]
        self._mux_initialized = False
        self._mux_init_lock = threading.Lock()

        # Abort/recovery state
        self._in_recovery: bool = False
        # When True, timed loops won't raise on abort_condition().
        # Used while executing abort actions so they can actually recover.
        self._suppress_abort_checks: bool = False

        # Experiment publishing state
        self.experiment_enabled: bool = False
        self._last_force: Optional[Wrench] = None
        self.create_timer(0.1, self._experiment_publisher)

    def initialize_system(self) -> None:
        # Wait for action server
        while rclpy.ok() and not self.pose_action_client.wait_for_server(timeout_sec=0.5):
            self._throttled_log(
                "wait_nav_action",
                2.0,
                lambda: f"Waiting for {TOPIC_POSE_GOAL} action server...",
            )

        # Wait for controller enable/disable services
        self._wait_for_service(self.pose_enable_client, SERVICE_POSE_ENABLE)
        self._wait_for_service(self.vel_enable_client, SERVICE_VEL_ENABLE)


        # Default abort lands back at origin after timeout
        self.default_abort_action = {
            "name": "default_abort",
            "action": self.run_pose_action,
            "parameters": {
                "x": 0.0,
                "y": 0.0,
                "yaw": 0.0,
                "duration": 60.0,
                "wait": 10.0,
                "grace": 2.0,
            },
        }

        self.actions = [
            # i want boat in same starting position each time.
            {
                "name": "pose_neg_diag",
                "action": self.run_pose_action,
                "parameters": {
                    "x": -2.5,
                    "y": 1.5,
                    "yaw": -np.pi / 12,
                    "duration": 60.0,
                    "wait": 10.0,
                },
            },
            # command constant forces
            {
                "name": "command_constant_force",
                "action": self.run_force_action,
                "parameters": {
                    "force_x": lambda: self.rng.uniform(0.5, 1.25),
                    "force_y": lambda: self.rng.uniform(-0.1, 0.1),
                    "torque_z": lambda: self.rng.uniform(-0.25, 0.25),
                    "duration": 20.0,
                    "experiment": True,  # Enable experiment flag
                    "continuous_sampling": True,  # Sample force continuously
                    "frequency": 1.0,  # Hz
                }
            },
            {
                "name": "pose_pos_diag",
                "action": self.run_pose_action,
                "parameters": {
                    "x": -2.5,
                    "y": -1.0,
                    "yaw": np.pi / 7,
                    "duration": 60.0,
                    "wait": 10.0,
                },
            },
                        # command constant forces
            {
                "name": "command_constant_force",
                "action": self.run_force_action,
                "parameters": {
                    "force_x": lambda: self.rng.uniform(0.7, 1.25),
                    "force_y": lambda: self.rng.uniform(-0.1, 0.1),
                    "torque_z": lambda: self.rng.uniform(-0.25, 0.25),
                    "duration": 20.0,
                    "experiment": True,  # Enable experiment flag
                    "continuous_sampling": True,  # Sample force continuously
                    "frequency": 1.0,  # Hz
                }
            },
            {
                "name": "wait",
                "action": self.run_wait_action,
                "parameters": {"duration": 10.0},
            }
            # {
            #     "name": "pose_neg_diag",
            #     "action": self.run_pose_action,
            #     "parameters": {
            #         "x": -2.0,
            #         "y": -2.0,
            #         "yaw": np.pi / 4,
            #         "duration": 60.0,
            #     },
            # },
            # {
            #     "name": "velocity_random_1",
            #     "action": self.run_velocity_action,
            #     "parameters": {
            #         # Sample linear and angular components each publish
            #         "linear_x": lambda: self.rng.uniform(0, 0.3),
            #         "linear_y": lambda: self.rng.uniform(-0.1, 0.1),
            #         "angular_z": lambda: self.rng.uniform(-0.1, 0.1),
            #         "duration": 15.0,
            #         "margin": 0.01,  # Allow some margin for early success
            #     },
            #     "abort_action": {
            #         "name": "velocity_random_1_abort",
            #         "action": self.run_pose_action,
            #         "parameters": {
            #             "x": 2.0,
            #             "y": 2.0,
            #             "yaw": 5 * np.pi / 4,
            #             "duration": 60.0,
            #             "grace": 15.0,
            #         },
            #     },
            # },
            # {
            #     "name": "force_random_1",
            #     "action": self.run_force_action,
            #     "parameters": {
            #         "force_x": lambda: self.rng.uniform(0.5, 1.5),
            #         "force_y": lambda: self.rng.uniform(-1.5, 1.5),
            #         "torque_z": lambda: self.rng.uniform(-1.5, 1.5),
            #         "duration": 20.0,
            #         "experiment": True,  # Enable experiment flag
            #         "continuous_sampling": True,  # Sample force continuously
            #         "frequency": 1.0,  # Hz
            #     },
            #     "abort_action": {
            #         "name": "force_random_1_abort",
            #         "action": self.run_pose_action,
            #         "parameters": {
            #             "x": 2.0,
            #             "y": 2.0,
            #             "yaw": 5 * np.pi / 4,
            #             "duration": 60.0,
            #             "grace": 15.0,
            #         },
            #     },
            # },
            # {
            #     "name": "pose_pos_diag",
            #     "action": self.run_pose_action,
            #     "parameters": {
            #         "x": 2.0,
            #         "y": 2.0,
            #         "yaw": 5 * np.pi / 4,
            #         "duration": 60.0,
            #     },
            # },
            # {
            #     "name": "velocity_random_2",
            #     "action": self.run_velocity_action,
            #     "parameters": {
            #         # Sample linear and angular components each publish
            #         "linear_x": lambda: self.rng.uniform(0, 0.3),
            #         "linear_y": lambda: self.rng.uniform(-0.1, 0.1),
            #         "angular_z": lambda: self.rng.uniform(-0.1, 0.1),
            #         "duration": 15.0,
            #         "margin": 0.01,  # Allow some margin for early success
            #     },
            #     "abort_action": {
            #         "name": "velocity_random_2_abort",
            #         "action": self.run_pose_action,
            #         "parameters": {
            #             "x": -2.0,
            #             "y": -2.0,
            #             "yaw": np.pi / 4,
            #             "duration": 60.0,
            #             "grace": 15.0,
            #         },
            #     },
            # },
            # {
            #     "name": "force_random_2",
            #     "action": self.run_force_action,
            #     "parameters": {
            #         "force_x": lambda: self.rng.uniform(0, 0.2),
            #         "force_y": lambda: self.rng.uniform(0, 0.05),
            #         "torque_z": lambda: self.rng.uniform(0, 0.05),
            #         "duration": 20.0,
            #         "experiment": True,  # Enable experiment flag
            #         "continuous_sampling": True,  # Sample force continuously
            #         "frequency": 1.0,  # Hz
            #     },
            #     "abort_action": {
            #         "name": "force_random_2_abort",
            #         "action": self.run_pose_action,
            #         "parameters": {
            #             "x": -2.0,
            #             "y": -2.0,
            #             "yaw": np.pi / 4,
            #             "duration": 60.0,
            #             "grace": 15.0,
            #         },
            #     },
            # },
            # {
            #     "name": "wait",
            #     "action": self.run_wait_action,
            #     "parameters": {"duration": 10.0},
            # },
        ]

    def wait_until_ready(self) -> None:
        """
        Block until the first odometry message is received.

        Ensures the vehicle has a valid initial pose before performing actions.
        """
        while rclpy.ok() and not self._first_odom_event.wait(timeout=0.5):
            self._throttled_log(
                "wait_odom",
                2.0,
                lambda: "Waiting for initial odometry data...",
            )
        if self.current_odom is not None:
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

    def _throttled_log(self, key: str, period_s: float, msg_fn, level: str = "info") -> None:
        now = time.monotonic()
        last = self._log_last_time.get(key, 0.0)
        if now - last < period_s:
            return
        self._log_last_time[key] = now
        msg = msg_fn() if callable(msg_fn) else str(msg_fn)
        logger = self.get_logger()
        if level == "debug":
            logger.debug(msg)
        elif level == "warn":
            logger.warn(msg)
        elif level == "error":
            logger.error(msg)
        else:
            logger.info(msg)

    def _wait_for_service(self, client: rclpy.client.Client, name: str) -> None:
        while rclpy.ok() and not client.wait_for_service(timeout_sec=0.5):
            self._throttled_log(
                f"wait_service::{name}",
                2.0,
                lambda: f"Waiting for service '{name}'...",
            )

    def _wait_future(self, future: Any, timeout_s: Optional[float], desc: str) -> Any:
        if hasattr(future, "done") and future.done():
            return future.result()
        done = threading.Event()
        result_container: Dict[str, Any] = {}

        def _on_done(_f: Any) -> None:
            result_container["done"] = True
            done.set()

        future.add_done_callback(_on_done)
        if not done.wait(timeout=None if timeout_s is None else timeout_s):
            raise TimeoutError(f"Timed out waiting for {desc}")
        return future.result()

    def _call_service_sync(
        self,
        client: rclpy.client.Client,
        req: Any,
        timeout_s: float = 5.0,
        desc: str = "service call",
    ) -> Any:
        future = client.call_async(req)
        return self._wait_future(future, timeout_s, desc)

    def _publish_zero_force_once(self) -> None:
        msg = Wrench()
        msg.force.x = 0.0
        msg.force.y = 0.0
        msg.force.z = 0.0
        msg.torque.x = 0.0
        msg.torque.y = 0.0
        msg.torque.z = 0.0
        self.force_zero_pub.publish(msg)


    def add_mux(self, topic: str) -> None:
        """
        Add a topic to the force multiplexer using MuxAdd service.

        Args:
            topic (str): ROS topic name to add to the mux.
        """
        req = MuxAdd.Request()
        req.topic = topic
        future = self.mux_add_client.call_async(req)
        result = self._wait_future(future, 5.0, f"MuxAdd({topic})")
        # MuxAdd returns a result with a success flag
        if result.success:
            self.get_logger().info(f"Mux added topic {topic}")
        else:
            self.get_logger().error(f"Failed to add mux topic {topic}")

    def initialize_mux(self) -> None:
        with self._mux_init_lock:
            if self._mux_initialized:
                return
            for topic in self._mux_topics_to_add:
                self.add_mux(topic)
            self._mux_initialized = True

    def abort_condition(self) -> bool:
        """
        Determine if the mission should abort based on odometry limits.

        Returns:
            bool: True if |x| or |y| position exceeds 2.5 meters, False otherwise.
        """
        pos_x = self.current_odom.pose.pose.position.x if self.current_odom else float("inf")
        pos_y = self.current_odom.pose.pose.position.y if self.current_odom else float("inf")

        if ABORT_LIMIT_X_MIN < pos_x < ABORT_LIMIT_X_MAX and ABORT_LIMIT_Y_MIN < pos_y < ABORT_LIMIT_Y_MAX:
            return False
        else:
            return True

        # return (
        #     abs(self.current_odom.pose.pose.position.x) > ABORT_LIMIT
        #     or abs(self.current_odom.pose.pose.position.y) > ABORT_LIMIT
        # )

    def _recovered_condition(self) -> bool:
        if not self.current_odom:
            return False
        return (
            abs(self.current_odom.pose.pose.position.x) < RECOVER_LIMIT
            and abs(self.current_odom.pose.pose.position.y) < RECOVER_LIMIT
        )

    def recover_until_safe(
        self,
        *,
        timeout_s: float = 30.0,
        stable_s: float = 2.0,
        period_s: float = 0.2,
    ) -> None:
        """Hold zero-force until we're back inside recovery bounds for stable_s."""

        self._in_recovery = True
        safe_start: Optional[float] = None

        self.call_mux(TOPIC_FORCE_ZERO)
        self._publish_zero_force_once()

        def _tick(elapsed: float) -> None:
            nonlocal safe_start
            self._publish_zero_force_once()

            if self._recovered_condition():
                if safe_start is None:
                    safe_start = elapsed
                if elapsed - safe_start >= stable_s:
                    raise StopIteration()
            else:
                safe_start = None

            if self.current_odom:
                pos = self.current_odom.pose.pose.position
                self._throttled_log(
                    "recovery_feedback",
                    2.0,
                    lambda: (
                        f"Recovery: pos=({pos.x:.2f}, {pos.y:.2f}) "
                        f"safe_for={(0.0 if safe_start is None else (elapsed - safe_start)):.1f}s"
                    ),
                )

        try:
            # No abort checks during recovery; we're already in abort handling.
            self._run_timed_loop(
                duration=timeout_s,
                period=period_s,
                tick_cb=_tick,
                abort_checks=False,
            )
        except StopIteration:
            self.get_logger().info("Recovery complete (back within bounds)")
            return
        else:
            raise RuntimeError(f"Recovery timed out after {timeout_s:.1f}s")
        finally:
            self._in_recovery = False

    def call_mux(self, topic: str) -> None:
        """
        Select the active force control topic on the mux.

        Args:
            topic (str): ROS topic name to select for force commands.
        """
        req = MuxSelect.Request()
        req.topic = topic
        future = self.mux_select_client.call_async(req)
        result = self._wait_future(future, 5.0, f"MuxSelect({topic})")
        if not result:
            self.get_logger().error(f"Failed to switch mux to {topic}")
        else:
            self._throttled_log(
                f"mux_select::{topic}",
                2.0,
                lambda: f"Mux selected: {topic}",
            )

    def set_enable(self, client: rclpy.client.Client, enable: bool) -> None:
        """
        Enable or disable a controller via SetBool service.

        Args:
            client (rclpy.client.Client): Service client for SetBool.
            enable (bool): True to enable, False to disable.
        """
        req = SetBool.Request()
        req.data = enable
        result = self._call_service_sync(client, req, timeout_s=5.0, desc="SetBool")
        if result.success:
            self._throttled_log(
                f"set_enable::{id(client)}::{enable}",
                1.0,
                lambda: f"Controller enable set to {enable}",
            )
        else:
            self.get_logger().error(f"Failed to set controller enable={enable}")

    def _run_timed_loop(
        self,
        *,
        duration: float,
        period: float,
        tick_cb,
        grace: float = 0.0,
        abort_msg: str = "Abort condition met",
        abort_checks: bool = True,
    ) -> None:
        done_event = threading.Event()
        err_container: Dict[str, Exception] = {}
        start_mono = time.monotonic()
        timer_holder: Dict[str, Any] = {"timer": None}

        def _finish(err: Optional[Exception] = None) -> None:
            if err is not None and "err" not in err_container:
                err_container["err"] = err
            t = timer_holder.get("timer")
            if t is not None:
                t.cancel()
            done_event.set()

        def _timer_cb() -> None:
            try:
                elapsed = time.monotonic() - start_mono

                if elapsed >= duration:
                    _finish()
                    return

                if (
                    abort_checks
                    and not self._suppress_abort_checks
                    and not self._in_recovery
                    and elapsed >= grace
                    and self.current_odom
                    and self.abort_condition()
                ):
                    _finish(RuntimeError(abort_msg))
                    return

                tick_cb(elapsed)
            except Exception as e:
                _finish(e)

        timer_holder["timer"] = self.create_timer(period, _timer_cb)
        done_event.wait()
        if "err" in err_container:
            raise err_container["err"]

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
            self._throttled_log(
                "pose_feedback",
                2.0,
                lambda: f"NavigateToPose fb: distance_remaining={dist:.2f}",
            )
        else:
            self._throttled_log(
                "pose_feedback_generic",
                2.0,
                lambda: "NavigateToPose feedback received",
            )

    def run_pose_action(self, **params: Any) -> None:
        """
        Navigate to a target pose and monitor for timeouts or abort conditions.

        Sends a NavigateToPose goal and runs for the full `duration` unless aborted.
        Reaching the pose (within odom tolerance) does NOT end the action early; it only
        marks "reached" and we stay in pose mode until `duration` elapses.

        Args:
            x (float): Target x-position in meters.
            y (float): Target y-position in meters.
            yaw (float): Target yaw angle in radians.
            duration (float): Total pose action time in seconds.
            wait (float, optional): Once pose is reached, wait this long (seconds) before
                completing the action. The action will still never exceed `duration`.
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
        wait_s_raw = resolved_params.get("wait", None)
        wait_s: Optional[float] = None if wait_s_raw is None else float(wait_s_raw)
        pos_tolerance = float(resolved_params.get("pos_tolerance", 0.3))
        yaw_tolerance = float(resolved_params.get("yaw_tolerance", 0.4))
        stable_s = float(resolved_params.get("stable_s", 1.0))
        self.get_logger().info(
            f"Pose action start (x={x:.2f}, y={y:.2f}, yaw={yaw:.2f}), "
            f"duration={duration:.1f}s, grace={grace:.1f}s, "
            f"tol=(pos={pos_tolerance:.2f}m, yaw={yaw_tolerance:.2f}rad, stable={stable_s:.1f}s)"
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

        handle = self._wait_future(goal_fut, 10.0, "NavigateToPose send_goal")
        if not handle.accepted:
            self.get_logger().error("NavigateToPose goal rejected")
            self.set_enable(self.pose_enable_client, False)
            return

        self.get_logger().info("NavigateToPose goal accepted, monitoring execution")

        # Monitor odom progress for logging (and optional reach detection), while
        # allowing Nav2 result to arrive any time.
        result_fut = handle.get_result_async()
        stable_start: Optional[float] = None
        reached_at: Optional[float] = None
        dwell_until: Optional[float] = None
        logged_done = False

        def _angle_wrap(a: float) -> float:
            return math.atan2(math.sin(a), math.cos(a))

        def _tick(elapsed: float) -> None:
            nonlocal stable_start, reached_at, dwell_until, logged_done

            if result_fut.done() and not logged_done:
                logged_done = True
                try:
                    res = result_fut.result()
                    status = getattr(res, "status", None)
                    self.get_logger().info(f"NavigateToPose result received (status={status})")
                except Exception as e:
                    self.get_logger().warn(f"NavigateToPose result received (failed to read): {e}")

            if not self.current_odom:
                return

            pos = self.current_odom.pose.pose.position
            dx = float(x) - float(pos.x)
            dy = float(y) - float(pos.y)
            dist = math.hypot(dx, dy)

            q = self.current_odom.pose.pose.orientation
            _, _, yaw_cur = euler_from_quaternion([q.x, q.y, q.z, q.w])
            yaw_err = abs(_angle_wrap(float(yaw) - float(yaw_cur)))

            self._throttled_log(
                "pose_tick",
                2.0,
                lambda: (
                    f"Pose fb: dist={dist:.2f}m yaw_err={yaw_err:.2f}rad "
                    f"elapsed={elapsed:.1f}s"
                ),
            )

            within = dist <= pos_tolerance and yaw_err <= yaw_tolerance
            if within:
                if stable_start is None:
                    stable_start = elapsed
                if reached_at is None and (elapsed - stable_start) >= stable_s:
                    reached_at = elapsed
                    if wait_s is not None:
                        dwell_until = min(float(duration), float(reached_at) + max(0.0, float(wait_s)))
                        self.get_logger().info(
                            f"Pose reached (odom tolerance) at t={reached_at:.1f}s; dwelling until t={dwell_until:.1f}s"
                        )
                    else:
                        self.get_logger().info(
                            f"Pose reached (odom tolerance) at t={reached_at:.1f}s; holding until duration"
                        )
            else:
                stable_start = None

            # If a post-reach wait was requested, complete once dwell time is satisfied,
            # but never exceed the action duration.
            if reached_at is not None and wait_s is not None and dwell_until is not None:
                if elapsed >= dwell_until:
                    raise StopIteration()

        try:
            self._run_timed_loop(
                duration=duration,
                period=0.2,
                tick_cb=_tick,
                grace=grace,
                abort_msg="Abort condition met during pose action, aborting",
            )
        except StopIteration:
            self.get_logger().info("Pose wait complete")
            if not result_fut.done():
                cancel_fut = handle.cancel_goal_async()
                try:
                    self._wait_future(cancel_fut, 5.0, "NavigateToPose cancel")
                except Exception as e:
                    self.get_logger().warn(f"Cancel after wait failed: {e}")
        except RuntimeError:
            cancel_fut = handle.cancel_goal_async()
            self._wait_future(cancel_fut, 5.0, "NavigateToPose cancel")
            self.set_enable(self.pose_enable_client, False)
            raise
        else:
            if reached_at is None:
                self.get_logger().info(
                    "Pose action completed due to duration (pose not reached within tolerance)"
                )
            else:
                self.get_logger().info(
                    f"Pose action completed due to duration (held for {max(0.0, float(duration) - float(reached_at)):.1f}s)"
                )

            if not result_fut.done():
                cancel_fut = handle.cancel_goal_async()
                self._wait_future(cancel_fut, 5.0, "NavigateToPose cancel")

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

        duration = float(resolved_params.get("duration", 0.0))
        grace = float(resolved_params.get("grace", 0.0))
        period = 0.1

        def _tick(elapsed: float) -> None:
            # Publish the commanded velocity
            msg.linear.x = lx
            msg.linear.y = ly
            msg.angular.z = az
            self.vel_pub.publish(msg)

            if self.current_odom:
                lvel = self.current_odom.twist.twist.linear
                avel = self.current_odom.twist.twist.angular
                self._throttled_log(
                    "vel_feedback",
                    2.0,
                    lambda: (
                        f"Velocity fb: vel=(vx={lvel.x:.2f}, vy={lvel.y:.2f}, r={avel.z:.2f}), "
                        f"cmd=(u={lx:.2f}, v={ly:.2f}, r={az:.2f})"
                    ),
                )

            if margin is not None and self.current_odom:
                act = self.current_odom.twist.twist
                if abs(act.linear.x - lx) < margin and abs(act.linear.y - ly) < margin:
                    raise StopIteration()

        try:
            self._run_timed_loop(
                duration=duration,
                period=period,
                tick_cb=_tick,
                grace=grace,
                abort_msg="Abort condition met during velocity action, aborting",
            )
        except StopIteration:
            self.get_logger().info("Velocity within margin, stopping early")

        # Ensure no command is held after action
        self.call_mux(TOPIC_FORCE_ZERO)
        self._publish_zero_force_once()

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
            force_topic (str, optional): Mux force topic to select during wait.
                Defaults to `control/force/command/zero`.

        Raises:
            RuntimeError: If abort_condition() returns True after grace period.
        """
        resolved_params = {k: v() if callable(v) else v for k, v in params.items()}
        duration = resolved_params.get("duration", 0.0)
        grace = resolved_params.get("grace", 0.0)
        force_topic = str(resolved_params.get("force_topic", TOPIC_FORCE_ZERO))
        self.get_logger().info(f"Waiting for {duration:.2f} seconds")

        # Minimal behavior: just select the requested mux topic.
        self.call_mux(force_topic)

        def _tick(elapsed: float) -> None:
            if self.current_odom:
                pos = self.current_odom.pose.pose.position
                self._throttled_log(
                    "wait_feedback",
                    2.0,
                    lambda: f"Wait fb: pos=({pos.x:.2f}, {pos.y:.2f}), elapsed={elapsed:.1f}s",
                )

        self._run_timed_loop(
            duration=float(duration),
            period=0.5,
            tick_cb=_tick,
            grace=float(grace),
            abort_msg="Abort condition met during wait action, aborting",
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

        duration = float(resolved_params.get("duration", 0.0))
        grace = float(resolved_params.get("grace", 0.0))

        continuous_sampling = bool(resolved_params.get("continuous_sampling", False))
        frequency = float(resolved_params.get("frequency", 1.0))  # Hz

        # Loop at specified frequency (or 2 Hz)
        period = 1.0 / (frequency if continuous_sampling else 2.0)

        def _tick(elapsed: float) -> None:
            if continuous_sampling:
                msg.force.x = params.get("force_x", 0.0)()
                msg.force.y = params.get("force_y", 0.0)()
                msg.torque.z = params.get("torque_z", 0.0)()
            else:
                msg.force.x = float(resolved_params.get("force_x", 0.0))
                msg.force.y = float(resolved_params.get("force_y", 0.0))
                msg.torque.z = float(resolved_params.get("torque_z", 0.0))

            # record last force for experiment publishing (copy to avoid mutation)
            self._last_force = copy.deepcopy(msg)
            self.force_pub.publish(msg)

            if self.current_odom:
                pos = self.current_odom.pose.pose.position
                self._throttled_log(
                    "force_feedback",
                    2.0,
                    lambda: (
                        f"Force fb: (fx={msg.force.x:.2f}, fy={msg.force.y:.2f}, tz={msg.torque.z:.2f}), "
                        f"pos=({pos.x:.2f}, {pos.y:.2f}), elapsed={elapsed:.1f}s"
                    ),
                )

        self._run_timed_loop(
            duration=duration,
            period=period,
            tick_cb=_tick,
            grace=grace,
            abort_msg="Abort condition met during force action, aborting",
        )

        # Ensure no wrench is held after action
        self.call_mux(TOPIC_FORCE_ZERO)
        self._publish_zero_force_once()

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

        # Always disable controllers between actions
        self.set_enable(self.pose_enable_client, False)
        self.set_enable(self.vel_enable_client, False)

        # Default to zero command between actions to avoid holding last wrench
        try:
            self.call_mux(TOPIC_FORCE_ZERO)
            self._publish_zero_force_once()
        except Exception as e:
            self._throttled_log(
                "mux_zero_fail",
                5.0,
                lambda: f"Failed to switch to zero mux topic: {e}",
                level="warn",
            )

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
        # Safe to do waits / async service calls now that executor is spinning
        self.initialize_system()
        self.initialize_mux()
        self.wait_until_ready()

        completed_sequences = 0
        while rclpy.ok() and completed_sequences < self.repeat:
            sequence_aborted = False

            for action in self.actions:
                try:
                    self.execute_action(action)
                except RuntimeError as e:
                    sequence_aborted = True

                    # Stop experiment logging ASAP on abort
                    self.experiment_enabled = False

                    abort_action = action.get("abort_action", self.default_abort_action)
                    self.get_logger().warn(
                        f"Error executing action {action['name']}: {e}"
                    )
                    self.get_logger().warn(
                        f"Executing abort action: {abort_action['name']}"
                    )
                    try:
                        # Abort actions are intended to recover from unsafe states.
                        # Suppress abort checks while executing them to avoid immediate re-abort.
                        self._suppress_abort_checks = True
                        try:
                            self.execute_action(abort_action)
                        finally:
                            self._suppress_abort_checks = False
                        self.recover_until_safe(timeout_s=30.0, stable_s=2.0)
                    except RuntimeError as abort_e:
                        self.get_logger().error(f"Abort/recovery failed: {abort_e}")
                        self.set_enable(self.pose_enable_client, False)
                        self.set_enable(self.vel_enable_client, False)
                        self.call_mux(TOPIC_FORCE_BASE)
                        rclpy.shutdown()
                        raise SystemExit(1)

                    # Restart the full sequence after recovery (avoid abort thrash)
                    break

            if not sequence_aborted:
                completed_sequences += 1

        self.get_logger().info("Mission complete")
        rclpy.shutdown()


    def odom_callback(self, msg):
        """
        Callback for Odometry subscription; stores the latest message.

        Args:
            msg (Odometry): Received odometry message containing pose and twist.
        """
        self.current_odom = msg
        self._first_odom_event.set()


def main(args: Optional[List[str]] = None) -> None:
    rclpy.init(args=args)
    node = ActionRunner()

    executor = MultiThreadedExecutor()
    executor.add_node(node)

    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    try:
        node.execute_actions()
    finally:
        executor.shutdown()
        spin_thread.join(timeout=1.0)

if __name__ == "__main__":
    main()
