#!/usr/bin/env python3
"""
ROS2 Action Server for LOS Guidance using cybership_interfaces/LOSGuidance

Provides two implementations:
- LOSGuidanceVelocityROS: Publishes Twist velocity commands
- LOSGuidanceForceCtrROS: Publishes Wrench force/torque commands
"""
import math
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.action import CancelResponse, GoalResponse
from rclpy.executors import SingleThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from abc import ABC, abstractmethod

from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import Twist, Wrench, Vector3, Point
from visualization_msgs.msg import Marker
from tf_transformations import euler_from_quaternion

from cybership_interfaces.action import LOSGuidance

# Import underlying LOS guidance implementation
from cybership_controller.guidance.los import LOSGuidance as BaseLOSGuidance
import numpy as np
import typing
from scipy.interpolate import splev


class BaseLOSGuidanceROS(Node, ABC):
    """Base class for LOS Guidance ROS2 Action Server

    Provides common functionality for path-following guidance with different
    control strategies (velocity vs force control).
    """

    def __init__(self, node_name='los_guidance_server'):
        super().__init__(node_name, namespace="cybership")

        # Declare common parameters
        self.declare_parameter('desired_speed', 0.3)
        self.declare_parameter('lookahead', 1.4)
        self.declare_parameter('heading_gain', 1.0)

        # Declare subclass-specific parameters
        self._declare_control_parameters()

        # Vehicle pose (base attributes)
        self._position = None
        self._yaw = None

        # Declare subclass-specific vehicle state
        self._declare_vehicle_state()

        # Setup publishers and subscribers
        self._marker_pub = self.create_publisher(
            Marker, 'visualization_marker', 10)
        self._odom_sub = self.create_subscription(
            Odometry, 'measurement/odom', self.odom_callback, 10)

        # Setup control publisher (subclass-specific)
        self._setup_control_publisher()

        # Action Server
        self._action_server = ActionServer(
            self,
            LOSGuidance,
            'los_guidance',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=MutuallyExclusiveCallbackGroup())

        self.goal_uuids: typing.List = []

        # Guidance loop state
        self.active_goal_handle = None
        self.guidance = None
        self.last_wp = None
        self.guidance_hz = 2
        self.guidance_timer = None
        self.path_msg = None

        self.get_logger().info(f'{node_name} started')

    @abstractmethod
    def _declare_control_parameters(self):
        """Subclasses override to declare control-specific parameters"""
        pass

    @abstractmethod
    def _declare_vehicle_state(self):
        """Subclasses override to declare control-specific vehicle state attributes"""
        pass

    @abstractmethod
    def _setup_control_publisher(self):
        """Subclasses override to setup their control message publisher"""
        pass

    @abstractmethod
    def _compute_and_publish_control(self, chi_d: float, vel_cmd: np.ndarray, x: float, y: float):
        """Subclasses override to compute and publish control commands"""
        pass

    def odom_callback(self, msg: Odometry):
        """Base odometry callback that extracts position and heading"""
        # Update position and heading
        self._position = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        q = msg.pose.pose.orientation
        # Convert quaternion to yaw using tf_transformations
        quat = [q.x, q.y, q.z, q.w]
        _, _, yaw = euler_from_quaternion(quat)
        self._yaw = yaw

        # Update subclass-specific odometry data
        self._update_odom_helper(msg)

    @abstractmethod
    def _update_odom_helper(self, msg: Odometry):
        """Subclasses override to extract additional odometry data"""
        pass

    def goal_callback(self, goal_request: LOSGuidance.Goal) -> GoalResponse:
        if len(goal_request.path.poses) < 2:
            self.get_logger().warn("Path has fewer than 2 poses — rejecting goal.")
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle) -> CancelResponse:
        return CancelResponse.ACCEPT

    def prepare_waypoints(self, path_msg: Path, extend: bool = True, n_intermediate: int = 2) -> np.ndarray:
        waypoints = [(pose.pose.position.x, pose.pose.position.y)
                     for pose in path_msg.poses]

        if extend:
            # Add intermediate points
            extended_waypoints = []
            for i in range(len(waypoints) - 1):
                p1 = waypoints[i]
                p2 = waypoints[i + 1]
                extended_waypoints.append(p1)
                # Add intermediate points
                for j in range(1, n_intermediate + 1):
                    mid_point = (
                        p1[0] + (p2[0] - p1[0]) * j / (n_intermediate + 1),
                        p1[1] + (p2[1] - p1[1]) * j / (n_intermediate + 1)
                    )
                    extended_waypoints.append(mid_point)
            extended_waypoints.append(waypoints[-1])
            waypoints = extended_waypoints

        return np.array(waypoints)

    def guidance_loop_timer(self):
        """Non-blocking guidance loop executed periodically via timer."""
        if self.active_goal_handle is None:
            return

        goal_handle = self.active_goal_handle

        # Check if goal is still valid
        if tuple(goal_handle.goal_id.uuid) not in self.goal_uuids:
            self.get_logger().info(f"LOS guidance goal no longer current.")
            self.active_goal_handle = None
            self.guidance_timer.cancel()
            return

        if goal_handle.is_cancel_requested:
            goal_handle.canceled()
            self.get_logger().info(f"LOS guidance canceled.")
            self.active_goal_handle = None
            self.guidance_timer.cancel()
            return

        if self._position is None or self._yaw is None:
            return

        x, y = self._position
        dt = 1.0 / self.guidance_hz
        chi_d, vel_cmd, extras = self.guidance.guidance(x, y, dt=dt)

        # Compute and publish control command (subclass-specific)
        self._compute_and_publish_control(chi_d, vel_cmd, x, y)

        # Feedback
        feedback = LOSGuidance.Feedback()
        feedback.heading = chi_d
        feedback.vel_cmd = Vector3(x=vel_cmd[0], y=vel_cmd[1], z=0.0)
        goal_handle.publish_feedback(feedback)

        self.publish_arrow_marker(x, y, vel_cmd)
        self.publish_lookahead_marker(
            extras["lookahead_point"][0], extras["lookahead_point"][1])

        # Check completion
        if math.hypot(x - self.last_wp[0], y - self.last_wp[1]) < self.guidance.delta:
            try:
                goal_handle.succeed()
                self.get_logger().info("LOS guidance finished successfully.")
            except Exception as e:
                self.get_logger().warn(
                    f"Failed to set goal to succeeded state: {e}. "
                    f"Goal may have been aborted by client or timeout.")
            finally:
                self.active_goal_handle = None
                self.guidance_timer.cancel()

    def execute_callback(self, goal_handle):
        """Non-blocking action server callback. Guidance loop runs in timer.

        Returns without a result to keep goal ACTIVE.
        Goal completion is managed in guidance_loop_timer().
        """
        if len(self.goal_uuids) > 0:
            self.goal_uuids.pop(0)

        self.goal_uuids.append(tuple(goal_handle.goal_id.uuid))

        path_msg: Path = goal_handle.request.path
        self.path_msg = path_msg
        waypoints = []
        waypoints.extend([(pose.pose.position.x, pose.pose.position.y)
                          for pose in path_msg.poses])
        if self._position is not None:
            waypoints.append(self._position)

        waypoints = self.prepare_waypoints(
            path_msg, extend=True, n_intermediate=2)

        # Initialize guidance
        V_d = float(self.get_parameter('desired_speed').value)
        delta = float(self.get_parameter('lookahead').value)
        self.guidance = BaseLOSGuidance(waypoints, V_d=V_d, delta=delta)
        self.last_wp = waypoints[-1]

        # Visualize spline path
        self.publish_spline_marker(self.guidance)
        self.publish_path_marker(path_msg)

        # Store goal handle for timer callback to manage completion
        self.active_goal_handle = goal_handle

        # Start guidance loop timer
        if self.guidance_timer is not None:
            self.guidance_timer.cancel()
        self.guidance_timer = self.create_timer(
            1.0 / self.guidance_hz, self.guidance_loop_timer)

    def publish_path_marker(self, path_msg: Path):
        marker = Marker()
        marker.header = path_msg.header
        marker.ns = 'los_path'
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = 0.05
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0
        marker.points = [Point(x=pose.pose.position.x,
                               y=pose.pose.position.y,
                               z=0.0)
                         for pose in path_msg.poses]
        self._marker_pub.publish(marker)

    def publish_arrow_marker(self, x, y, vel_cmd):
        marker = Marker()
        marker.header.frame_id = 'world'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'los_arrow'
        marker.id = 1
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        marker.scale.x = 0.1
        marker.scale.y = 0.1
        marker.scale.z = 0.1
        # Use a copy of vel_cmd for scaling to avoid in-place modification
        vel_arrow = vel_cmd * 3
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.5
        marker.color.a = 1.0
        start = Point(x=x, y=y, z=0.0)
        end = Point(x=x + vel_arrow[0], y=y + vel_arrow[1], z=0.0)
        marker.points = [start, end]
        self._marker_pub.publish(marker)

    def publish_spline_marker(self, guidance):
        marker = Marker()
        marker.header.frame_id = 'world'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'los_spline'
        marker.id = 2
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = 0.05
        marker.color.r = 0.0
        marker.color.g = 0.0
        marker.color.b = 1.0
        marker.color.a = 1.0
        # Sample spline path points
        ts = np.linspace(0, 1, 1000)
        spline_pts = np.array(splev(ts, guidance.tck)).T
        marker.points = [Point(x=pt[0], y=pt[1], z=0.0) for pt in spline_pts]
        self._marker_pub.publish(marker)

    def publish_lookahead_marker(self, x_la, y_la):
        m = Marker()
        m.header.frame_id = 'world'
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = 'los_lookahead'
        m.id = 3
        m.type = Marker.SPHERE
        m.action = Marker.ADD
        m.scale.x = 0.25   # diameter in meters
        m.scale.y = 0.25
        m.scale.z = 0.25
        m.color.r = 1.0
        m.color.g = 0.85
        m.color.b = 0.0
        m.color.a = 1.0  # golden/yellow
        m.pose.position.x = float(x_la)
        m.pose.position.y = float(y_la)
        m.pose.position.z = 0.0
        self._marker_pub.publish(m)

    def publish_lookahead_line(self, x, y, x_la, y_la):
        m = Marker()
        m.header.frame_id = 'world'
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = 'los_lookahead'
        m.id = 4
        m.type = Marker.LINE_STRIP
        m.action = Marker.ADD
        m.scale.x = 0.03
        m.color.r = 1.0
        m.color.g = 0.85
        m.color.b = 0.0
        m.color.a = 0.9
        start = Point(x=float(x),    y=float(y),    z=0.0)
        end = Point(x=float(x_la), y=float(y_la), z=0.0)
        m.points = [start, end]
        self._marker_pub.publish(m)


class LOSGuidanceVelocityROS(BaseLOSGuidanceROS):
    """LOS Guidance implementation with velocity control (Twist messages)"""

    def __init__(self):
        super().__init__('los_guidance_velocity_server')

    def _declare_control_parameters(self):
        """Velocity control uses only heading gain"""
        pass  # heading_gain already declared in base

    def _declare_vehicle_state(self):
        """No additional state needed for velocity control"""
        pass

    def _setup_control_publisher(self):
        """Setup Twist publisher for velocity commands"""
        self._cmd_pub = self.create_publisher(
            Twist, 'control/velocity/command/los', 10)

    def _update_odom_helper(self, msg: Odometry):
        """Velocity control doesn't need linear/angular velocities"""
        pass

    def _compute_and_publish_control(self, chi_d: float, vel_cmd: np.ndarray, x: float, y: float):
        """Compute and publish velocity command"""
        # Get heading gain from parameters
        k_h = float(self.get_parameter('heading_gain').value)

        twist = Twist()
        # Compute forward velocity in body frame using current heading
        v_forward = vel_cmd[0] * \
            math.cos(self._yaw) + vel_cmd[1] * math.sin(self._yaw)
        # Use computed forward speed (ensuring non-negative speed)
        twist.linear.x = max(v_forward, 0.0)
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        # Compute and normalize heading error
        heading_error = math.atan2(
            math.sin(chi_d - self._yaw), math.cos(chi_d - self._yaw))
        twist.angular.z = k_h * heading_error
        self._cmd_pub.publish(twist)


class LOSGuidanceForceCtrROS(BaseLOSGuidanceROS):
    """LOS Guidance implementation with force control (Wrench messages)"""

    def __init__(self):
        super().__init__('los_guidance_force_server')

    def _declare_control_parameters(self):
        """Force control uses additional parameters for force/torque computation"""
        self.declare_parameter('heading_rate_gain', 0.3)
        self.declare_parameter('surge_p_gain', 1.0)
        self.declare_parameter('surge_d_gain', 0.2)
        self.declare_parameter('max_force_xy', 1.0)
        self.declare_parameter('max_torque_z', 0.2)

    def _declare_vehicle_state(self):
        """Force control needs velocity measurements"""
        self._linear_velocity = None
        self._angular_velocity_z = None

    def _setup_control_publisher(self):
        """Setup Wrench publisher for force commands"""
        self._force_pub = self.create_publisher(
            Wrench, 'control/force/command/los', 10)

    def _update_odom_helper(self, msg: Odometry):
        """Force control needs linear and angular velocities"""
        twist = msg.twist.twist
        self._linear_velocity = (twist.linear.x, twist.linear.y)
        self._angular_velocity_z = twist.angular.z

    def _compute_and_publish_control(self, chi_d: float, vel_cmd: np.ndarray, x: float, y: float):
        """Compute and publish force command"""
        wrench = self.compute_force_command(vel_cmd, chi_d)
        self._force_pub.publish(wrench)

    def compute_force_command(self, vel_cmd: np.ndarray, chi_d: float) -> Wrench:
        """Compute force and torque command from velocity command and heading"""
        vel_body_ref = self._world_to_body(vel_cmd, self._yaw)
        vel_body_meas = np.array(self._linear_velocity
                                 if self._linear_velocity is not None else (0.0, 0.0))

        surge_kp = float(self.get_parameter('surge_p_gain').value)
        surge_kd = float(self.get_parameter('surge_d_gain').value)

        force_x = surge_kp * (vel_body_ref[0] - vel_body_meas[0]) - \
            surge_kd * vel_body_meas[0]
        # Suppress sway forces to avoid crabbing behavior
        force_y = 0.0

        max_force = float(self.get_parameter('max_force_xy').value)
        force_x = self._saturate(force_x, max_force)

        heading_error = math.atan2(
            math.sin(chi_d - self._yaw), math.cos(chi_d - self._yaw))
        yaw_rate = self._angular_velocity_z if self._angular_velocity_z is not None else 0.0
        k_h = float(self.get_parameter('heading_gain').value)
        k_hr = float(self.get_parameter('heading_rate_gain').value)
        torque_z = k_h * heading_error - k_hr * yaw_rate
        max_torque = float(self.get_parameter('max_torque_z').value)
        torque_z = self._saturate(torque_z, max_torque)

        wrench = Wrench()
        wrench.force.x = float(force_x)
        wrench.force.y = float(force_y)
        wrench.force.z = 0.0
        wrench.torque.x = 0.0
        wrench.torque.y = 0.0
        wrench.torque.z = float(torque_z)
        return wrench

    @staticmethod
    def _world_to_body(vel_cmd: np.ndarray, yaw: float) -> np.ndarray:
        """Convert velocity from world frame to body frame"""
        c = math.cos(yaw)
        s = math.sin(yaw)
        return np.array([
            c * vel_cmd[0] + s * vel_cmd[1],
            -s * vel_cmd[0] + c * vel_cmd[1]
        ])

    @staticmethod
    def _saturate(value: float, limit: float) -> float:
        """Saturate a value to a symmetric limit [-limit, limit]"""
        if limit <= 0.0:
            return float(value)
        return float(max(-limit, min(limit, value)))


def main(args=None):
    rclpy.init(args=args)

    # Uncomment one of the following to run the desired controller:
    # node = LOSGuidanceVelocityROS()
    node = LOSGuidanceForceCtrROS()

    executor = SingleThreadedExecutor()
    try:
        executor.add_node(node)
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down on SIGINT (Ctrl+C)')
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
