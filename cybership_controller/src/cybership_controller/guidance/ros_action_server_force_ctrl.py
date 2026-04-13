#!/usr/bin/env python3
"""
ROS2 Action Server for LOS Guidance using cybership_interfaces/LOSGuidance
"""
import math
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, ActionClient
from rclpy.action import CancelResponse, GoalResponse
from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.qos import QoSProfile, DurabilityPolicy
from rcl_interfaces.msg import SetParametersResult

from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import Wrench, Vector3, Point
from visualization_msgs.msg import Marker
from tf_transformations import euler_from_quaternion

from cybership_interfaces.action import LOSGuidance

# Import underlying LOS guidance implementation
from cybership_controller.guidance.los import LOSGuidance as BaseLOSGuidance
import numpy as np
import typing
from scipy.interpolate import splev


class LOSGuidanceROS(Node):
    def __init__(self):
        super().__init__('los_guidance_server', namespace="cybership")
        # Parameters
        self.declare_parameter('desired_speed', 0.3)
        self.declare_parameter('lookahead', 1.4)
        # Heading control gain
        self.declare_parameter('heading_gain', 1.0)
        self.declare_parameter('heading_rate_gain', 0.3)
        # Velocity-force controller gains
        self.declare_parameter('surge_p_gain', 1.0)
        self.declare_parameter('surge_d_gain', 0.2)
        self.declare_parameter('max_force_xy', 1.0)
        self.declare_parameter('max_torque_z', 0.2)
        # Publishers and Subscribers
        self._force_pub = self.create_publisher(
            Wrench, 'control/force/command/los', 10)
        # Publish markers with transient local durability so RViZ receives past markers
        self._marker_pub = self.create_publisher(
            Marker, 'visualization_marker', 10)
        self._odom_sub = self.create_subscription(
            Odometry, 'measurement/odom', self.odom_callback, 10)
        # Vehicle pose
        self._position = None
        self._yaw = None
        self._linear_velocity = None
        self._angular_velocity_z = None
        # Action Server
        self._action_server = ActionServer(
            self,
            LOSGuidance,
            'los_guidance',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=ReentrantCallbackGroup())

        self.goal_uuids: typing.List = []

        # Guidance loop state
        self.active_goal_handle = None
        self.guidance = None
        self.last_wp = None
        self.guidance_hz = 10
        self.guidance_timer = None
        self.path_msg = None

        # Allow runtime tuning of controller/guidance parameters.
        self._param_cb_handle = self.add_on_set_parameters_callback(
            self.on_parameter_update)

        self.get_logger().info('LOS Guidance Action Server started')

    def on_parameter_update(self, params):
        positive_only = {
            'desired_speed',
            'lookahead',
            'surge_p_gain',
            'max_force_xy',
            'max_torque_z',
        }
        non_negative = {
            'heading_rate_gain',
            'surge_d_gain',
            'heading_gain',
        }

        for param in params:
            if param.name in positive_only:
                if float(param.value) <= 0.0:
                    return SetParametersResult(
                        successful=False,
                        reason=f"{param.name} must be > 0.0")
            elif param.name in non_negative:
                if float(param.value) < 0.0:
                    return SetParametersResult(
                        successful=False,
                        reason=f"{param.name} must be >= 0.0")

        # If guidance is active, apply changed guidance parameters immediately.
        if self.guidance is not None:
            for param in params:
                if param.name == 'desired_speed' and hasattr(self.guidance, 'V_d'):
                    self.guidance.V_d = float(param.value)
                elif param.name == 'lookahead' and hasattr(self.guidance, 'delta'):
                    self.guidance.delta = float(param.value)

        if params:
            changed = ', '.join([p.name for p in params])
            self.get_logger().info(f"Updated parameters: {changed}")

        return SetParametersResult(successful=True)

    def odom_callback(self, msg: Odometry):
        # Update position and heading
        self._position = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        q = msg.pose.pose.orientation
        # Convert quaternion to yaw using tf_transformations
        quat = [q.x, q.y, q.z, q.w]
        _, _, yaw = euler_from_quaternion(quat)
        self._yaw = yaw
        twist = msg.twist.twist
        self._linear_velocity = (twist.linear.x, twist.linear.y)
        self._angular_velocity_z = twist.angular.z

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

        if not goal_handle.is_active:
            self.get_logger().info(f"LOS guidance no longer active.")
            self.active_goal_handle = None
            self.guidance_timer.cancel()
            return

        if self._position is None or self._yaw is None:
            return

        x, y = self._position
        dt = 1.0 / self.guidance_hz
        chi_d, vel_cmd, extras = self.guidance.guidance(x, y, dt=dt)

        wrench = self.compute_force_command(vel_cmd, chi_d)
        self._force_pub.publish(wrench)

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
            goal_handle.succeed()
            self.get_logger().info("LOS guidance finished (result empty).")
            self.active_goal_handle = None
            self.guidance_timer.cancel()

    async def execute_callback(self, goal_handle: LOSGuidance.Goal) -> LOSGuidance.Result:
        """Non-blocking action server callback. Guidance loop runs in timer."""
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

        # Store goal handle for timer callback
        self.active_goal_handle = goal_handle

        # Start guidance loop timer
        if self.guidance_timer is not None:
            self.guidance_timer.cancel()
        self.guidance_timer = self.create_timer(
            1.0 / self.guidance_hz, self.guidance_loop_timer)

        # Return immediately; guidance loop runs in timer
        return LOSGuidance.Result()

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

    def compute_force_command(self, vel_cmd: np.ndarray, chi_d: float) -> Wrench:
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
        c = math.cos(yaw)
        s = math.sin(yaw)
        return np.array([
            c * vel_cmd[0] + s * vel_cmd[1],
            -s * vel_cmd[0] + c * vel_cmd[1]
        ])

    @staticmethod
    def _saturate(value: float, limit: float) -> float:
        if limit <= 0.0:
            return float(value)
        return float(max(-limit, min(limit, value)))

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


def main(args=None):
    rclpy.init(args=args)
    node = LOSGuidanceROS()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
