#!/usr/bin/env python3
"""
ROS2 Action Server for LOS Guidance using cybership_interfaces/LOSGuidance
"""
import math
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.action import CancelResponse, GoalResponse
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, DurabilityPolicy

from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import Twist, Vector3, Point
from visualization_msgs.msg import Marker
from tf_transformations import euler_from_quaternion

from cybership_interfaces.action import LOSGuidance

# Import underlying LOS guidance implementation
from cybership_controller.guidance.los import LOSGuidance as BaseLOSGuidance
import numpy as np
from scipy.interpolate import splev

class LOSGuidanceROS(Node):
    def __init__(self):
        super().__init__('los_guidance_server', namespace="cybership")
        # Parameters
        self.declare_parameter('desired_speed', 0.3)
        self.declare_parameter('lookahead', 1.0)
        # Heading control gain
        self.declare_parameter('heading_gain', 2.0)
        # Publishers and Subscribers
        self._cmd_pub = self.create_publisher(Twist, 'control/velocity/command', 10)
        # Publish markers with transient local durability so RViZ receives past markers
        self._marker_pub = self.create_publisher(Marker, 'visualization_marker', 10)
        self._odom_sub = self.create_subscription(Odometry, 'measurement/odom', self.odom_callback, 10)
        # Vehicle pose
        self._position = None
        self._yaw = None
        # Action Server
        self._action_server = ActionServer(
            self,
            LOSGuidance,
            'los_guidance',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback)
        self.get_logger().info('LOS Guidance Action Server started')

    def odom_callback(self, msg: Odometry):
        # Update position and heading
        self._position = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        q = msg.pose.pose.orientation
        # Convert quaternion to yaw using tf_transformations
        quat = [q.x, q.y, q.z, q.w]
        _, _, yaw = euler_from_quaternion(quat)
        self._yaw = yaw

    def goal_callback(self, goal_request):
        # Accept all goals
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle):
        # Extract waypoints from Path and include current vehicle position as initial point
        path_msg: Path = goal_handle.request.path
        # Build waypoints list, prepending vehicle's initial position if available
        waypoints = []
        if self._position is not None:
            waypoints.append(self._position)


        waypoints.extend([(pose.pose.position.x, pose.pose.position.y)
                          for pose in path_msg.poses])
        # Initialize guidance
        V_d = float(self.get_parameter('desired_speed').value)
        delta = float(self.get_parameter('lookahead').value)
        # Heading control gain
        k_h = float(self.get_parameter('heading_gain').value)
        guidance = BaseLOSGuidance(waypoints, V_d=V_d, delta=delta)
        last_wp = waypoints[-1]
        # Visualize spline path
        self.publish_spline_marker(guidance)
        # Define rate for loop (10 Hz)
        rate = self.create_rate(10)

        # Publish path marker

        while rclpy.ok():
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                return LOSGuidance.Result(success=False)
            if self._position is None or self._yaw is None:
                rate.sleep()
                continue
            x, y = self._position
            chi_d, vel_cmd = guidance.guidance(x, y)
            twist = Twist()
            # Compute forward velocity in body frame using current heading
            v_forward = vel_cmd[0] * math.cos(self._yaw) + vel_cmd[1] * math.sin(self._yaw)
            # Use computed forward speed (ensuring non-negative speed)
            twist.linear.x = max(v_forward, 0.0)
            twist.linear.y = 0.0
            twist.linear.z = 0.0
            # Compute and normalize heading error
            heading_error = math.atan2(math.sin(chi_d - self._yaw), math.cos(chi_d - self._yaw))
            twist.angular.z = k_h * heading_error
            self._cmd_pub.publish(twist)
            # Feedback
            feedback = LOSGuidance.Feedback()
            feedback.heading = chi_d
            feedback.vel_cmd = Vector3(x=vel_cmd[0], y=vel_cmd[1], z=0.0)
            goal_handle.publish_feedback(feedback)
            self.publish_path_marker(path_msg)
            self.publish_arrow_marker(x, y, vel_cmd)
            # Check completion
            if math.hypot(x - last_wp[0], y - last_wp[1]) < delta:
                break
            rate.sleep()

        goal_handle.succeed()
        result = LOSGuidance.Result()
        return result

    def publish_path_marker(self, path_msg: Path):
        marker = Marker()
        marker.header = path_msg.header
        marker.ns = 'los_path'
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = 0.05
        marker.color.r = 1.0; marker.color.g = 0.0; marker.color.b = 0.0; marker.color.a = 1.0
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
        marker.color.r = 0.0; marker.color.g = 1.0; marker.color.b = 0.5; marker.color.a = 1.0
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
        marker.color.r = 0.0; marker.color.g = 0.0; marker.color.b = 1.0; marker.color.a = 1.0
        # Sample spline path points
        ts = np.linspace(0, 1, 100)
        spline_pts = np.array(splev(ts, guidance.tck)).T
        marker.points = [Point(x=pt[0], y=pt[1], z=0.0) for pt in spline_pts]
        self._marker_pub.publish(marker)


def main(args=None):
    rclpy.init(args=args)
    node = LOSGuidanceROS()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()
    executor.shutdown()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()