#!/usr/bin/env python3

# ----------------------------------------------------------------------------
# This code is part of the MCSimPython toolbox and repository
# Created By: Jan-Erik Hygen
# Created Date: 2023-01-30,
# Revised: 2025-01-31 Kristian Magnus Roen   Now fitting the MC-Gym for csad.
# Revised: 2025-03-26 Emir Cem Gezer Applied to the Cybership Software Suite
# Tested:
# Copyright (C) 2025: NTNU, Trondheim
# Licensed under GPL-3.0-or-later
# ---------------------------------------------------------------------------

import time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from geometry_msgs.msg import Wrench, Pose2D, PoseStamped, TwistStamped
from nav_msgs.msg import Odometry
import math
import numpy as np
from scipy.spatial.transform import Rotation as R
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionServer, GoalResponse, CancelResponse, ActionClient
from rclpy.executors import MultiThreadedExecutor
from cybership_tests.go_to_client import NavigateToPoseClient
from shoeboxpy.model3dof import Shoebox
from visualization_msgs.msg import Marker
from std_msgs.msg import Float32MultiArray
from std_srvs.srv import SetBool, Trigger, Empty
from cybership_controller.position.reference_filter import ThirdOrderReferenceFilter

try:
    from cybership_interfaces.msg import PerformanceMetrics
except ImportError:
    print("cybership_msgs not found. Skipping performance metrics.")


def wrap_to_pi(angle):
    """
    Wrap an angle in radians to the interval [-pi, pi].
    """
    return (angle + math.pi) % (2 * math.pi) - math.pi


def Rz(psi):
    """3DOF Rotation matrix about z-axis.

    Parameters
    ----------
    psi : float
        Yaw angle (rad)

    Returns
    -------
    Rz : array_like
        3x3 rotation matrix.

    """
    return np.array(
        [[np.cos(psi), -np.sin(psi), 0],
         [np.sin(psi), np.cos(psi), 0], [0, 0, 1]]
    )


def saturate(x, z):
    """
    Saturation function: returns x / (|x| + z)
    This ensures a smooth saturation between -1 and 1.
    """
    return x / (np.abs(x) + z)


class PositionController(Node):
    def __init__(self):
        super().__init__("position_controller", namespace="cybership")

        # Declare parameters with default values
        self.declare_parameters(
            namespace='',
            parameters=[
                # Controller gains
                ('control.p_gain.pos', 4.0),
                ('control.i_gain.pos', 0.2),
                ('control.d_gain.pos', 0.2),
                ('control.p_gain.vel', 0.7),
                ('control.i_gain.vel', 0.1),
                ('control.d_gain.vel', 0.5),
                ('control.p_gain.yaw', 1.3),
                ('control.i_gain.yaw', 0.2),
                ('control.d_gain.yaw', 1.0),

                # Controller limits
                ('control.max_integral_error.pos', 1.0),
                ('control.max_integral_error.yaw', 1.4),
                ('control.saturation.pos', 0.1),
                ('control.saturation.yaw', 0.1),

                # Tolerances
                ('control.tolerance.pos', 0.25),
                ('control.tolerance.yaw', 0.1),
                # Require being within tolerance for this long before success
                ('control.success_hold_time', 1.0),

                # Vessel properties
                ('vessel.length', 1.0),
                ('vessel.beam', 0.3),
                ('vessel.draft', 0.02),

                # Reference filter parameters
                ('filter.omega', [0.15, 0.15, 0.15]),
                ('filter.delta', [0.8, 0.8, 0.8]),

                # Performance metrics
                ('metrics.window_size', 200),
                ('metrics.interval', 1.0),

                # Time step
                ('dt', 0.01),
            ]
        )

        # Initialize the 3rd order reference filter.
        # Here, we treat [x, y, yaw] as the 3D pose to be smoothed.
        self.ref_filter = None

        self.start_time = None

        self.error_window = []  # For position error
        self.error_yaw_window = []  # For yaw error
        self.sample_count = 0
        self.last_metrics_time = 0.0

        # Add parameter callback for runtime updates
        self.add_on_set_parameters_callback(self.parameters_callback)

        # Publisher to send control commands (force and torque)
        self.control_pub = self.create_publisher(
            Wrench, "control/force/command", 10)
        # Debug publishers for tracking performance metrics
        self.debug_pose_pub = self.create_publisher(
            PoseStamped, "control/pose/debug/reference_pose", 10)
        self.debug_vel_pub = self.create_publisher(
            TwistStamped, "control/pose/debug/reference_velocity", 10)
        self.debug_error_pose_pub = self.create_publisher(
            Pose2D, "control/pose/debug/tracking_error_position", 10)
        self.debug_error_vel_pub = self.create_publisher(
            TwistStamped, "control/pose/debug/tracking_error_velocity", 10)

        self.debug_metrics_pub = None
        if PerformanceMetrics is not None:
            self.debug_metrics_pub = self.create_publisher(
                PerformanceMetrics, "control/pose/debug/performance_metrics", 10)

        # self.create_subscription(PoseStamped, "/goal_pose", self.goal_pose_callback, 10)
        self.create_subscription(
            Odometry, "measurement/odom", self.odom_callback, 10)

        self.marker_pub = self.create_publisher(
            Marker, "visualization_marker", 10)

        # Get time step from parameters
        self.dt = self.get_parameter('dt').value

        # Initialize controller parameters from ROS parameters
        self.update_configuration()

        # Latest odometry message storage
        self.latest_odom = None

        # --- Target state ---
        # Set initial desired target position and orientation.
        self.target_x = None  # target x position (meters)
        self.target_y = None  # target y position (meters)
        self.target_yaw = None  # target yaw (radians)

        # Track integral error
        self.integral_error_pos = np.zeros(2)
        self.integral_error_yaw = 0.0

        self.error_pos = np.zeros(2)
        self.error_yaw = 0.0

        self.get_logger().info(
            "Goto Point Controller (Reference Filter Version) Initialized with parameters."
        )

        # Timer for periodic control updates
        self.timer = self.create_timer(self.dt, self.control_loop)

        # --- Action Server using nav2_msgs/NavigateToPose ---
        self._action_server = ActionServer(
            self,
            NavigateToPose,
            "navigate_to_pose",
            execute_callback=self.execute_callback,
            goal_callback=self.action_goal_callback,
            cancel_callback=self.action_cancel_callback,
        )
        # Service to enable/disable the controller
        self.enabled = True
        self.state_service = self.create_service(
            SetBool,
            f"{self.get_name()}/change_state",
            self.change_state_callback
        )

    def update_configuration(self):
        """Update controller configuration from parameters"""
        # Get controller gains
        self.Kp_pos = self.get_parameter('control.p_gain.pos').value
        self.Ki_pos = self.get_parameter('control.i_gain.pos').value
        self.Kd_pos = self.get_parameter('control.d_gain.pos').value
        self.Kp_vel = self.get_parameter('control.p_gain.vel').value
        self.Ki_vel = self.get_parameter('control.i_gain.vel').value
        self.Kd_vel = self.get_parameter('control.d_gain.vel').value
        self.Kp_yaw = self.get_parameter('control.p_gain.yaw').value
        self.Ki_yaw = self.get_parameter('control.i_gain.yaw').value
        self.Kd_yaw = self.get_parameter('control.d_gain.yaw').value

        # Get controller limits
        self.max_integral_error_pos = self.get_parameter(
            'control.max_integral_error.pos').value
        self.max_integral_error_yaw = self.get_parameter(
            'control.max_integral_error.yaw').value
        self.saturation_pos = self.get_parameter(
            'control.saturation.pos').value
        self.saturation_yaw = self.get_parameter(
            'control.saturation.yaw').value

        # Get tolerances
        self.pos_tol = self.get_parameter('control.tolerance.pos').value
        self.yaw_tol = self.get_parameter('control.tolerance.yaw').value
        self.success_hold_time = self.get_parameter('control.success_hold_time').value

        # Get vessel properties
        vessel_length = self.get_parameter('vessel.length').value
        vessel_beam = self.get_parameter('vessel.beam').value
        vessel_draft = self.get_parameter('vessel.draft').value

        # Create vessel model
        self.shoebox = Shoebox(
            L=vessel_length,
            B=vessel_beam,
            T=vessel_draft,
        )

        # Get performance metrics parameters
        self.window_size = self.get_parameter('metrics.window_size').value
        self.metrics_interval = self.get_parameter('metrics.interval').value

        # Reset reference filter if parameters change
        if self.ref_filter is not None and self.latest_odom is not None:
            # Get filter parameters
            filter_omega = self.get_parameter('filter.omega').value
            filter_delta = self.get_parameter('filter.delta').value

            # Save current state
            current_eta = self.ref_filter.eta_d

            # Create new filter with updated parameters
            self.ref_filter = ThirdOrderReferenceFilter(
                dt=self.dt,
                omega=filter_omega,
                delta=filter_delta,
                initial_eta=current_eta,
            )
            self.ref_filter.eta_d = current_eta

        self.get_logger().info(
            f"Updated configuration - vessel: [{vessel_length}, {vessel_beam}, {vessel_draft}], " +
            f"POS gains: P={self.Kp_pos}, I={self.Ki_pos}, D={self.Kd_pos}, " +
            f"YAW gains: P={self.Kp_yaw}, I={self.Ki_yaw}, D={self.Kd_yaw}"
        )

    def parameters_callback(self, params):
        """Handle parameter updates"""
        update_needed = False

        for param in params:
            if param.name.startswith(('control.', 'vessel.', 'filter.', 'metrics.')):
                update_needed = True

            if param.name == 'dt':
                self.dt = param.value
                # Recreate the timer with new dt
                self.timer.cancel()
                self.timer = self.create_timer(self.dt, self.control_loop)
                update_needed = True

        # Update configuration if parameters changed
        if update_needed:
            self.update_configuration()

        return True  # Accept all parameter changes

    # def odom_callback(self, msg: Odometry):
    #     """
    #     Callback to update the latest odometry measurement.
    #     """
    #     if self.ref_filter is None:
    #         print("Setting target position from odometry.")
    #         self.target_x = msg.pose.pose.position.x
    #         self.target_y = msg.pose.pose.position.y
    #         _, _, self.target_yaw = R.from_quat(
    #             [
    #                 msg.pose.pose.orientation.x,
    #                 msg.pose.pose.orientation.y,
    #                 msg.pose.pose.orientation.z,
    #                 msg.pose.pose.orientation.w,
    #             ]
    #         ).as_euler("xyz", degrees=False)

    #         # Get filter parameters from ROS parameters
    #         filter_omega = self.get_parameter('filter.omega').value
    #         filter_delta = self.get_parameter('filter.delta').value

    #         self.ref_filter = ThirdOrderReferenceFilter(
    #             dt=self.dt,
    #             omega=filter_omega,
    #             delta=filter_delta,
    #             initial_eta=[self.target_x, self.target_y, self.target_yaw],
    #         )
    #         self.ref_filter.eta_d = np.array(
    #             [self.target_x, self.target_y, self.target_yaw]
    #         )

    #     self.latest_odom = msg

    def odom_callback(self, msg: Odometry):
        current_x = msg.pose.pose.position.x
        current_y = msg.pose.pose.position.y
        _, _, current_yaw = R.from_quat([
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w
        ]).as_euler("xyz", degrees=False)

        # Always store latest odom
        self.latest_odom = msg

        if self.ref_filter is None:
            # Initialize the filter to CURRENT pose (not the target)
            filter_omega = self.get_parameter('filter.omega').value
            filter_delta = self.get_parameter('filter.delta').value
            self.ref_filter = ThirdOrderReferenceFilter(
                dt=self.dt,
                omega=filter_omega,
                delta=filter_delta,
                initial_eta=[current_x, current_y, current_yaw],
            )
            self.ref_filter.eta_d = np.array([current_x, current_y, current_yaw])

            # Only seed a target from odom if no goal has ever been set
            if self.target_x is None:
                self.target_x = current_x
                self.target_y = current_y
                self.target_yaw = current_yaw

    def process_performance_metrics(self, desired_pose, desired_vel, error_pos, error_vel, error_yaw):
        """
        Process and publish debug information using a moving window approach.
        """
        # Publish reference pose (from reference filter)
        debug_ref_pose = PoseStamped()
        debug_ref_pose.header.stamp = self.get_clock().now().to_msg()
        debug_ref_pose.header.frame_id = "world"
        debug_ref_pose.pose.position.x = desired_pose[0]
        debug_ref_pose.pose.position.y = desired_pose[1]
        debug_ref_pose.pose.position.z = 0.0
        # Convert yaw to quaternion
        q = R.from_euler('z', desired_pose[2]).as_quat()
        debug_ref_pose.pose.orientation.x = q[0]
        debug_ref_pose.pose.orientation.y = q[1]
        debug_ref_pose.pose.orientation.z = q[2]
        debug_ref_pose.pose.orientation.w = q[3]
        self.debug_pose_pub.publish(debug_ref_pose)

        debug_ref_vel = TwistStamped()
        debug_ref_vel.header.stamp = self.get_clock().now().to_msg()
        debug_ref_vel.header.frame_id = "base_link"
        debug_ref_vel.twist.linear.x = desired_vel[0]
        debug_ref_vel.twist.linear.y = desired_vel[1]
        debug_ref_vel.twist.angular.z = desired_vel[2]
        self.debug_vel_pub.publish(debug_ref_vel)

        debug_error_vel = TwistStamped()
        debug_error_vel.header.stamp = self.get_clock().now().to_msg()
        debug_error_vel.header.frame_id = "base_link"
        debug_error_vel.twist.linear.x = error_vel[0]
        debug_error_vel.twist.linear.y = error_vel[1]
        debug_error_vel.twist.angular.z = error_yaw
        self.debug_error_vel_pub.publish(debug_error_vel)

        # Publish error information
        debug_error_pose = Pose2D()
        debug_error_pose.x = error_pos[0]
        debug_error_pose.y = error_pos[1]
        debug_error_pose.theta = error_yaw
        self.debug_error_pose_pub.publish(debug_error_pose)

        # Update metrics using moving window
        error_norm = np.sqrt(error_pos[0]**2 + error_pos[1]**2)

        # Initialize start time if not already set
        if self.start_time is None:
            self.start_time = self.get_clock().now().nanoseconds / 1e9

        # Add newest error to window
        self.error_window.append(error_norm)
        self.error_yaw_window.append(abs(error_yaw))

        # Keep window at fixed size
        if len(self.error_window) > self.window_size:
            self.error_window.pop(0)
            self.error_yaw_window.pop(0)

        self.sample_count += 1

        current_time = self.get_clock().now().nanoseconds / 1e9
        if current_time - self.last_metrics_time >= self.metrics_interval and len(self.error_window) > 0:
            self.last_metrics_time = current_time

            # Calculate window statistics
            error_array = np.array(self.error_window)

            metrics_msg = PerformanceMetrics()
            metrics_msg.header.stamp = self.get_clock().now().to_msg()
            metrics_msg.header.frame_id = "voyager"
            metrics_msg.message = "Position tracking error (meters)"

            # Calculate statistics over the window
            metrics_msg.mean = np.mean(error_array)
            metrics_msg.median = np.median(error_array)
            metrics_msg.rms = np.sqrt(np.mean(np.square(error_array)))
            metrics_msg.min = np.min(error_array)
            metrics_msg.max = np.max(error_array)
            metrics_msg.stddev = np.std(error_array)

            self.debug_metrics_pub.publish(metrics_msg)

            self.get_logger().info(
                f"Position tracking metrics - Mean: {metrics_msg.mean:.3f}m, "
                f"RMS: {metrics_msg.rms:.3f}m, Max: {metrics_msg.max:.3f}m"
            )

    def control_loop(self):
        """
        Periodic control loop to update reference filter and publish control commands.
        This version uses a 3rd order reference filter to generate a smooth desired trajectory.
        """
        # Skip control loop if controller is disabled
        if not self.enabled:
            return

        if self.latest_odom is None:
            return

        if self.ref_filter is None:
            return

        # Extract current position from odometry (global frame)
        pos = self.latest_odom.pose.pose.position
        current_x = pos.x
        current_y = pos.y

        # Extract yaw (heading) from quaternion orientation
        orientation = self.latest_odom.pose.pose.orientation
        rot = R.from_quat([orientation.x, orientation.y,
                          orientation.z, orientation.w])
        _, _, current_yaw = rot.as_euler("xyz", degrees=False)

        # Also get current velocities from odometry for the derivative term.
        current_vx = self.latest_odom.twist.twist.linear.x
        current_vy = self.latest_odom.twist.twist.linear.y
        current_yaw_rate = self.latest_odom.twist.twist.angular.z

        # Update the reference filter with the latest target pose.
        # This "command" is smoothed by the filter.
        ref = np.array([self.target_x, self.target_y, self.target_yaw])
        self.ref_filter.set_eta_r(ref)
        self.ref_filter.update()

        # Retrieve filtered outputs: desired pose, velocity, and acceleration.
        desired_pose = self.ref_filter.eta_d  # [x, y, yaw]
        desired_vel = self.ref_filter.eta_d_dot  # velocity
        # acceleration (feedforward term)
        desired_acc = self.ref_filter.eta_d_ddot

        # Compute errors (position and yaw) between the filtered desired state and current state.
        error_pos = np.array(
            [desired_pose[0] - current_x, desired_pose[1] - current_y])
        error_yaw = wrap_to_pi(desired_pose[2] - current_yaw)

        self.error_pos = error_pos
        self.error_yaw = error_yaw

        # Compute velocity errors
        current_vel = np.array([current_vx, current_vy])
        error_vel = np.array(
            [desired_vel[0] - current_vx, desired_vel[1] - current_vy])
        error_yaw_rate = desired_vel[2] - current_yaw_rate

        acc = self.shoebox.M_eff @ self.ref_filter.get_nu_d()

        self.integral_error_pos += error_pos * self.dt
        self.integral_error_yaw += error_yaw * self.dt

        # Apply saturation to the integral error
        self.integral_error_pos = np.clip(
            self.integral_error_pos, -self.max_integral_error_pos, self.max_integral_error_pos)
        self.integral_error_yaw = np.clip(
            self.integral_error_yaw, -self.max_integral_error_yaw, self.max_integral_error_yaw)

        # Compute control commands.
        # For position, we use feedforward desired acceleration plus a PID correction.
        world_x = (
            acc[0]
            + self.Kp_pos * error_pos[0]
            + self.Kd_vel * error_vel[0]
            + self.Ki_pos * self.integral_error_pos[0]
        )
        world_y = (
            acc[1]
            + self.Kp_pos * error_pos[1]
            + self.Kd_vel * error_vel[1]
            + self.Ki_pos * self.integral_error_pos[1]
        )
        world_yaw = (
            acc[2]
            + self.Kp_yaw * error_yaw
            + self.Kd_yaw * error_yaw_rate
            + self.Ki_yaw * self.integral_error_yaw
        )

        # Process and publish performance metrics
        self.process_performance_metrics(
            desired_pose, desired_vel, error_pos, error_vel, error_yaw)

        control_x, control_y, control_yaw = Rz(
            current_yaw).T @ np.array([world_x, world_y, world_yaw])

        # Optionally apply saturation.
        # control_x = saturate(control_x, self.saturation_pos)
        # control_y = saturate(control_y, self.saturation_pos)
        # control_yaw = saturate(control_yaw, self.saturation_yaw)

        wrench_msg = Wrench()

        # Publish these instead
        wrench_msg.force.x = control_x
        wrench_msg.force.y = control_y
        wrench_msg.force.z = 0.0  # No vertical force
        wrench_msg.torque.x = 0.0
        wrench_msg.torque.y = 0.0
        wrench_msg.torque.z = control_yaw

        # Publish the control command.
        self.control_pub.publish(wrench_msg)

        # Debug logging.
        self.get_logger().debug(
            f"Current: ({current_x:.2f}, {current_y:.2f}, {current_yaw:.2f}), "
            f"Filtered desired: ({desired_pose[0]:.2f}, {desired_pose[1]:.2f}, {desired_pose[2]:.2f}), "
            f"Control: ({control_x:.2f}, {control_y:.2f}, {control_yaw:.2f})"
        )

    def action_goal_callback(self, goal_request):
        self.get_logger().info("Received NavigateToPose goal request")
        return GoalResponse.ACCEPT

    def action_cancel_callback(self, goal_handle):
        self.get_logger().info("Received cancel request for NavigateToPose")
        return CancelResponse.ACCEPT

    def change_state_callback(self, request, response):
        """Service callback to enable/disable the controller."""
        if request.data:
            self.enabled = True
            # Reset controller state
            self.start_time = None
            self.error_window.clear()
            self.error_yaw_window.clear()
            self.sample_count = 0
            self.last_metrics_time = 0.0
            self.integral_error_pos = np.zeros(2)
            self.integral_error_yaw = 0.0
            self.error_pos = np.zeros(2)
            self.error_yaw = 0.0
            self.ref_filter = None

            response.success = True
            response.message = "Controller enabled and reset."
        else:
            self.enabled = False
            # Publish zero forces and torques to reset the controller
            zero_wrench = Wrench()
            self.control_pub.publish(zero_wrench)

            response.success = True
            response.message = "Controller disabled."
        return response

    def execute_callback(self, goal_handle):
        self.get_logger().info("Executing NavigateToPose goal...")

        # Extract target pose from the goal message
        target_pose: PoseStamped = goal_handle.request.pose
        # Warn if frame differs from odom, since we compute distances in odom frame
        if target_pose.header.frame_id and target_pose.header.frame_id not in ("odom", "/odom"):
            self.get_logger().warn(
                f"Goal frame_id '{target_pose.header.frame_id}' differs from 'odom'; no TF transform is applied."
            )
        self.target_x = target_pose.pose.position.x
        self.target_y = target_pose.pose.position.y

        self.get_logger().info(
            f"Target position: ({self.target_x}, {self.target_y})")

        # Convert quaternion to yaw angle
        orientation = target_pose.pose.orientation
        rot = R.from_quat([
            orientation.x,
            orientation.y,
            orientation.z,
            orientation.w]
        )
        _, _, self.target_yaw = rot.as_euler("xyz", degrees=False)

        feedback_msg = NavigateToPose.Feedback()
        current_feedback_pose = PoseStamped()

        self.publish_target_pose_marker(target_pose)

        # Loop until the robot reaches the target (within tolerance for a hold time) or the goal is canceled.
        within_since = None
        while rclpy.ok():  # Loop until target reached or goal is canceled
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info("NavigateToPose goal canceled")
                result = NavigateToPose.Result()
                return result

            # Wait for odometry
            if self.latest_odom is None:
                time.sleep(1.0)
                continue

            # Extract current pose from odometry
            current_odom = self.latest_odom.pose.pose
            current_feedback_pose.header.stamp = self.get_clock().now().to_msg()
            current_feedback_pose.header.frame_id = "odom"

            current_feedback_pose.pose.position = current_odom.position
            current_feedback_pose.pose.orientation = current_odom.orientation

            # Compute remaining distance
            error_norm = np.sqrt(
                (self.target_x - current_feedback_pose.pose.position.x) ** 2
                + (self.target_y - current_feedback_pose.pose.position.y) ** 2
            )
            _, _, current_yaw = R.from_quat(
                [
                    current_feedback_pose.pose.orientation.x,
                    current_feedback_pose.pose.orientation.y,
                    current_feedback_pose.pose.orientation.z,
                    current_feedback_pose.pose.orientation.w,
                ]
            ).as_euler("xyz", degrees=False)
            error_yaw = wrap_to_pi(self.target_yaw - current_yaw)

            feedback_msg.current_pose = current_feedback_pose
            feedback_msg.distance_remaining = float(error_norm)

            goal_handle.publish_feedback(feedback_msg)
            self.get_logger().debug(
                f"Distance remaining: {error_norm:.2f}, angle error: {error_yaw:.2f}"
            )

            # Check if target is reached (both position and yaw) and held
            now_t = self.get_clock().now().nanoseconds / 1e9
            if error_norm < self.pos_tol and abs(error_yaw) < self.yaw_tol:
                if within_since is None:
                    within_since = now_t
                if (now_t - within_since) >= self.success_hold_time:
                    # Mark large errors to avoid lingering metrics effects
                    self.error_pos = np.array([np.inf, np.inf])
                    self.error_yaw = np.inf
                    break
            else:
                within_since = None

            time.sleep(1.0)  # Publish feedback at ~1Hz

        goal_handle.succeed()
        result = NavigateToPose.Result()
        self.get_logger().info("NavigateToPose goal succeeded")
        return result

    def publish_target_pose_marker(self, pose_stamped: PoseStamped):
        """Publish a simple marker (e.g., an arrow) in RViz to visualize the requested pose."""
        marker = Marker()
        marker.header = pose_stamped.header
        marker.header.frame_id = "world"  # Adjust frame if necessary
        marker.ns = "target_pose"
        marker.id = 0
        marker.type = Marker.ARROW
        marker.action = Marker.ADD

        # Pose (position/orientation)
        marker.pose = pose_stamped.pose

        # Scale and color
        marker.scale.x = 0.5  # Arrow length
        marker.scale.y = 0.1  # Arrow width
        marker.scale.z = 0.1
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0

        # Lifetime (0 = forever)
        marker.lifetime.sec = 0
        marker.lifetime.nanosec = 0

        self.marker_pub.publish(marker)


def main(args=None):
    rclpy.init(args=args)
    server_node = PositionController()
    executor = (
        MultiThreadedExecutor()
    )  # Allows processing multiple callbacks concurrently
    executor.add_node(server_node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        server_node.get_logger().info("Keyboard interrupt, shutting down...")

    finally:
        server_node.destroy_node()
        executor.shutdown()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
