#!/usr/bin/env python3

# ----------------------------------------------------------------------------
# This code is part of the Cybership Software Suite
# Created By: Emir Cem Gezer
# Created Date: 2025-03-26
# Copyright (C) 2025: NTNU, Trondheim
# Licensed under GPL-3.0-or-later
# ---------------------------------------------------------------------------

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from geometry_msgs.msg import Twist, Wrench
from nav_msgs.msg import Odometry
import shoeboxpy.model3dof as box

from cybership_controller.velocity.reference_feedforward import RffVelocityController as VelocityController

# Add import for performance metrics
try:
    from cybership_interfaces.msg import PerformanceMetrics
except ImportError:
    print("cybership_msgs not found. Skipping performance metrics.")



class VelocityControlNode(Node):
    """
    ROS Node for velocity control using a reference filter.
    Subscribes to cmd_vel for velocity commands and publishes control forces/moments.
    """

    def __init__(self):
        super().__init__("velocity_control_node", namespace="voyager")

        # Declare parameters with default values
        self.declare_parameters(
            namespace='',
            parameters=[
                # Vessel dimensions
                ('vessel.length', 1.0),
                ('vessel.beam', 0.3),
                ('vessel.draft', 0.05),

                # Control gains
                ('control.p_gain.surge', 5.0),
                ('control.p_gain.sway', 5.0),
                ('control.p_gain.yaw', 5.0),

                ('control.i_gain.surge', 0.0),
                ('control.i_gain.sway', 0.0),
                ('control.i_gain.yaw', 0.0),

                ('control.d_gain.surge', 0.3),
                ('control.d_gain.sway', 0.3),
                ('control.d_gain.yaw', 0.3),

                ('control.i_max', 20.0),
                ('control.smooth_limit', True),
                ('control.filter_alpha', 0.1),

                # Performance metrics
                ('metrics.window_size', 50),
                ('metrics.interval', 1.0),

                # Time step
                ('dt', 0.1),
            ]
        )

        # Add parameter callback for runtime updates
        self.add_on_set_parameters_callback(self.parameters_callback)

        # Get current parameter values
        self.dt = self.get_parameter('dt').value

        # Initialize vessel and controller
        self.update_configuration()

        self.nu_prev = np.zeros(3)  # [u, v, r] previous velocities
        self.nu_cmd = np.zeros(3)  # [u, v, r] desired velocities
        self.nu_cmd_prev = np.zeros(3)  # [u, v, r] desired velocities

        # Current velocity state
        self.nu = np.zeros(3)  # [u, v, r]

        # Subscribe to cmd_vel (velocity commands)
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            'control/velocity/command',
            self.cmd_vel_callback,
            10
        )

        # Subscribe to odometry for current velocity
        self.odom_sub = self.create_subscription(
            Odometry,
            'measurement/odom',
            self.odom_callback,
            1
        )

        # Publisher for control commands
        self.control_pub = self.create_publisher(
            Wrench,
            'control/force/command/velocity',
            10
        )

        # Add debug publishers for tracking performance
        self.debug_vel_cmd_pub = self.create_publisher(
            Twist,
            'control/velocity/debug/reference_velocity',
            10
        )
        self.debug_error_vel_pub = self.create_publisher(
            Twist,
            'control/velocity/debug/tracking_error_velocity',
            10
        )

        # Performance metrics publisher
        self.debug_metrics_pub = None
        if 'PerformanceMetrics' in globals():
            self.debug_metrics_pub = self.create_publisher(
                PerformanceMetrics,
                "control/velocity/debug/performance_metrics",
                10
            )

        # --- Performance metrics ---
        # Performance metrics tracking with moving window
        self.window_size = self.get_parameter('metrics.window_size').value
        self.error_window = []  # Store recent velocity errors
        self.start_time = None
        self.sample_count = 0

        # Time between metrics calculations
        self.metrics_interval = self.get_parameter('metrics.interval').value
        self.last_metrics_time = 0.0

        # Timer for control loop
        self.timer = self.create_timer(self.dt, self.control_loop)

        self.get_logger().info("Velocity controller initialized with parameters")

    def update_configuration(self):
        """Update vessel and controller configuration from parameters"""
        # Get vessel dimensions
        vessel_length = self.get_parameter('vessel.length').value
        vessel_beam = self.get_parameter('vessel.beam').value
        vessel_draft = self.get_parameter('vessel.draft').value

        # Create vessel model
        self.vessel = box.Shoebox(L=vessel_length, B=vessel_beam, T=vessel_draft)

        # Get control gains
        self.k_p_gain = np.array([
            self.get_parameter('control.p_gain.surge').value,
            self.get_parameter('control.p_gain.sway').value,
            self.get_parameter('control.p_gain.yaw').value
        ])

        self.k_i_gain = np.array([
            self.get_parameter('control.i_gain.surge').value,
            self.get_parameter('control.i_gain.sway').value,
            self.get_parameter('control.i_gain.yaw').value
        ])

        self.k_d_gain = np.array([
            self.get_parameter('control.d_gain.surge').value,
            self.get_parameter('control.d_gain.sway').value,
            self.get_parameter('control.d_gain.yaw').value
        ])

        # Create/update controller
        self.controller = VelocityController(
            config={
                "M": self.vessel.M_eff,
                "D": self.vessel.D,
                "Kp": np.diag(self.k_p_gain),
                "Ki": np.diag(self.k_i_gain),
                "Kd": np.diag(self.k_d_gain),
                "I_max": self.get_parameter('control.i_max').value,
                "smooth_limit": self.get_parameter('control.smooth_limit').value,
                "filter_alpha": self.get_parameter('control.filter_alpha').value,
                "dt": self.dt,
            }
        )

        # Update metrics parameters
        self.window_size = self.get_parameter('metrics.window_size').value
        self.metrics_interval = self.get_parameter('metrics.interval').value

        self.get_logger().info(f"Updated configuration - vessel: [{vessel_length}, {vessel_beam}, {vessel_draft}], " +
                              f"P: {self.k_p_gain}, I: {self.k_i_gain}, D: {self.k_d_gain}")

    def parameters_callback(self, params):
        """Handle parameter updates"""
        update_needed = False

        for param in params:
            if param.name in [
                'vessel.length', 'vessel.beam', 'vessel.draft',
                'control.p_gain.surge', 'control.p_gain.sway', 'control.p_gain.yaw',
                'control.i_gain.surge', 'control.i_gain.sway', 'control.i_gain.yaw',
                'control.d_gain.surge', 'control.d_gain.sway', 'control.d_gain.yaw',
                'control.i_max', 'control.smooth_limit', 'control.filter_alpha',
                'metrics.window_size', 'metrics.interval'
            ]:
                update_needed = True

            if param.name == 'dt':
                self.dt = param.value
                # Recreate the timer with new dt
                self.timer.cancel()
                self.timer = self.create_timer(self.dt, self.control_loop)
                update_needed = True

        # Update vessel and controller if parameters changed
        if update_needed:
            self.update_configuration()

        return True  # Accept all parameter changes

    def cmd_vel_callback(self, msg: Twist):
        """
        Process incoming velocity commands.

        Parameters:
        -----------
        msg : Twist
            Desired velocity command
        """
        # Extract the desired velocities from the Twist message
        # Linear velocities: x (surge), y (sway)
        # Angular velocity: z (yaw rate)
        self.nu_cmd = np.array([
            msg.linear.x,  # surge velocity (u)
            msg.linear.y,  # sway velocity (v)
            msg.angular.z  # yaw rate (r)
        ])

        self.get_logger().debug(
            f"Received cmd_vel: [{self.nu_cmd[0]:.2f}, {self.nu_cmd[1]:.2f}, {self.nu_cmd[2]:.2f}]")

    def odom_callback(self, msg: Odometry):
        """
        Update current velocity from odometry.

        Parameters:
        -----------
        msg : Odometry
            Odometry message containing current state
        """
        # Extract body-fixed velocities
        self.nu = np.array([
            msg.twist.twist.linear.x,   # surge velocity (u)
            msg.twist.twist.linear.y,   # sway velocity (v)
            msg.twist.twist.angular.z   # yaw rate (r)
        ])

    def process_performance_metrics(self, desired_vel, error_vel):
        """
        Process and publish debug information using a moving window approach.
        """
        # Publish command velocity for debugging
        msg = Twist()
        msg.linear.x = desired_vel[0]
        msg.linear.y = desired_vel[1]
        msg.angular.z = desired_vel[2]

        self.debug_vel_cmd_pub.publish(msg)

        # Publish velocity error for debugging
        error_twist = Twist()
        error_twist.linear.x = error_vel[0]
        error_twist.linear.y = error_vel[1]
        error_twist.linear.z = 0.0
        error_twist.angular.x = 0.0
        error_twist.angular.y = 0.0
        error_twist.angular.z = error_vel[2]
        self.debug_error_vel_pub.publish(error_twist)

        # Update metrics using moving window
        error_norm = np.sqrt(
            error_vel[0]**2 + error_vel[1]**2 + error_vel[2]**2)

        # Initialize start time if not already set
        if self.start_time is None:
            self.start_time = self.get_clock().now().nanoseconds / 1e9

        # Add newest error to window
        self.error_window.append(error_norm)

        # Keep window at fixed size
        if len(self.error_window) > self.window_size:
            self.error_window.pop(0)

        self.sample_count += 1

        current_time = self.get_clock().now().nanoseconds / 1e9
        if current_time - self.last_metrics_time >= self.metrics_interval and len(self.error_window) > 0:
            self.last_metrics_time = current_time

            # Calculate window statistics
            error_array = np.array(self.error_window)

            if self.debug_metrics_pub is not None:
                metrics_msg = PerformanceMetrics()
                metrics_msg.header.stamp = self.get_clock().now().to_msg()
                metrics_msg.header.frame_id = "voyager"
                metrics_msg.message = "Velocity tracking error"

                # Calculate statistics over the window
                metrics_msg.mean = np.mean(error_array)
                metrics_msg.median = np.median(error_array)
                metrics_msg.rms = np.sqrt(np.mean(np.square(error_array)))
                metrics_msg.min = np.min(error_array)
                metrics_msg.max = np.max(error_array)
                metrics_msg.stddev = np.std(error_array)

                self.debug_metrics_pub.publish(metrics_msg)

                self.get_logger().info(
                    f"Velocity tracking metrics - Mean: {metrics_msg.mean:.3f}, "
                    f"RMS: {metrics_msg.rms:.3f}, Max: {metrics_msg.max:.3f}"
                )

    def control_loop(self):
        """
        Periodic control loop to update reference filter and publish control commands.
        """
        tau = self.controller.update(
            current_velocity=self.nu,
            desired_velocity=self.nu_cmd,
            dt=self.dt
        )

        # Calculate velocity error for metrics
        error_vel = self.nu_cmd - self.nu

        # Process and publish performance metrics
        self.process_performance_metrics(self.nu_cmd, error_vel)

        # Create and publish Wrench message
        wrench_msg = Wrench()
        wrench_msg.force.x = float(tau[0])
        wrench_msg.force.y = float(tau[1])
        wrench_msg.force.z = 0.0
        wrench_msg.torque.x = 0.0
        wrench_msg.torque.y = 0.0
        wrench_msg.torque.z = float(tau[2])

        self.control_pub.publish(wrench_msg)


def main(args=None):
    """
    Main function to initialize and run the velocity control node with a multithreaded executor.
    """
    rclpy.init(args=args)
    node = VelocityControlNode()

    # Create a multithreaded executor
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)

    try:
        # Spin the executor instead of the node directly
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard interrupt, shutting down...")
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
