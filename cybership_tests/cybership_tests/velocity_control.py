#!/usr/bin/env python3

# ----------------------------------------------------------------------------
# This code is part of the Cybership Software Suite
# Created By: Emir Cem Gezer
# Created Date: 2025-03-26
# Copyright (C) 2025: NTNU, Trondheim
# Licensed under GPL-3.0-or-later
# ---------------------------------------------------------------------------

import numpy as np
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Wrench, PoseStamped, Pose2D
from nav_msgs.msg import Odometry
from typing import Tuple
import shoeboxpy.model3dof as box
from visualization_msgs.msg import Marker
from scipy.spatial.transform import Rotation as R
import numpy as np
from typing import Dict, Any, Optional


# Add import for performance metrics
try:
    from cybership_interfaces.msg import PerformanceMetrics
except ImportError:
    print("cybership_msgs not found. Skipping performance metrics.")

import numpy as np

class ExponentialSmoothing():
    """
    Exponential smoothing class
    """

    def __init__(self, r: float = 0.7) -> None:
        """
        Exponential smoothing class constructor

        :param r: A weighting factor in the set [0-1]
        """
        self.r = r
        self.x = None
        self.x_p = None
        self.dx_p = None

    def __call__(self, x: np.ndarray) -> np.ndarray:
        """
        Exponential smoothing function

        :param x: current value of x
        :return: dx
        """
        if self.x_p is None or self.dx_p is None:
            self.x_p = x
            self.dx_p = x
            return x
        else:
            self.dx_p = ExponentialSmoothing.__func(self.r, x, self.x_p, self.dx_p)
            self.x_p = x
            return self.dx_p

    def reset(self, x: np.ndarray) -> None:
        """
        Reset the exponential smoothing

        :param x: current value of x
        :return:
        """
        self.x_p = x
        self.dx_p = x

    @staticmethod
    def __func(
            r: float, x: np.ndarray, x_p: np.ndarray = None, dx_p: np.ndarray = None
    ) -> np.ndarray:
        r"""
        Exponential smoothing function

        .. math::
            \begin{aligned}
                s_{0}&=x_{0}\\
                s_{t}&=\alpha x_{t}+(1-\alpha )s_{t-1},\quad t>0
            \end{aligned}


        :param r: A weighting factor in the set [0-1]
        :param x: current value of x
        :param x_p: previous value of x
        :param dx_p: last computed filtered x value
        :return:
        """
        if x_p is None or dx_p is None:
            return x
        else:
            return (1 - r) * dx_p + r * (x - x_p)

def _saturate(vec: np.ndarray, limit: float) -> np.ndarray:
    """Scale `vec` so its l2-norm does not exceed `limit`. If `limit` <= 0, return vec unchanged."""
    nrm = np.linalg.norm(vec)
    return vec if nrm <= limit or limit <= 0.0 else vec * (limit / nrm)

# ---------------------------------------------------------------------- #
#  Controller implementation                                             #
# ---------------------------------------------------------------------- #

class AccelLimitedInverseDynamicsPI():
    """Inverse-dynamics velocity controller with PI action and acceleration limiting."""

    def __init__(self, config: Optional[Dict[str, Any]] = None):

        self.config = config or {}
        # Matrices and gains
        self.M: np.ndarray = np.array(self.config.get("M", np.eye(3)), dtype=float)
        self.D: np.ndarray = np.array(self.config.get("D", np.zeros((3, 3))), dtype=float)
        self.Kp: np.ndarray = np.array(self.config.get("Kp", np.eye(3)), dtype=float)
        self.Ki: np.ndarray = np.array(self.config.get("Ki", np.zeros((3, 3))), dtype=float)
        self.Kd: np.ndarray = np.array(self.config.get("Kd", np.zeros((3, 3))), dtype=float)
        self.dt: float = float(self.config.get("dt", 0.01))  # seconds

        # Pre-compute inverse inertia
        self.M_inv = np.linalg.inv(self.M)

        # Limits and behaviours
        self.a_max: float = float(self.config.get("a_max", -1.0))       # hard acc limit (<=0 => off)
        self.I_max: float = float(self.config.get("I_max", -1.0))       # integral wind-up cap (<=0 => off)
        self.smooth: bool = bool(self.config.get("smooth_limit", False))

        # Internal state
        self._prev_vd: Optional[np.ndarray] = None  # for desired-velocity derivative
        self._int_e: np.ndarray = np.zeros(3)       # integral of error dt

    # ------------------------------------------------------------------ #
    #  Public API                                                        #
    # ------------------------------------------------------------------ #
    def update(
        self,
        current_velocity: np.ndarray,
        desired_velocity: np.ndarray,
        dt: float,
    ) -> np.ndarray:
        """Compute the force/torque command tau."""

        if dt <= 0.0:
            raise ValueError("dt must be positive")

        v  = current_velocity.reshape(3)
        vd = desired_velocity.reshape(3)

        if not hasattr(self, '_vd_dot_smoother'):
            r = self.config.get("filter_alpha", 0.7)
            self._vd_dot_smoother = ExponentialSmoothing(r=r)

        if not hasattr(self, '_e_dot_smoother'):
            r = self.config.get("filter_alpha", 0.7)
            self._e_dot_smoother = ExponentialSmoothing(r=r)

        raw_vd_dot = (vd - self._prev_vd) / dt if self._prev_vd is not None else np.zeros(3)
        vd_dot = self._vd_dot_smoother(raw_vd_dot)
        self._prev_vd = vd.copy()

        e = v - vd
        raw_e_dot = (v - self._prev_vd) / dt if self._prev_vd is not None else np.zeros(3)
        e_dot = self._e_dot_smoother(raw_e_dot)

        self._int_e = self._int_e + e * dt
        self._int_e = np.clip(self._int_e, -self.I_max, self.I_max)

        a_des = vd_dot - self.Kp @ e - self.Ki @ self._int_e - self.Kd @ e_dot

        tau = self.M @ a_des + self.D @ vd
        return tau

    def reset(self):
        """Clear stored derivative and integral information."""
        self._prev_vd = None
        self._int_e = np.zeros(3)

    # ------------------------------------------------------------------ #
    #  Private helpers                                                   #
    # ------------------------------------------------------------------ #
    def _limit_acceleration(self, a_des: np.ndarray) -> tuple[np.ndarray, bool]:
        """Return (a_cmd, saturated_flag) after applying the configured limiter."""
        if self.a_max <= 0.0:
            return a_des, False  # unlimited

        if self.smooth:
            # Smooth limiter using tanh
            a_cmd = self.a_max * np.tanh(a_des / self.a_max)
            saturated = not np.allclose(a_cmd, a_des)
            return a_cmd, saturated
        else:
            a_cmd = _saturate(a_des, self.a_max)
            saturated = not np.allclose(a_cmd, a_des)
            return a_cmd, saturated

class VelocityControlNode(Node):
    """
    ROS Node for velocity control using a reference filter.
    Subscribes to cmd_vel for velocity commands and publishes control forces/moments.
    """

    def __init__(self):
        super().__init__("velocity_control_node", namespace="voyager")

        # Create a velocity controller
        self.dt = 0.05  # seconds
        self.vessel = box.Shoebox(L=1.0, B=0.3, T=0.05)

        # # Control gains for surge, sway, and yaw
        self.k_p_gain = np.array([5.0, 5.0, 5.0])
        self.k_i_gain = np.array([0.0, 0.0, 0.0])
        self.k_d_gain = np.array([0.3, 0.3, 0.3])

        self.controller = AccelLimitedInverseDynamicsPI(
            config={
                "M": self.vessel.M_eff,
                "D": self.vessel.D,
                "Kp": np.diag(self.k_p_gain),
                "Ki": np.diag(self.k_i_gain),
                "Kd": np.diag(self.k_d_gain),
                "I_max": 20.0,  # Nms
                "smooth_limit": True,
                "filter_alpha": 0.1,  # Smoothing factor for desired velocity
                "dt": self.dt,
            }
        )

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
            10
        )

        # Publisher for control commands
        self.control_pub = self.create_publisher(
            Wrench,
            'control/force/mux/velocity',
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
        self.window_size = 200  # 2 seconds of data at 100Hz
        self.error_window = []  # Store recent velocity errors
        self.start_time = None
        self.sample_count = 0

        # Time between metrics calculations
        self.metrics_interval = 1.0  # seconds
        self.last_metrics_time = 0.0

        # Timer for control loop
        self.timer = self.create_timer(self.dt, self.control_loop)

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
        # Update the reference filter

        now   = self.get_clock().now().nanoseconds * 1e-9
        dt_rt = now - self._prev_t if hasattr(self, '_prev_t') else self.dt
        tau = self.controller.update(
            current_velocity=self.nu,
            desired_velocity=self.nu_cmd,
            dt=dt_rt
        )
        self._prev_t = now

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
