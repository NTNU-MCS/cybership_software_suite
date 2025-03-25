#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Wrench, Pose2D, PoseStamped
from nav_msgs.msg import Odometry
import math
import numpy as np
from scipy.spatial.transform import Rotation as R


def wrap_to_pi(angle):
    """
    Wrap an angle in radians to the interval [-pi, pi].
    """
    return (angle + math.pi) % (2 * math.pi) - math.pi


def saturate(x, z):
    """
    Saturation function: returns x / (|x| + z)
    This ensures a smooth saturation between -1 and 1.
    """
    return x / (np.abs(x) + z)


class GotoPointController(Node):
    def __init__(self):
        super().__init__("goto_point_controller", namespace="voyager")

        # Publisher to send control commands (force and torque)
        self.control_pub = self.create_publisher(Wrench, "control/force/command", 10)

        # Subscriber to receive odometry measurements
        self.create_subscription(
            Pose2D, "guidance/position/command", self.pose_callback, 10
        )

        self.create_subscription(PoseStamped, "/goal_pose", self.goal_pose_callback, 10)

        self.create_subscription(Odometry, "measurement/odom", self.odom_callback, 10)

        # Timer for periodic control updates
        self.dt = 0.01  # seconds
        self.timer = self.create_timer(self.dt, self.control_loop)

        # Latest odometry message storage
        self.latest_odom = None

        # --- Target state ---
        # Set desired target position and orientation here.
        self.target_x = -1.5  # target x position (meters)
        self.target_y = -1.5  # target y position (meters)
        self.target_yaw = 0.0  # target yaw (radians)

        self.last_error_x = 0.0
        self.last_error_y = 0.0
        self.last_error_yaw = 0.0

        # --- Controller gains ---
        self.Kp_pos = 1.0  # proportional gain for position
        self.Kd_pos = 0.0  # derivative gain for position
        self.Ki_pos = 0.01  # integral gain for position

        self.Kp_yaw = 1.0  # proportional gain for yaw
        self.Kd_yaw = 0.4  # derivative gain for yaw
        self.Ki_yaw = 0.0  # integral gain for yaw

        # Saturation parameter for the controller (tune as needed)
        self.sat_z = 0.5

        self.saturation_x = 0.5
        self.saturation_y = 0.5
        self.saturation_yaw = 0.5

        # Integral error initialization
        self.integral_x = 0.0
        self.integral_y = 0.0
        self.integral_yaw = 0.0

        # Tolerances for considering the target reached
        self.pos_tol = 0.2  # meters
        self.yaw_tol = 0.05  # radians

        self.get_logger().info("Goto Point Controller Initialized.")

    def odom_callback(self, msg: Odometry):
        """
        Callback to update the latest odometry measurement.
        """
        self.latest_odom = msg

    def control_loop(self):
        """
        Control loop that computes and publishes the control command.
        """
        if self.latest_odom is None:
            return

        # Extract current position from odometry (global frame)
        pos = self.latest_odom.pose.pose.position
        current_x = pos.x
        current_y = pos.y

        # Extract yaw (heading) from quaternion orientation
        orientation = self.latest_odom.pose.pose.orientation
        rot = R.from_quat([orientation.x, orientation.y, orientation.z, orientation.w])
        _, _, current_yaw = rot.as_euler("xyz", degrees=False)

        # Compute position errors (global frame)
        error_x = self.target_x - current_x
        error_y = self.target_y - current_y
        error_norm = math.sqrt(error_x**2 + error_y**2)

        if error_norm > 0.5:
            self.integral_x = 0.0
            self.integral_y = 0.0
            self.integral_yaw = 0.0

        # Compute yaw error (wrap to [-pi, pi])
        error_yaw = wrap_to_pi(self.target_yaw - current_yaw)

        # Update integral error terms
        self.integral_x += error_x * self.dt
        self.integral_y += error_y * self.dt
        self.integral_yaw += error_yaw * self.dt

        # Compute control commands with integral action using the saturation function
        # Control law: u = saturate(Kp*error + Ki*integral, sat_z)
        control_x = saturate(
            self.Kp_pos * error_x
            + self.Ki_pos * self.integral_x
            + self.Kd_pos * ((error_x - self.last_error_x) / self.dt),
            self.saturation_x,
        )
        control_y = saturate(
            self.Kp_pos * error_y
            + self.Ki_pos * self.integral_y
            + self.Kd_pos * ((error_y - self.last_error_y) / self.dt),
            self.saturation_y,
        )
        control_yaw = saturate(
            self.Kp_yaw * error_yaw
            + self.Kd_yaw * ((error_yaw - self.last_error_yaw) / self.dt)
            + self.Ki_yaw * self.integral_yaw,
            self.saturation_yaw,
        )

        self.last_error_x = error_x
        self.last_error_y = error_y
        self.last_error_yaw = wrap_to_pi(self.target_yaw - current_yaw)

        # Zero control output if within tolerance (optional)
        if error_norm < self.pos_tol:
            control_x = 0.0
            control_y = 0.0
        if abs(error_yaw) < self.yaw_tol:
            control_yaw = 0.0

        # Create and populate the Wrench message
        wrench_msg = Wrench()
        wrench_msg.force.x = control_x
        wrench_msg.force.y = control_y
        wrench_msg.force.z = 0.0  # No vertical force
        wrench_msg.torque.x = 0.0
        wrench_msg.torque.y = 0.0
        wrench_msg.torque.z = control_yaw

        # Publish the control command
        self.control_pub.publish(wrench_msg)

        # Debug logging (change to info if needed)
        self.get_logger().debug(
            f"Current pos: ({current_x:.2f}, {current_y:.2f}), Target: ({self.target_x:.2f}, {self.target_y:.2f}), "
            f"Errors: ({error_x:.2f}, {error_y:.2f}), Yaw error: {error_yaw:.2f}"
        )

    def pose_callback(self, msg: Pose2D):
        """
        Callback to update the target position and yaw.
        """
        self.target_x = msg.x
        self.target_y = msg.y
        self.target_yaw = msg.theta

    def goal_pose_callback(self, msg: PoseStamped):
        """
        Callback to update the target position and yaw from a PoseStamped message.

        This is useful for receiving target poses from RViZ.
        """
        self.target_x = msg.pose.position.x
        self.target_y = msg.pose.position.y

        # Convert quaternion to yaw angle
        orientation = msg.pose.orientation
        rot = R.from_quat([orientation.x, orientation.y, orientation.z, orientation.w])
        _, _, self.target_yaw = rot.as_euler("xyz", degrees=False)

        self.get_logger().info(
            f"New target pose received: ({self.target_x}, {self.target_y}, {self.target_yaw})"
        )


def main(args=None):
    rclpy.init(args=args)
    node = GotoPointController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Goto Point Controller stopped by user.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
