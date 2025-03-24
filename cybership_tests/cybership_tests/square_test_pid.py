#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Wrench
from nav_msgs.msg import Odometry
import math
import numpy as np
from scipy.spatial.transform import Rotation as R

def wrap_to_pi(angle):
    """
    Wrap an angle in radians to the interval [-pi, pi].
    """
    return (angle + math.pi) % (2 * math.pi) - math.pi

class SquareMoveController(Node):
    def __init__(self):
        super().__init__('square_move_controller', namespace='voyager')

        # Publisher to send control commands (force and torque)
        self.control_pub = self.create_publisher(Wrench, 'control/force/command', 10)

        # Subscriber to receive odometry measurements
        self.create_subscription(Odometry, 'measurement/odom', self.odom_callback, 10)

        # Timer for periodic control update (dt = 0.01 s)
        self.dt = 0.01
        self.timer = self.create_timer(self.dt, self.control_loop)

        # Latest odometry message storage
        self.latest_odom = None

        # Square path parameters (in meters)
        self.side_length = 3.0
        self.corners = [
            (0.0, 0.0),
            (self.side_length, 0.0),
            (self.side_length, self.side_length),
            (0.0, self.side_length),
            (0.0, 0.0)
        ]

        for i in range(len(self.corners)):
            # Convert to numpy array for easier manipulation
            self.corners[i] = (self.corners[i][0] - 1.5, self.corners[i][1] - 1.5)

        self.corners = self.add_points(self.corners, num_points=15)

        self.target_index = 0
        self.current_target = self.corners[self.target_index]
        self.tol = 0.25  # Position tolerance in meters

        # Desired yaw: maintain heading of 0 radians (facing "north")
        self.desired_yaw = np.pi / 4

        # PID controller gains for position (x-y)
        self.Kp_pos = 3.0
        self.Ki_pos = 0.2
        self.Kd_pos = 0.6

        # PID controller gains for yaw
        self.Kp_yaw = 1.2
        self.Ki_yaw = 0.01
        self.Kd_yaw = 0.2

        # Integral and last-error states for x, y, yaw
        self.int_error_x = 0.0
        self.int_error_y = 0.0
        self.int_error_yaw = 0.0

        self.last_error_x = 0.0
        self.last_error_y = 0.0
        self.last_error_yaw = 0.0

        # Limit for integral windup
        self.max_integral = 0.1

        self.get_logger().info("Square Move PID Controller Initialized.")

    def add_points(self, corners, num_points=1):
        """
        Insert a specified number of points (num_points) between each pair of
        consecutive corners. The points will be equally spaced in a straight line.

        :param corners: A list of tuples (x, y) representing corners
        :param num_points: The number of extra points to insert between each pair
        :return: A new list of corners with the additional points inserted
        """
        new_corners = []

        for i in range(len(corners) - 1):
            x1, y1 = corners[i]
            x2, y2 = corners[i + 1]

            # Add the starting corner
            new_corners.append((x1, y1))

            # Insert 'num_points' equally spaced points between corners[i] and corners[i+1]
            for j in range(1, num_points + 1):
                t = j / (num_points + 1)
                x = x1 + t * (x2 - x1)
                y = y1 + t * (y2 - y1)
                new_corners.append((x, y))

        # Add the final corner
        new_corners.append(corners[-1])

        return new_corners

    def odom_callback(self, msg: Odometry):
        """
        Callback to update the latest odometry measurement.
        """
        self.latest_odom = msg

    def control_loop(self):
        """
        Control loop that computes and publishes the control command using PID.
        """
        if self.latest_odom is None:
            return

        # Extract current position
        pos = self.latest_odom.pose.pose.position
        current_x = pos.x
        current_y = pos.y

        # Extract current yaw
        orientation = self.latest_odom.pose.pose.orientation
        rot = R.from_quat([orientation.x, orientation.y, orientation.z, orientation.w])
        _, _, current_yaw = rot.as_euler('xyz', degrees=False)

        # Compute position error relative to the current target
        error_x = self.current_target[0] - current_x
        error_y = self.current_target[1] - current_y
        error_norm = math.sqrt(error_x**2 + error_y**2)

        # Check if the target is reached and update to next corner
        if error_norm < self.tol:
            self.target_index += 1
            if self.target_index >= len(self.corners):
                self.get_logger().info("Square path completed. Restarting square.")
                # Restart or stop; here we choose to restart at corner 1
                self.target_index = 1
            self.current_target = self.corners[self.target_index]
            self.get_logger().info(f"New target: {self.current_target}")

        # -------- PID for X --------
        # Proportional
        p_x = self.Kp_pos * error_x
        # Integral
        self.int_error_x += error_x * self.dt
        # Anti-windup for integral
        self.int_error_x = max(min(self.int_error_x, self.max_integral), -self.max_integral)
        i_x = self.Ki_pos * self.int_error_x
        # Derivative
        d_x = self.Kd_pos * ((error_x - self.last_error_x) / self.dt)

        # Combine for total output (force in X)
        tau_x = p_x + i_x + d_x

        # -------- PID for Y --------
        p_y = self.Kp_pos * error_y
        self.int_error_y += error_y * self.dt
        self.int_error_y = max(min(self.int_error_y, self.max_integral), -self.max_integral)
        i_y = self.Ki_pos * self.int_error_y
        d_y = self.Kd_pos * ((error_y - self.last_error_y) / self.dt)
        tau_y = p_y + i_y + d_y

        # -------- PID for Yaw --------
        yaw_error = wrap_to_pi(self.desired_yaw - current_yaw)
        p_yaw = self.Kp_yaw * yaw_error
        self.int_error_yaw += yaw_error * self.dt
        self.int_error_yaw = max(min(self.int_error_yaw, self.max_integral), -self.max_integral)
        i_yaw = self.Ki_yaw * self.int_error_yaw
        d_yaw = self.Kd_yaw * ((yaw_error - self.last_error_yaw) / self.dt)
        tau_yaw = p_yaw + i_yaw + d_yaw

        # Store current errors for next derivative calculation
        self.last_error_x = error_x
        self.last_error_y = error_y
        self.last_error_yaw = yaw_error

        # Optionally clamp the total output to avoid saturating the thrusters
        tau_x = np.clip(tau_x, -1.0, 1.0)
        tau_y = np.clip(tau_y, -1.0, 1.0)
        tau_yaw = np.clip(tau_yaw, -1.0, 1.0)

        # Create and publish the Wrench message
        wrench_msg = Wrench()
        wrench_msg.force.x = tau_x
        wrench_msg.force.y = tau_y
        wrench_msg.force.z = 0.0
        wrench_msg.torque.x = 0.0
        wrench_msg.torque.y = 0.0
        wrench_msg.torque.z = tau_yaw

        self.control_pub.publish(wrench_msg)

        # Debug logging
        self.get_logger().debug(
            f"Pos: ({current_x:.2f}, {current_y:.2f}), Target: {self.current_target}, "
            f"Err: ({error_x:.2f}, {error_y:.2f}), Yaw err: {yaw_error:.2f}, "
            f"Ctrl: fx={tau_x:.2f}, fy={tau_y:.2f}, tz={tau_yaw:.2f}"
        )

def main(args=None):
    rclpy.init(args=args)
    node = SquareMoveController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Square Move Controller stopped by user.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

