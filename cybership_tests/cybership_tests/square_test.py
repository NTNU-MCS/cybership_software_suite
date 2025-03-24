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
        self.side_length = 2.0
        # Define corners: starting at (0,0), then (10,0), (10,10), (0,10), and back to (0,0)
        self.corners = [(0.0, 0.0),
                        (self.side_length, 0.0),
                        (self.side_length, self.side_length),
                        (0.0, self.side_length),
                        (0.0, 0.0)]

        for i in range(len(self.corners)):
            # Convert to numpy array for easier manipulation
            self.corners[i] = (self.corners[i][0] - 1, self.corners[i][1] - 1)


        # Start with the first target (moving from (0,0) to (10,0))
        self.target_index = 1
        self.current_target = self.corners[self.target_index]
        self.tol = 0.2 # Position tolerance in meters

        # Controller gains (tune as needed)
        self.Kp_pos = 1.2 # Gain for position control (x-y)
        self.Kp_yaw = 0.8  # Gain for yaw control

        # Desired yaw: maintain north (0 radians)
        self.desired_yaw = 0.0 # np.pi / 4.0

        self.get_logger().info("Square Move Controller Initialized.")

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

        # Extract yaw (heading) from the quaternion orientation
        orientation = self.latest_odom.pose.pose.orientation
        rot = R.from_quat([orientation.x, orientation.y, orientation.z, orientation.w])
        _,_, current_yaw = rot.as_euler('xyz', degrees=False)

        # _, _, current_yaw = quaternion_to_euler(orientation)

        # Compute position error relative to the current target
        error_x = self.current_target[0] - current_x
        error_y = self.current_target[1] - current_y
        error_norm = math.sqrt(error_x**2 + error_y**2)

        # Check if the target is reached and update to next target if needed
        if error_norm < self.tol:
            self.target_index += 1
            if self.target_index >= len(self.corners):
                self.get_logger().info("Square path completed. Restarting square.")
                # Optionally, restart the square or stop control here.
                self.target_index = 1  # restart from the second corner
            self.current_target = self.corners[self.target_index]
            self.get_logger().info(f"New target: {self.current_target}")

        # Compute control forces for position (global frame)
        tau_x = np.clip(self.Kp_pos * error_x, -2.0, 2.0)
        tau_y = np.clip(self.Kp_pos * error_y, -2.0, 2.0)



        # Compute yaw control to maintain desired yaw (north)
        yaw_error = wrap_to_pi(self.desired_yaw - current_yaw)
        tau_yaw = np.clip(self.Kp_yaw * yaw_error, -1.0, 1.0)

        # Create and populate the Wrench message (only surge, sway, and yaw are controlled)
        wrench_msg = Wrench()
        wrench_msg.force.x = tau_x
        wrench_msg.force.y = tau_y
        wrench_msg.force.z = 0.0  # No vertical force
        wrench_msg.torque.x = 0.0
        wrench_msg.torque.y = 0.0
        wrench_msg.torque.z = tau_yaw

        # Publish the control command
        self.control_pub.publish(wrench_msg)

        # Debug logging (can be set to info if preferred)
        self.get_logger().debug(
            f"Pos: ({current_x:.2f}, {current_y:.2f}), Target: {self.current_target}, "
            f"Error: ({error_x:.2f}, {error_y:.2f}), Yaw error: {yaw_error:.2f}"
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
