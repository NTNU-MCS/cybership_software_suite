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
        super().__init__("square_move_controller", namespace="voyager")

        # Publisher to send control commands (force and torque)
        self.control_pub = self.create_publisher(Wrench, "control/force/command", 10)

        # Subscriber to receive odometry measurements
        self.create_subscription(Odometry, "measurement/odom", self.odom_callback, 10)

        # Timer for periodic control update (dt = 0.01 s)
        self.dt = 0.01
        self.timer = self.create_timer(self.dt, self.control_loop)

        # Latest odometry message storage
        self.latest_odom = None

        # Square path parameters (in meters)
        # (Example: a 3×3 square centered at (0,0).)
        self.side_length = 3.0
        self.corners = [
            (0.0, 0.0),
            (self.side_length, 0.0),
            (self.side_length, self.side_length),
            (0.0, self.side_length),
            (0.0, 0.0),  # Close the loop
        ]
        # Shift corners so the square is centered at (-1.5, -1.5):
        for i in range(len(self.corners)):
            self.corners[i] = (self.corners[i][0] - 1.5, self.corners[i][1] - 1.5)

        # Desired headings at each corner (radians):
        self.corner_headings = [
            0.0,  # corner 0
            0.0,  # corner 1
            math.pi / 4,  # corner 2
            0.0,  # corner 3
            0.0,
        ]

        # ---------------------------------------
        # Controller gains (PID-like)
        # ---------------------------------------
        # Proportional gains
        self.kp_xy = 1.0  # cross-track & along-track
        self.kp_yaw = 0.6  # heading
        self.kp_speed = 0.6  # forward speed

        # Integral gains
        self.ki_xy = 0.1  # cross-track integrator
        self.ki_yaw = 0.0  # heading integrator (set to 0 if not desired)
        self.ki_speed = 0.0  # forward speed integrator (set to 0 if not desired)

        # Desired forward speed along the path
        self.desired_speed = 1.0  # m/s

        # Index to track current segment start corner
        self.current_waypoint_index = 0

        # Threshold to decide when we have "arrived" near the next corner
        self.segment_arrival_threshold = 0.25

        # ---------------------------------------
        # Integral error states
        # ---------------------------------------
        # We'll track x- and y-components of cross-track error
        self.cross_x_int = 0.2
        self.cross_y_int = 0.2

        # Integral of yaw error
        self.yaw_err_int = 0.0

        # Integral of speed error
        self.speed_err_int = 0.0

        # Maximum allowed velocities (in m/s) for the vessel
        self.max_surge_velocity = 0.05
        self.max_sway_velocity = 0.1

        self.get_logger().info(
            "Square Move Controller with cross-track integral initialized."
        )

    def reset_integrators(self):
        """
        Resets integral states to zero.
        Typically called when you switch to a new segment.
        """
        self.cross_x_int = 0.0
        self.cross_y_int = 0.0
        self.yaw_err_int = 0.0

    def odom_callback(self, msg: Odometry):
        """
        Callback to update the latest odometry measurement.
        """
        self.latest_odom = msg

    def control_loop(self):
        """
        Control loop that computes and publishes the control command.
        Now includes integral control for cross-track, heading, and (optionally) speed.
        """
        if self.latest_odom is None:
            return

            # Check if the loop is complete
        if self.current_waypoint_index == len(self.corners) - 1:
            self.get_logger().info("Completed square path.")
            # Stop the control loop
            self.destroy_timer(self.timer)
            return

        # Extract current position
        pos = self.latest_odom.pose.pose.position
        current_x = pos.x
        current_y = pos.y

        # Extract current yaw
        orientation = self.latest_odom.pose.pose.orientation
        rot = R.from_quat([orientation.x, orientation.y, orientation.z, orientation.w])
        _, _, current_yaw = rot.as_euler("xyz", degrees=False)

        # Determine current segment from corners[i] to corners[i+1]
        p1_index = self.current_waypoint_index
        p2_index = (p1_index + 1) % len(self.corners)
        p1 = self.corners[p1_index]
        p2 = self.corners[p2_index]

        # Segment vector
        dx = p2[0] - p1[0]
        dy = p2[1] - p1[1]
        segment_length = math.sqrt(dx * dx + dy * dy)

        # Handle degenerate case
        if segment_length < 1e-6:
            # Move to next segment
            self.current_waypoint_index = p2_index
            self.reset_integrators()
            return

        # Normalized direction along the segment
        ux = dx / segment_length
        uy = dy / segment_length

        # Vector from p1 to current position
        ex = current_x - p1[0]
        ey = current_y - p1[1]

        # Along-track distance via dot product
        along_track_dist = ex * ux + ey * uy

        # Check if we're close enough to the end of the segment
        if along_track_dist >= segment_length - self.segment_arrival_threshold:
            self.current_waypoint_index = p2_index
            self.reset_integrators()
            return

        # Clamp if below 0
        if along_track_dist < 0.0:
            along_track_dist = 0.0

        # Nearest point on the segment
        nearest_x = p1[0] + along_track_dist * ux
        nearest_y = p1[1] + along_track_dist * uy

        # Cross-track error (vector from nearest point to current position)
        cross_x = current_x - nearest_x
        cross_y = current_y - nearest_y

        # ---------------------------------------
        # Update integrators
        # ---------------------------------------
        # Integrate cross-track error in x and y
        self.cross_x_int += cross_x * self.dt
        self.cross_y_int += cross_y * self.dt

        # Heading
        desired_yaw = self.corner_headings[p1_index]
        yaw_error = wrap_to_pi(desired_yaw - current_yaw)
        self.yaw_err_int += yaw_error * self.dt

        # Speed integrator
        # velocity in global frame
        vx = self.latest_odom.twist.twist.linear.x
        vy = self.latest_odom.twist.twist.linear.y
        # along-track velocity
        v_along = vx * ux + vy * uy
        speed_err = self.desired_speed - v_along
        self.speed_err_int += speed_err * self.dt

        # ---------------------------------------
        # Compute Control (P + I)
        # ---------------------------------------
        # Cross-track control: push back toward the path
        #   tau_ct = -(kp*error + ki*integral_of_error)
        tau_ct_x = -(self.kp_xy * cross_x + self.ki_xy * self.cross_x_int)
        tau_ct_y = -(self.kp_xy * cross_y + self.ki_xy * self.cross_y_int)
        # Along-track control for speed
        #   (kp*speed_err + ki*integral_of_speed_err) in direction of segment
        speed_control = self.kp_speed * speed_err + self.ki_speed * self.speed_err_int
        tau_along_x = speed_control * ux
        tau_along_y = speed_control * uy

        # Combine cross-track + along-track
        tau_x = tau_ct_x + tau_along_x
        tau_y = tau_ct_y + tau_along_y

        # Heading control
        #   tau_yaw = kp_yaw * yaw_err + ki_yaw * yaw_err_int
        tau_yaw = self.kp_yaw * yaw_error + self.ki_yaw * self.yaw_err_int

        # --- Velocity Limiting ---
        # Assume the odometry twist is expressed in the vessel's body frame.
        current_surge_velocity = self.latest_odom.twist.twist.linear.x
        current_sway_velocity = self.latest_odom.twist.twist.linear.y

        # Limit surge force
        if abs(current_surge_velocity) > self.max_surge_velocity:
            scaling_factor = self.max_surge_velocity / abs(current_surge_velocity)
            self.get_logger().debug(
                f"Surge velocity ({current_surge_velocity:.2f} m/s) exceeds max. Scaling force_x by {scaling_factor:.2f}"
            )
            tau_x *= scaling_factor

        # Limit sway force
        if abs(current_sway_velocity) > self.max_sway_velocity:
            scaling_factor = self.max_sway_velocity / abs(current_sway_velocity)
            self.get_logger().debug(
                f"Sway velocity ({current_sway_velocity:.2f} m/s) exceeds max. Scaling force_y by {scaling_factor:.2f}"
            )
            tau_y *= scaling_factor

        wrench_msg = Wrench()
        wrench_msg.force.x = float(tau_x)
        wrench_msg.force.y = float(tau_y)
        wrench_msg.force.z = 0.0
        wrench_msg.torque.x = 0.0
        wrench_msg.torque.y = 0.0
        wrench_msg.torque.z = float(tau_yaw)
        self.control_pub.publish(wrench_msg)

        cross_track_error_mag = math.sqrt(cross_x**2 + cross_y**2)
        self.get_logger().info(
            f"Pos=({current_x:.2f}, {current_y:.2f}), "
            f"Yaw={math.degrees(current_yaw):.1f} deg, "
            f"dYaw={(math.degrees(yaw_error)):.1f} deg, "
            f"SegDist={along_track_dist:.2f}/{segment_length:.2f}, "
            f"XTE={cross_track_error_mag:.3f}, "
            f"Fx={tau_x:.2f}, Fy={tau_y:.2f}, Tz={tau_yaw:.2f}"
            f"SurgeVel={current_surge_velocity:.2f}, SwayVel={current_sway_velocity:.2f}, "
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


if __name__ == "__main__":
    main()
