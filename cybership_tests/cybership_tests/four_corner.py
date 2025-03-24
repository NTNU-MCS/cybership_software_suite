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
        # Define corners of the square around center (0,0)
        self.corners = [
            (0.0, 0.0),
            (self.side_length, 0.0),
            (self.side_length, self.side_length),
            (0.0, self.side_length),
        ]
        for i in range(len(self.corners)):
            self.corners[i] = (self.corners[i][0] - 1.5, self.corners[i][1] - 1.5)

        self.corner_headings = [
            math.pi/4,          # corner 1 angle
            0.0,                # corner 3 angle (radians)
            math.pi/4,          # corner 1 angle
            0.0,            # corner 2 angle
        ]

        # Controller gains
        self.kp_xy = 1.7   # Proportional gain for cross-track and along-track position
        self.kp_yaw = 0.6  # Proportional gain for heading

        # Desired speed along each segment (m/s) if you want to incorporate forward motion control
        self.desired_speed = 0.25  # Desired speed in m/s
        self.kp_speed = 1.2  # Proportional gain for speed tracking

        # Index to track the current segment start corner
        self.current_waypoint_index = 0

        # A small threshold to decide when we have "arrived" near the next corner
        self.segment_arrival_threshold = 0.25

        self.get_logger().info("Square Move Controller with cross-track error minimization initialized.")

    def odom_callback(self, msg: Odometry):
        """
        Callback to update the latest odometry measurement.
        """
        self.latest_odom = msg

    def control_loop(self):
        """
        Control loop that computes and publishes the control command.
        The vessel orientation must remain at yaw=0.
        We implement a line-segment based path tracking to minimize cross track error.
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

        # Determine the current segment from corners[i] to corners[i+1]
        p1_index = self.current_waypoint_index
        p2_index = (self.current_waypoint_index + 1) % len(self.corners)
        p1 = self.corners[p1_index]
        p2 = self.corners[p2_index]

        # Segment vector
        dx = p2[0] - p1[0]
        dy = p2[1] - p1[1]
        segment_length = math.sqrt(dx*dx + dy*dy)

        # Handle degenerate case
        if segment_length < 1e-6:
            # Move to the next segment
            self.current_waypoint_index = p2_index
            return

        # Normalized direction along the segment
        ux = dx / segment_length
        uy = dy / segment_length

        # Vector from p1 to current position
        ex = current_x - p1[0]
        ey = current_y - p1[1]

        # Project ex,ey onto the segment direction to find along-track distance
        along_track_dist = ex * ux + ey * uy

        # If we've passed the end of the segment, switch to next
        if along_track_dist >= segment_length - self.segment_arrival_threshold:
            self.current_waypoint_index = p2_index
            return

        # If we haven't reached the start (if going in reverse, param < 0), just clamp
        if along_track_dist < 0.0:
            along_track_dist = 0.0

        # Nearest point on the segment to the current position
        nearest_x = p1[0] + along_track_dist * ux
        nearest_y = p1[1] + along_track_dist * uy

        # Cross track error vector from the nearest point to current position
        cross_x = current_x - nearest_x
        cross_y = current_y - nearest_y
        cross_track_error = math.sqrt(cross_x*cross_x + cross_y*cross_y)

        # Force to correct cross-track error (P-control)
        # We push the vehicle back toward the line.
        # The negative sign ensures we push opposite the cross-track offset
        tau_ct_x = -self.kp_xy * cross_x
        tau_ct_y = -self.kp_xy * cross_y

        # Optionally, add along-track control to move forward along the line
        # We can measure velocity in the global frame if needed
        vx = self.latest_odom.twist.twist.linear.x
        vy = self.latest_odom.twist.twist.linear.y
        # along-track velocity
        v_along = vx * ux + vy * uy
        # simple P-control for speed
        speed_err = self.desired_speed - v_along
        tau_along_x = self.kp_speed * speed_err * ux
        tau_along_y = self.kp_speed * speed_err * uy

        # Total x,y control effort
        tau_x = tau_ct_x + tau_along_x
        tau_y = tau_ct_y + tau_along_y

        # We want the vessel's orientation to stay at yaw=0, so heading error:
        desired_yaw = self.corner_headings[self.current_waypoint_index]
        yaw_error = wrap_to_pi(desired_yaw - current_yaw)
        tau_yaw = self.kp_yaw * yaw_error

        # Create and publish the Wrench message
        wrench_msg = Wrench()
        wrench_msg.force.x = float(tau_x)
        wrench_msg.force.y = float(tau_y)
        wrench_msg.force.z = 0.0
        wrench_msg.torque.x = 0.0
        wrench_msg.torque.y = 0.0
        wrench_msg.torque.z = float(tau_yaw)

        self.control_pub.publish(wrench_msg)

        self.get_logger().info(
            f"Pos=({current_x:.2f}, {current_y:.2f}), Yaw={current_yaw:.2f}, "
            f"Segment=[{p1}, {p2}], Along={along_track_dist:.2f}, XTE={cross_track_error:.3f}, "
            f"Fx={tau_x:.2f}, Fy={tau_y:.2f}, Tz={tau_yaw:.2f}"
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
