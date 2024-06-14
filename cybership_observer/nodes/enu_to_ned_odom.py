#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
import tf_transformations
import numpy as np

class NEDToENUPublisher(Node):

    def __init__(self):
        super().__init__('ned_to_enu_publisher')

        # Declare and get the frame_id parameter
        self.declare_parameter('frame_id', 'world')
        self.frame_id = self.get_parameter(
            'frame_id').get_parameter_value().string_value

        # Subscriber to the NED frame PoseWithCovarianceStamped messages
        self.ned_sub = self.create_subscription(
            Odometry,
            'in_odom',
            self.ned_callback,
            10
        )

        # Publisher for the transformed ENU frame PoseWithCovarianceStamped messages
        self.enu_pub = self.create_publisher(
            Odometry,
            'out_odom',
            10
        )

    def ned_callback(self, msg):
        # Transform the pose from NED to ENU frame
        enu_msg = Odometry()
        enu_msg.header = msg.header
        enu_msg.header.frame_id = self.frame_id  # Set the frame_id parameter

        enu_msg.child_frame_id = msg.child_frame_id

        # Position transformation
        enu_msg.pose.pose.position.x = - msg.pose.pose.position.y
        enu_msg.pose.pose.position.y = - msg.pose.pose.position.x
        enu_msg.pose.pose.position.z = - msg.pose.pose.position.z

        enu_msg.twist.twist.linear.x = msg.twist.twist.linear.y
        enu_msg.twist.twist.linear.y = msg.twist.twist.linear.x
        enu_msg.twist.twist.linear.z = - msg.twist.twist.linear.z

        enu_msg.twist.twist.angular.x = msg.twist.twist.angular.x
        enu_msg.twist.twist.angular.y = - msg.twist.twist.angular.y
        enu_msg.twist.twist.angular.z = - msg.twist.twist.angular.z


        # Orientation transformation (quaternion)
        q_ned = [
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w
        ]

        roll, pitch, yaw = tf_transformations.euler_from_quaternion(
            q_ned, axes='sxyz')  # Check the orientation in Euler angles

        q_enu = tf_transformations.quaternion_from_euler(
            roll, pitch, -yaw - np.pi / 2.0, axes='sxyz')

        enu_msg.pose.pose.orientation.x = q_enu[0]
        enu_msg.pose.pose.orientation.y = q_enu[1]
        enu_msg.pose.pose.orientation.z = q_enu[2]
        enu_msg.pose.pose.orientation.w = q_enu[3]

        # Copy the covariance matrix as it is (assuming it remains the same)
        enu_msg.pose.covariance = msg.pose.covariance

        # Publish the transformed message
        self.enu_pub.publish(enu_msg)


def main(args=None):
    rclpy.init(args=args)
    node = NEDToENUPublisher()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
