#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import tf_transformations
import numpy as np

class ENUtoNEDPublisher(Node):

    def __init__(self):
        super().__init__('enu_to_ned_twist_publisher')

        # Declare and get the frame_id parameter
        self.declare_parameter('frame_id', 'world')
        self.frame_id = self.get_parameter(
            'frame_id').get_parameter_value().string_value

        # Subscriber to the NED frame PoseWithCovarianceStamped messages
        self.ned_sub = self.create_subscription(
            Twist,
            'in_twist',
            self.ned_callback,
            10
        )

        # Publisher for the transformed ENU frame PoseWithCovarianceStamped messages
        self.enu_pub = self.create_publisher(
            Twist,
            'out_twist',
            10
        )

    def ned_callback(self, msg):
        # Transform the pose from NED to ENU frame
        enu_msg = Twist()

        enu_msg.linear.x = msg.linear.x
        enu_msg.linear.y = -msg.linear.y
        enu_msg.linear.z = -msg.linear.z

        enu_msg.angular.x = msg.angular.x
        enu_msg.angular.y = -msg.angular.y
        enu_msg.angular.z = -msg.angular.z

        # Publish the transformed message
        self.enu_pub.publish(enu_msg)


def main(args=None):
    rclpy.init(args=args)
    node = ENUtoNEDPublisher()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
