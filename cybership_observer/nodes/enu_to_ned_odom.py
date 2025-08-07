#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, PoseWithCovariance, TwistWithCovariance, Quaternion

from tf_transformations import euler_from_quaternion, quaternion_from_euler

class OdomEnuToNedNode(Node):
    def __init__(self):
        super().__init__('odom_enu_to_ned')
        self.sub = self.create_subscription(
            Odometry,
            'in_odom',
            self.odom_callback,
            10
        )
        self.declare_parameter('frame_id', 'world')
        self.pub = self.create_publisher(Odometry, 'out_odom', 10)

    def odom_callback(self, msg):
        ned_msg = Odometry()
        ned_msg.header = msg.header
        ned_msg.child_frame_id = "base_link_ned"

        # Position transformation
        ned_msg.pose.pose.position.x = msg.pose.pose.position.y
        ned_msg.pose.pose.position.y = msg.pose.pose.position.x
        ned_msg.pose.pose.position.z = -msg.pose.pose.position.z

        # Orientation transformation
        # Convert quaternion to euler
        q = msg.pose.pose.orientation
        euler = euler_from_quaternion([q.x, q.y, q.z, q.w])

        roll = euler[0]
        pitch = -euler[1]
        yaw = -euler[2]

        # Convert back to quaternion
        new_q = quaternion_from_euler(roll, pitch, yaw)
        ned_msg.pose.pose.orientation = Quaternion()
        ned_msg.pose.pose.orientation.x = new_q[0]
        ned_msg.pose.pose.orientation.y = new_q[1]
        ned_msg.pose.pose.orientation.z = new_q[2]
        ned_msg.pose.pose.orientation.w = new_q[3]

        # Velocity transformation
        ned_msg.twist.twist.linear.x = msg.twist.twist.linear.y
        ned_msg.twist.twist.linear.y = msg.twist.twist.linear.x
        ned_msg.twist.twist.linear.z = -msg.twist.twist.linear.z

        ned_msg.twist.twist.angular.x = msg.twist.twist.angular.y
        ned_msg.twist.twist.angular.y = msg.twist.twist.angular.x
        ned_msg.twist.twist.angular.z = -msg.twist.twist.angular.z

        self.pub.publish(ned_msg)

def main(args=None):
    rclpy.init(args=args)
    node = OdomEnuToNedNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
