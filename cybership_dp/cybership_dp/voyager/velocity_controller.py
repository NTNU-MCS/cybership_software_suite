import rclpy
import rclpy.node

import geometry_msgs.msg
import nav_msgs.msg

import numpy as np
import skadipy
import skadipy.allocator.reference_filters


class VelocityControllerROS(rclpy.node.Node):
    def __init__(self, *args, **kwargs):
        super().__init__(
            node_name="cybership_velocity_controller",
            allow_undeclared_parameters=True,
            *args,
            **kwargs,
        )

        self.vel_cmd_msg = geometry_msgs.msg.Twist()
        self.odom_msg = nav_msgs.msg.Odometry()

        self.subs = {}
        self.subs["odometry"] = self.create_subscription(
            nav_msgs.msg.Odometry, "measurements/odometry", self.odom_callback, 10
        )
        self.subs["command"] = self.create_subscription(
            geometry_msgs.msg.Twist,
            "control/velocity/command",
            self.vel_cmd_callback,
            10,
        )

        self.pubs = {}
        self.pubs["force"] = self.create_publisher(
            geometry_msgs.msg.Wrench, "control/force/command", 10
        )

        self.declare_parameter("frequency", rclpy.Parameter.Type.DOUBLE)
        self.freq = (
            self.get_parameter_or("frequency", 1.0).get_parameter_value().double_value
        )

        self.create_timer(1.0 / self.freq, self.timer_callback)

    def timer_callback(self):

        error_msg = geometry_msgs.msg.Wrench()

        error_msg.force.x = 1 * (
            - self.odom_msg.twist.twist.linear.x + self.vel_cmd_msg.linear.x
        )
        error_msg.force.y = 1 * (
            - self.odom_msg.twist.twist.linear.y + self.vel_cmd_msg.linear.y
        )
        error_msg.torque.z = 3 * (
            - self.odom_msg.twist.twist.angular.z + self.vel_cmd_msg.angular.z
        )

        self.pubs["force"].publish(error_msg)

    def vel_cmd_callback(self, msg: geometry_msgs.msg.Twist):
        self.vel_cmd_msg = msg

    def odom_callback(self, msg: geometry_msgs.msg.Wrench):
        self.odom_msg = msg


def main(args=None):
    rclpy.init(args=args)
    velocity_controller = VelocityControllerROS()
    rclpy.spin(velocity_controller)
    velocity_controller.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
