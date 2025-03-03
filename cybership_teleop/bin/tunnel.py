#!/usr/bin/env python3

import rclpy
import rclpy.node
import geometry_msgs.msg
import sensor_msgs.msg


class enterpriseTeleop(rclpy.node.Node):

    def __init__(self):
        super().__init__('cse_teleop')

        # Declare parameters for the axes
        self.declare_parameter('axis1', 5)
        self.declare_parameter('axis2', 2)

        # Retrieve the parameters
        self.axis1 = self.get_parameter('axis1').get_parameter_value().integer_value
        self.axis2 = self.get_parameter('axis2').get_parameter_value().integer_value

        self.publisher = self.create_publisher(geometry_msgs.msg.Wrench, 'thruster/tunnel/command', 1)
        self.subscriber = self.create_subscription(sensor_msgs.msg.Joy, 'joy', self.cb_joy, 10)

    def cb_joy(self, msg):
        wrench = geometry_msgs.msg.Wrench()
        wrench.force.x = 0.4 * (msg.axes[self.axis1] - msg.axes[self.axis2]) / 2.0
        self.publisher.publish(wrench)


def main(args=None):
    rclpy.init(args=args)

    cse_teleop = enterpriseTeleop()

    rclpy.spin(cse_teleop)
    cse_teleop.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()