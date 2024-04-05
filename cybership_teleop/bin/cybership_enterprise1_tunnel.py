#!/usr/bin/env python3

import rclpy
import rclpy.node
import geometry_msgs.msg
import sensor_msgs.msg


class CSEITeleop(rclpy.node.Node):

    def __init__(self):
        super().__init__('cse_teleop')

        self.publisher = self.create_publisher(geometry_msgs.msg.Wrench, '/CSEI/thrusters/tunnel/command', 1)
        self.subscriber = self.create_subscription(sensor_msgs.msg.Joy, '/joy', self.cb_joy, 10)

    def cb_joy(self, msg):


        wrench = geometry_msgs.msg.Wrench()
        wrench.force.x = 0.4 * (msg.axes[5] - msg.axes[2]) / 2.0
        self.publisher.publish(wrench)


def main(args=None):
    rclpy.init(args=args)

    cse_teleop = CSEITeleop()

    rclpy.spin(cse_teleop)

    cse_teleop.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()