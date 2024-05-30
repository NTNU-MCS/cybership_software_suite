import rclpy
import rclpy.node

import geometry_msgs.msg
import nav_msgs.msg
import rcl_interfaces.msg

import numpy as np


class VelocityControllerROS(rclpy.node.Node):
    def __init__(self, *args, **kwargs):
        super().__init__(
            node_name="voyager_velocity_controller",
            allow_undeclared_parameters=True,
            automatically_declare_parameters_from_overrides=False,
        )

        # Declare and get parameters
        self._declare_parameters()
        self._update_parameters()

        # Initialize messages
        self._vel_cmd_msg = geometry_msgs.msg.Twist()
        self._odom_msg = nav_msgs.msg.Odometry()

        # Initialize variables
        self._previous_error = np.zeros(3, dtype=np.float32)
        self._k_pid = np.zeros((3, 3), dtype=np.float32)

        # Initialize subscribers and publishers
        self._subs = {}
        self._pubs = {}
        self._subs["odometry"] = self.create_subscription(
            nav_msgs.msg.Odometry, "measurement/odom", self._odom_callback, 10
        )
        self._subs["command"] = self.create_subscription(
            geometry_msgs.msg.Twist,
            "control/velocity/command",
            self._vel_cmd_callback,
            10,
        )
        self._pubs["force"] = self.create_publisher(
            geometry_msgs.msg.Wrench, "control/force/command", 10
        )

        # Initialize timer
        self._dt = 1.0 / self._freq
        self.create_timer(self._dt, self.timer_callback)

    def timer_callback(self):

        self._update_parameters()

        cmd_force = geometry_msgs.msg.Wrench()

        x = np.array(
            [
                self._odom_msg.twist.twist.linear.x,
                self._odom_msg.twist.twist.linear.y,
                self._odom_msg.twist.twist.angular.z,
            ],
            dtype=np.float32,
        )

        u = np.array(
            [
                self._vel_cmd_msg.linear.x,
                self._vel_cmd_msg.linear.y,
                self._vel_cmd_msg.angular.z,
            ],
            dtype=np.float32,
        )

        # Calculate error
        e = u - x

        # Proportional term
        self._k_pid[0] = self._K_P @ e

        # Integral term
        self._k_pid[1] += self._K_I @ (e * self._dt)

        # Derivative term
        self._k_pid[2] = self._K_D @ (e - self._previous_error) / self._dt

        # Calculate control force
        cmd = np.array(
            [
                self._k_pid[0][0] + self._k_pid[1][0] + self._k_pid[2][0],
                self._k_pid[0][1] + self._k_pid[1][1] + self._k_pid[2][1],
                self._k_pid[0][2] + self._k_pid[1][2] + self._k_pid[2][2],
            ],
            dtype=float,
        )

        cmd_force.force.x = cmd[0]
        cmd_force.force.y = cmd[1]
        cmd_force.torque.z = cmd[2]

        self._pubs["force"].publish(cmd_force)
        self._previous_error = e

    def _vel_cmd_callback(self, msg: geometry_msgs.msg.Twist):
        self._vel_cmd_msg = msg

    def _odom_callback(self, msg: geometry_msgs.msg.Wrench):
        self._odom_msg = msg

    def _declare_parameters(self):

        self.declare_parameter("frequency", rclpy.Parameter.Type.DOUBLE)

        dofs = ["surge", "sway", "yaw"]
        gains = ["p_gain", "i_gain", "i_limit", "d_gain"]
        for dof in dofs:
            for gain in gains:
                self.declare_parameter(
                    f"{dof}.{gain}",
                    0.0,
                    rcl_interfaces.msg.ParameterDescriptor(
                        description=f"{dof} {gain}",
                        type=rcl_interfaces.msg.ParameterType.PARAMETER_DOUBLE,
                        read_only=False,
                    ),
                )

    def _update_parameters(self):
        self._freq = self.get_parameter("frequency").get_parameter_value().double_value

        self._surge_p_gain = (
            self.get_parameter("surge.p_gain").get_parameter_value().double_value
        )
        self._surge_i_gain = (
            self.get_parameter("surge.i_gain").get_parameter_value().double_value
        )
        self._surge_d_gain = (
            self.get_parameter("surge.d_gain").get_parameter_value().double_value
        )
        self._surge_i_limit = (
            self.get_parameter("surge.i_limit").get_parameter_value().double_value
        )

        self._sway_p_gain = (
            self.get_parameter("sway.p_gain").get_parameter_value().double_value
        )
        self._sway_i_gain = (
            self.get_parameter("sway.i_gain").get_parameter_value().double_value
        )
        self._sway_d_gain = (
            self.get_parameter("sway.d_gain").get_parameter_value().double_value
        )
        self._sway_i_limit = (
            self.get_parameter("sway.i_limit").get_parameter_value().double_value
        )

        self._yaw_p_gain = (
            self.get_parameter("yaw.p_gain").get_parameter_value().double_value
        )
        self._yaw_i_gain = (
            self.get_parameter("yaw.i_gain").get_parameter_value().double_value
        )
        self._yaw_d_gain = (
            self.get_parameter("yaw.d_gain").get_parameter_value().double_value
        )
        self._yaw_i_limit = (
            self.get_parameter("yaw.i_limit").get_parameter_value().double_value
        )

        self._K_P = np.array(
            [
                [self._surge_p_gain, 0, 0],
                [0, self._sway_p_gain, 0],
                [0, 0, self._yaw_p_gain],
            ],
            dtype=np.float32,
        )

        self._K_I = np.array(
            [
                [self._surge_i_gain, 0, 0],
                [0, self._sway_i_gain, 0],
                [0, 0, self._yaw_i_gain],
            ],
            dtype=np.float32,
        )

        self._K_D = np.array(
            [
                [self._surge_d_gain, 0, 0],
                [0, self._sway_d_gain, 0],
                [0, 0, self._yaw_d_gain],
            ],
            dtype=np.float32,
        )


def main(args=None):
    rclpy.init(args=args)
    velocity_controller = VelocityControllerROS()
    rclpy.spin(velocity_controller)
    velocity_controller.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
