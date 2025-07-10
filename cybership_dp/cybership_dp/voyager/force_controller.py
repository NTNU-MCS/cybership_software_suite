import rclpy
import rclpy.node

import geometry_msgs.msg

import numpy as np
import skadipy
import skadipy.allocator.reference_filters


class ForceControllerROS(rclpy.node.Node):
    def __init__(self, *args, **kwargs):
        super().__init__(
            node_name="voyager_thrust_allocator",
            allow_undeclared_parameters=True,
            *args,
            **kwargs,
        )

        self.actuators: list[skadipy.actuator.ActuatorBase] = []
        self.allocator: skadipy.allocator.AllocatorBase = None

        self.tau_cmd = np.zeros((6, 1), dtype=np.float32)
        self.subs = {}
        self.subs["force"] = self.create_subscription(
            geometry_msgs.msg.Wrench, "control/force/command", self.force_callback, 10
        )

        self.pubs = {}
        self.pubs["port_thruster"] = self.create_publisher(
            geometry_msgs.msg.Wrench, "thruster/port/command", 10
        )
        self.pubs["starboard_thruster"] = self.create_publisher(
            geometry_msgs.msg.Wrench, "thruster/starboard/command", 10
        )
        self.pubs["tunnel_thruster"] = self.create_publisher(
            geometry_msgs.msg.Wrench, "thruster/tunnel/command", 10
        )

        self.declare_parameter("frequency", rclpy.Parameter.Type.DOUBLE)
        self.freq = (
            self.get_parameter_or("frequency", 1.0).get_parameter_value().double_value
        )

        self._initialize_thrusters()

        self._initialize_allocator()

        self.create_timer(1.0 / self.freq, self.timer_callback)

    def timer_callback(self):

        self.allocator.allocate(tau=self.tau_cmd)

        # Tunnel thruster
        u0_f = self.actuators[0].force
        msg = geometry_msgs.msg.Wrench()
        msg.force.x = float(u0_f[0])

        self.pubs["tunnel_thruster"].publish(msg)

        # Port azimuth thruster
        u1_f = self.actuators[1].force
        msg = geometry_msgs.msg.Wrench()
        msg.force.x = float(u1_f[0])
        msg.force.y = float(u1_f[1])

        self.pubs["port_thruster"].publish(msg)

        # Starboard azimuth thruster
        u2_f = self.actuators[2].force
        msg = geometry_msgs.msg.Wrench()
        msg.force.x = float(u2_f[0])
        msg.force.y = float(u2_f[1])

        self.pubs["starboard_thruster"].publish(msg)


    def force_callback(self, msg: geometry_msgs.msg.Wrench):

        self.tau_cmd = np.array(
            [
                msg.force.x,
                msg.force.y,
                msg.force.z,
                msg.torque.x,
                msg.torque.y,
                msg.torque.z,
            ],
            dtype=np.float32,
        ).reshape((6, 1))

    def _initialize_thrusters(self):
        tunnel = skadipy.actuator.Fixed(
            position=skadipy.toolbox.Point([0.3875, 0.0, 0.0]),
            orientation=skadipy.toolbox.Quaternion(
                axis=(0.0, 0.0, 1.0), radians=np.pi / 2.0
            ),
            extra_attributes={
                "rate_limit": 0.1,
                "saturation_limit": 0.7,
                "name": "tunnel",
            },
        )
        port_azimuth = skadipy.actuator.Azimuth(
            position=skadipy.toolbox.Point([-0.4574, -0.055, 0.0]),
            extra_attributes={
                "rate_limit": 0.1,
                "saturation_limit": 1.0,
                "reference_angle": np.pi / 2.0,
                "name": "port_azimuth",
            },
        )
        starboard_azimuth = skadipy.actuator.Azimuth(
            position=skadipy.toolbox.Point([-0.4547, 0.055, 0.0]),
            extra_attributes={
                "rate_limit": 0.1,
                "saturation_limit": 1.0,
                "reference_angle": -np.pi / 2.0,
                "name": "starboard_azimuth",
            },
        )

        # Put all actuators in a list and create the allocator object
        self.actuators = [
            tunnel,
            port_azimuth,
            starboard_azimuth,
        ]

    def _initialize_allocator(self):
        dofs = [
            skadipy.allocator.ForceTorqueComponent.X,
            skadipy.allocator.ForceTorqueComponent.Y,
            skadipy.allocator.ForceTorqueComponent.N,
        ]
        self.allocator = skadipy.allocator.reference_filters.MinimumMagnitudeAndAzimuth(
            actuators=self.actuators,
            force_torque_components=dofs,
            gamma=0.001,
            mu=0.01,
            rho=1,
            time_step=(1.0 /self.freq ),
            control_barrier_function=skadipy.safety.ControlBarrierFunctionType.SUMSQUARE,
        )
        self.allocator.compute_configuration_matrix()


def main(args=None):
    rclpy.init(args=args)
    force_controller = ForceControllerROS()
    rclpy.spin(force_controller)
    force_controller.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
