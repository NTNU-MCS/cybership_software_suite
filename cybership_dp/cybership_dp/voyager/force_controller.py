import rclpy

import numpy as np
import skadipy
import skadipy.allocator.reference_filters

from cybership_dp.force_controller_base import BaseForceControllerROS
from cybership_dp.force_controller_base import ThrusterPublisherSpec


class ForceControllerROS(BaseForceControllerROS):
    def __init__(self, *args, **kwargs):
        super().__init__(
            node_name="voyager_thrust_allocator",
            allow_undeclared_parameters=True,
            *args,
            **kwargs,
        )

    def _thruster_publisher_specs(self) -> list[ThrusterPublisherSpec]:
        return [
            ThrusterPublisherSpec(
                key="tunnel_thruster",
                topic="thruster/tunnel/command",
                actuator_index=0,
                mapping={"force.x": 0},
            ),
            ThrusterPublisherSpec(
                key="port_thruster",
                topic="thruster/port/command",
                actuator_index=1,
                mapping={"force.x": 0, "force.y": 1},
            ),
            ThrusterPublisherSpec(
                key="starboard_thruster",
                topic="thruster/starboard/command",
                actuator_index=2,
                mapping={"force.x": 0, "force.y": 1},
            ),
        ]

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
            gamma=0.01,
            mu=0.1,
            rho=1,
            time_step=(1.0 / self.freq),
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
