import rclpy

import numpy as np
import skadipy
import skadipy.allocator.reference_filters

from cybership_dp.force_controller_base import BaseForceControllerROS
from cybership_dp.force_controller_base import ThrusterPublisherSpec


class ForceControllerROS(BaseForceControllerROS):
    def __init__(self, *args, **kwargs):
        super().__init__(
            node_name="drillship_thrust_allocator",
            allow_undeclared_parameters=True,
            *args,
            **kwargs,
        )

    def _thruster_publisher_specs(self) -> list[ThrusterPublisherSpec]:
        # Keep the same active thrusters as the previous implementation
        # (center thrusters remain commented-out in the actuator list).
        return [
            ThrusterPublisherSpec(
                key="bow_port_azimuth",
                topic="thruster/bow_port/command",
                actuator_index=0,
                mapping={"force.x": 0, "force.y": 1},
            ),
            ThrusterPublisherSpec(
                key="bow_starboard_azimuth",
                topic="thruster/bow_starboard/command",
                actuator_index=1,
                mapping={"force.x": 0, "force.y": 1},
            ),
            ThrusterPublisherSpec(
                key="stern_port_azimuth",
                topic="thruster/stern_port/command",
                actuator_index=2,
                mapping={"force.x": 0, "force.y": 1},
            ),
            ThrusterPublisherSpec(
                key="stern_starboard_azimuth",
                topic="thruster/stern_starboard/command",
                actuator_index=3,
                mapping={"force.x": 0, "force.y": 1},
            ),
        ]

    def _initialize_thrusters(self):
        bow_port_azimuth = skadipy.actuator.Azimuth(
            position=skadipy.toolbox.Point([0.9344, -0.11, 0.0]),
            orientation=skadipy.toolbox.Quaternion(
                axis=(0.0, 0.0, 1.0), radians=np.pi / 2.0
            ),
            extra_attributes={
                "rate_limit": 0.1,
                "saturation_limit": 0.7,
                "reference_angle": 3* np.pi / 4.0,
            }
        )
        bow_center_azimuth = skadipy.actuator.Azimuth(
            position=skadipy.toolbox.Point([1.0678, 0.0, 0.0]),
            orientation=skadipy.toolbox.Quaternion(
                axis=(0.0, 0.0, 1.0), radians=np.pi / 2.0
            ),
            extra_attributes={
                "rate_limit": 0.1,
                "saturation_limit": 0.7,
                "reference_angle": -np.pi,
            }
        )
        bow_starboard_azimuth = skadipy.actuator.Azimuth(
            position=skadipy.toolbox.Point([0.9344, 0.11, 0.0]),
            orientation=skadipy.toolbox.Quaternion(
                axis=(0.0, 0.0, 1.0), radians=np.pi / 2.0
            ),
            extra_attributes={
                "rate_limit": 0.1,
                "saturation_limit": 0.7,
                "reference_angle": -3*np.pi / 4.0,
            }
        )

        stern_port_azimuth = skadipy.actuator.Azimuth(
            position=skadipy.toolbox.Point([-0.9911, -0.1644, -0.1]),
            orientation=skadipy.toolbox.Quaternion(
                axis=(0.0, 0.0, 1.0), radians=np.pi / 2.0
            ),
            extra_attributes={
                "rate_limit": 0.1,
                "saturation_limit": 0.7,
                "reference_angle": np.pi / 4.0,
            }
        )
        stern_center_azimuth = skadipy.actuator.Azimuth(
            position=skadipy.toolbox.Point([-1.1644, 0.0, 0.0]),
            orientation=skadipy.toolbox.Quaternion(
                axis=(0.0, 0.0, 1.0), radians=np.pi / 2.0
            ),
            extra_attributes={
                "rate_limit": 0.1,
                "saturation_limit": 0.7,
                "reference_angle": 0.0,
            }
        )
        stern_starboard_azimuth = skadipy.actuator.Azimuth(
            position=skadipy.toolbox.Point([-0.9911, 0.1644, 0.0]),
            orientation=skadipy.toolbox.Quaternion(
                axis=(0.0, 0.0, 1.0), radians=np.pi / 2.0
            ),
            extra_attributes={
                "rate_limit": 0.1,
                "saturation_limit": 0.7,
                "reference_angle": -np.pi / 4.0,
            }
        )

        # Put all actuators in a list and create the allocator object
        self.actuators = [
            bow_port_azimuth,
            # bow_center_azimuth,
            bow_starboard_azimuth,
            stern_port_azimuth,
            # stern_center_azimuth,
            stern_starboard_azimuth
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
