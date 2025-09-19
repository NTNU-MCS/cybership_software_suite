import launch_ros
import launch
import launch.actions
import launch.substitutions
import launch.launch_description_sources

from cybership_utilities.launch import include_launch_action_with_config



def generate_launch_description():

    vessel_name = 'voyager'

    vessel_model = 'voyager'

    ld = launch.LaunchDescription()

    ld.add_action(
        include_launch_action_with_config(
            vessel_model, vessel_name,
            'azimuth_controller.launch.py', 'azimuth_controller.yaml'
        )
    )

    ld.add_action(
        include_launch_action_with_config(
            vessel_model, vessel_name,
            'motion_capture_system_connector.launch.py', 'mocap_connector.yaml'
        )
    )

    ld.add_action(
        include_launch_action_with_config(
            vessel_model, vessel_name,
            'motion_capture_system_transformer.launch.py', 'mocap_transformer.yaml'
        )
    )

    ld.add_action(
        include_launch_action_with_config(
            vessel_model, vessel_name,
            'force_multiplexer.launch.py', 'force_multiplexer.yaml'
        )
    )

    ld.add_action(
        include_launch_action_with_config(
            vessel_model, vessel_name,
            'servo_driver.launch.py', 'servo_driver.yaml'
        )
    )

    ld.add_action(
        include_launch_action_with_config(
            vessel_model, vessel_name,
            'thruster_control.launch.py', 'thruster_control.yaml'
        )
    )

    ld.add_action(
        include_launch_action_with_config(
            vessel_model, vessel_name,
            'imu_bno055.launch.py', 'imu_bno055.yaml'
        )
    )

    # ld.add_action(
    #     include_launch_action_with_config(
    #         vessel_model, vessel_name,
    #         'urdf_description.launch.py', 'empty.yaml'
    #     )
    # )

    return ld
