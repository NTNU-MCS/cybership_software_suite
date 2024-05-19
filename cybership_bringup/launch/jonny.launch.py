#!/usr/bin/env python3

import launch_ros
import launch
import launch.actions
import launch.substitutions
import launch.launch_description_sources


def include_launch_action_with_config(bringup_pkg_dir, config_pkg_dir, model, launch_file, config_file, vessel_name):
    return launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            launch.substitutions.PathJoinSubstitution(
                [bringup_pkg_dir, 'launch', 'include', launch_file]
            )
        ),
        launch_arguments=[
            (
                'param_file',
                launch.substitutions.PathJoinSubstitution(
                    launch.substitutions.PathJoinSubstitution(
                        [config_pkg_dir, 'config', model, config_file]
                    )
                )
            ),
            (
                'vessel_name',
                vessel_name
            )
        ]
    )

def generate_launch_description():

    vessel_name = 'CSV'

    model = 'jonny'

    bringup_pkg_dir = launch_ros.substitutions.FindPackageShare('cybership_bringup')

    config_pkg_dir = launch_ros.substitutions.FindPackageShare('cybership_config')


    ld = launch.LaunchDescription()

    ld.add_action(
        include_launch_action_with_config(
            bringup_pkg_dir, config_pkg_dir, model, 'motion_capture_system_connector.launch.py', 'mocap_connector.yaml', vessel_name
        )
    )

    ld.add_action(
        include_launch_action_with_config(
            bringup_pkg_dir, config_pkg_dir, model, 'motion_capture_system_transformer.launch.py', 'mocap_transformer.yaml', vessel_name
        )
    )

    # ld.add_action(
    #     include_launch_action_with_config(
    #         bringup_pkg_dir, config_pkg_dir, model, 'robot_localization.launch.py', 'robot_localization.yaml', vessel_name
    #     )
    # )

    ld.add_action(
        include_launch_action_with_config(
            bringup_pkg_dir, config_pkg_dir, model, 'azimuth_controller.launch.py', 'azimuth_controller.yaml', vessel_name
        )
    )

    ld.add_action(
        include_launch_action_with_config(
            bringup_pkg_dir, config_pkg_dir, model, 'servo_driver.launch.py', 'servo_driver.yaml', vessel_name
        )
    )

    ld.add_action(
        include_launch_action_with_config(
            bringup_pkg_dir, config_pkg_dir, model, 'thruster_control.launch.py', 'thruster_control.yaml', vessel_name
        )
    )

    ld.add_action(
        include_launch_action_with_config(
            bringup_pkg_dir, config_pkg_dir, model, 'imu_bno055.launch.py', 'imu_bno055.yaml', vessel_name
        )
    )

    return ld