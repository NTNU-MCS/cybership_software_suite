#!/usr/bin/env python3

import launch
import launch.actions
import launch.substitutions
import launch.launch_description_sources

from ament_index_python.packages import get_package_share_directory

def include_launch_action_with_config(pkg_dir, model, launch_file, config_file):
    return launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            [pkg_dir, 'launch', 'include', launch_file]
        ),
        launch_arguments=[
            ('param_file',
                launch.substitutions.PathJoinSubstitution(
                    [pkg_dir, 'config', model, config_file]
                )
            )
        ]
    )

def generate_launch_description():

    model = 'enterprise1'

    pkg_dir = get_package_share_directory('cybership_bringup')


    ld = launch.LaunchDescription()

    ld.add_action(
        include_launch_action_with_config(
            pkg_dir, model, 'motion_capture_system_connector.launch.py', 'mocap_connector.yaml'
        )
    )

    ld.add_action(
        include_launch_action_with_config(
            pkg_dir, model, 'motion_capture_system_transformer.launch.py', 'mocap_transformer.yaml'
        )
    )

    ld.add_action(
        include_launch_action_with_config(
            pkg_dir, model, 'robot_localization.launch.py', 'robot_localization.yaml'
        )
    )

    ld.add_action(
        include_launch_action_with_config(
            pkg_dir, model, 'servo_driver.launch.py', 'servo_driver.yaml'
        )
    )

    ld.add_action(
        include_launch_action_with_config(
            pkg_dir, model, 'thruster_control.launch.py', 'thruster_control.yaml'
        )
    )

    return ld