#!/usr/bin/env python3

import launch
import launch.actions
import launch.substitutions
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    pkg_cybership_bringup = get_package_share_directory('cybership_bringup')

    config_param_file = launch.actions.DeclareLaunchArgument(
        'param_file',
        default_value=launch.substitutions.PathJoinSubstitution(
            [pkg_cybership_bringup, 'config', 'any', 'empty.config.yaml']
        ),
        description='Motion Capture System transformer configuration file'
    )

    node_robot_localization = launch_ros.actions.Node(
        package='robot_localization',
        executable='ekf_node',
        name='robot_localization_node',
        parameters=[launch.substitutions.LaunchConfiguration('param_file')],
        output='screen'
    )

    ld = launch.LaunchDescription()
    ld.add_action(config_param_file)
    ld.add_action(node_robot_localization)
    return ld