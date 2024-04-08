#!/usr/bin/env python3

import launch
import launch.actions
import launch.substitutions
import launch_ros.actions
import os
from ament_index_python.packages import get_package_share_directory

import lifecycle_msgs.msg

from launch.actions import SetEnvironmentVariable
from launch_ros.events.lifecycle import ChangeState


def generate_launch_description():

    pkg_cybership_bringup = get_package_share_directory('cybership_bringup')

    config_param_file = launch.actions.DeclareLaunchArgument(
        'param_file',
        default_value=launch.substitutions.PathJoinSubstitution(
            [pkg_cybership_bringup, 'config', 'any', 'mocap_transformer.config.yaml']
        ),
        description='Motion Capture System transformer configuration file'
    )

    node_mocap_transformer = launch_ros.actions.Node(
        package='cybership_mocap',
        executable='cybership_mocap_node',
        name='mocap_transformer_node',
        parameters=[launch.substitutions.LaunchConfiguration('param_file')],
        output='screen'
    )

    ld = launch.LaunchDescription()
    ld.add_action(config_param_file)
    ld.add_action(node_mocap_transformer)
    return ld