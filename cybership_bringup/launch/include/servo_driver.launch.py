#!/usr/bin/env python3

import launch
import launch.actions
import launch.substitutions
import launch_ros.actions
from cybership_utilities.utilities import anon


def generate_launch_description():

    arg_vessel_name = launch.actions.DeclareLaunchArgument(
        'vessel_name',
        default_value='cybership',
        description='vessel_name'
    )

    arg_param_file = launch.actions.DeclareLaunchArgument(
        'param_file',
        default_value=launch.substitutions.PathJoinSubstitution(
            [launch_ros.substitutions.FindPackageShare('cybership_config'), 'config', 'any', 'empty.config.yaml']
        ),
        description='Configuration for servo driver node'
    )

    node_servo_driver = launch_ros.actions.Node(
        package='ros2_pca9685',
        executable='ros2_pca9685_node',
        namespace=launch.substitutions.LaunchConfiguration('vessel_name'),
        name=f'pwm_driver_node_{anon()}',
        parameters=[launch.substitutions.LaunchConfiguration('param_file')],
        output='screen',
        respawn=True,
        respawn_delay=5
    )

    ld = launch.LaunchDescription()
    ld.add_action(arg_vessel_name)
    ld.add_action(arg_param_file)
    ld.add_action(node_servo_driver)

    return ld