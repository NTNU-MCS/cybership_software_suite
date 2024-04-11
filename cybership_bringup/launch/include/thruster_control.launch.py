#!/usr/bin/env python3

import launch
import launch.actions
import launch.substitutions
import launch_ros.actions


def generate_launch_description():

    arg_vessel_name = launch.actions.DeclareLaunchArgument(
        'vessel_name',
        default_value='cybership',
        description='vessel_name'
    )

    arg_param_file = launch.actions.DeclareLaunchArgument(
        'param_file',
        default_value=launch.substitutions.PathJoinSubstitution(
            [launch_ros.substitutions.FindPackageShare('cybership_bringup'), 'config', 'any', 'empty.config.yaml']
        ),
        description='Configuration for servo driver node'
    )

    node_thruster_control = launch_ros.actions.Node(
        package='cybership_thrusters',
        executable='cybership_thrusters_node',
        namespace=launch.substitutions.LaunchConfiguration('vessel_name'),
        name='thruster_control_node',
        parameters=[launch.substitutions.LaunchConfiguration('param_file')],
        output='screen'
    )

    ld = launch.LaunchDescription()
    ld.add_action(arg_param_file)
    ld.add_action(arg_vessel_name)
    ld.add_action(node_thruster_control)

    return ld