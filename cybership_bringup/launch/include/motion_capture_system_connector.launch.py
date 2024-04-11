#!/usr/bin/env python3

import launch
import launch.actions
import launch.substitutions
import launch_ros.actions

import lifecycle_msgs.msg

from launch.actions import SetEnvironmentVariable
from launch_ros.events.lifecycle import ChangeState


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
        description='Motion Capture System connector configuration file'
    )

    node_mocap_connector = launch_ros.actions.LifecycleNode(
        name='mocap_connector_node',
        namespace=launch.substitutions.LaunchConfiguration('vessel_name'),
        package='qualisys_driver',
        executable='qualisys_driver_main',
        output='screen',
        parameters=[launch.substitutions.LaunchConfiguration('param_file')],
    )

    driver_configure_trans_event = launch.actions.EmitEvent(
        event=ChangeState(
          lifecycle_node_matcher=launch.events.matchers.matches_action(node_mocap_connector),
          transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,
        )
    )

    driver_activate_trans_event = launch.actions.EmitEvent(
       event=launch_ros.events.lifecycle.ChangeState(
          lifecycle_node_matcher=launch.events.matchers.matches_action(node_mocap_connector),
          transition_id=lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE,
        )
    )


    ld = launch.LaunchDescription()
    ld.add_action(node_mocap_connector)
    ld.add_action(arg_param_file)
    ld.add_action(arg_vessel_name)
    ld.add_action(driver_configure_trans_event)
    ld.add_action(driver_activate_trans_event)

    return ld