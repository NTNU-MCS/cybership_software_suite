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
            [pkg_cybership_bringup, 'config', 'any', 'mocap_connector.config.yaml']
        ),
        description='Motion Capture System connector configuration file'
    )

    node_mocap_connector = launch_ros.actions.LifecycleNode(
        name='mocap_connector_node',
        namespace='',
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
    ld.add_action(config_param_file)
    ld.add_action(driver_configure_trans_event)
    ld.add_action(driver_activate_trans_event)

    return ld