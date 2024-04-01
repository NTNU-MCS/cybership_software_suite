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

    stdout_linebuf_envvar = launch.actions.SetEnvironmentVariable(
        'RCUTILS_CONSOLE_STDOUT_LINE_BUFFERED', '1')

    config_servo_driver = os.path.join(
        get_package_share_directory('cybership_bringup'),
        'param',
        'enterprise1',
        'servo_driver.yaml'
    )
    node_servo_driver = launch_ros.actions.Node(
        package='ros2_pca9685',
        executable='ros2_pca9685_node',
        name='servo_driver_node',
        parameters=[config_servo_driver],
        output='screen'
    )

    config_mocap_connector = os.path.join(
        get_package_share_directory('cybership_bringup'),
        'param',
        'enterprise1',
        'mocap_connector.yaml'
    )
    node_mocap_connector = launch_ros.actions.LifecycleNode(
        name='mocap_connector_node',
        namespace='',
        package='qualisys_driver',
        executable='qualisys_driver_main',
        output='screen',
        parameters=[config_mocap_connector],
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

    config_mocap_transformer = os.path.join(
        get_package_share_directory('cybership_bringup'),
        'param',
        'enterprise1',
        'mocap_transformer.yaml'
    )
    node_mocap_transformer = launch_ros.actions.Node(
        package='cybership_mocap',
        executable='cybership_mocap_node',
        name='mocap_transformer_node',
        parameters=[config_mocap_transformer],
        output='screen'
    )

    ld = launch.LaunchDescription()
    ld.add_action(stdout_linebuf_envvar)
    ld.add_action(node_servo_driver)
    ld.add_action(node_mocap_connector)
    ld.add_action(node_mocap_transformer)
    ld.add_action(driver_configure_trans_event)
    ld.add_action(driver_activate_trans_event)

    return ld