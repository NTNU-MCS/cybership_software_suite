import launch
import launch.actions
import launch.substitutions
import launch_ros.actions
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    config_cybership_thruster_control = os.path.join(
        get_package_share_directory('cybership_thrusters'),
        'param',
        'config.yaml'
    )

    node_cybership_thruster_control = launch_ros.actions.Node(
        package='cybership_thrusters',
        executable='cybership_thrusters_node',
        name='vessel_control_node',
        parameters=[config_cybership_thruster_control],
        output='screen'
    )

    ld = launch.LaunchDescription()

    ld.add_action(node_cybership_thruster_control)

    return ld