import launch
import launch.actions
import launch.substitutions
import launch_ros.actions
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    config_cybership_mocap = os.path.join(
        get_package_share_directory('cybership_mocap'),
        'param',
        'config.yaml'
    )

    node_cybership_mocap = launch_ros.actions.Node(
        package='cybership_mocap',
        executable='cybership_mocap_node',
        name='cybership_mocap_node',
        parameters=[config_cybership_mocap],
        output='screen'
    )

    ld = launch.LaunchDescription()

    ld.add_action(node_cybership_mocap)

    return ld