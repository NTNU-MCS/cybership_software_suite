import launch
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
from cybership_utilities.utilities import anon

def generate_launch_description():

    arg_vessel_name = launch.actions.DeclareLaunchArgument(
        'vessel_name',
        default_value='voyager',
        description='vessel_name'
    )

    node_joy = Node(
        namespace=launch.substitutions.LaunchConfiguration('vessel_name'),
        package='joy',
        executable='joy_node',
        name=f'vessel_joy_node_{anon()}',
        output='screen'
    )

    node_joy_teleop = Node(
        namespace=launch.substitutions.LaunchConfiguration('vessel_name'),
        package='joy_teleop',
        executable='joy_teleop',
        name=f'vessel_joy_teleop_{anon()}',
        output='screen',
        parameters=[
            os.path.join(
                get_package_share_directory('cybership_teleop'),
                'config',
                'voyager.yaml',
            ),
        ]
    )

    node_vessel_tunnel_thruster = Node(
        namespace=launch.substitutions.LaunchConfiguration('vessel_name'),
        package='cybership_teleop',
        executable='cybership_voyager_tunnel.py',
        name=f'vessel_tunnel_thruster_{anon()}',
        output='screen'
    )

    return LaunchDescription([
        arg_vessel_name,
        node_joy,
        node_joy_teleop,
        node_vessel_tunnel_thruster,
    ])