import os
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration

import launch
import launch.actions
import launch.substitutions
import launch_ros.actions
import os
from cybership_utilities.utilities import anon
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    arg_vessel_name = launch.actions.DeclareLaunchArgument(
        'vessel_name',
        default_value='cybership',
        description='vessel_name'
    )

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    arg_use_sim_time = launch.actions.DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation clock if true'
    )

    urdf_file_name = "voyager_base.urdf"
    urdf = os.path.join(
        get_package_share_directory('cybership_description'),
        'urdf',
        urdf_file_name)
    with open(urdf, 'r') as infp:
        robot_desc = infp.read()

    node_robot_state_publisher = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name=f'robot_state_publisher_{anon()}',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': robot_desc
        }],
        arguments=[urdf]
    )

    node_static_transform_publisher_world = launch_ros.actions.Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name=f'static_transform_publisher_{anon()}',
        output='screen',
        arguments=[
            "--yaw", "1.5707963267948966",
            "--pitch", "3.141592653589793",
            "--roll", "0.0",
            "--frame-id",  "world",
            "--child-frame-id", "world_ned"
        ]
    )

    node_static_transform_publisher_base = launch_ros.actions.Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name=f'static_transform_publisher_{anon()}',
        output='screen',
        arguments=[
            "--yaw", "0.0",
            "--pitch", "0.0",
            "--roll", "3.141592653589793",
            "--frame-id",  "base_link_ned",
            "--child-frame-id", "base_link"
        ]
    )
    return LaunchDescription([
        arg_use_sim_time,
        node_robot_state_publisher,
        node_static_transform_publisher_world,
        node_static_transform_publisher_base,
    ])
