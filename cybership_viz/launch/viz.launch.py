import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import *
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_description_sources import *

def generate_launch_description():
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        output='screen',
        arguments=[
            "-d",
            os.path.join(
                get_package_share_directory('cybership_viz'),
                'config',
                'cybership.rviz'
            )
        ]
    )

    # Description launch
    launch_cybership_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory('cybership_description'),
                'launch',
                'description.launch.py'
                )
            ]
        )
    )
    return LaunchDescription([
        rviz_node,
        launch_cybership_description
    ])
