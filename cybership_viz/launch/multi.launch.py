import launch
import launch_ros

import cybership_utilities.launch

# import os
# from ament_index_python.packages import get_package_share_directory
# from launch import LaunchDescription
# from launch.actions import *
# from launch.substitutions import LaunchConfiguration
# from launch_ros.actions import Node
# from launch.launch_description_sources import *


def generate_launch_description():

    ld = launch.LaunchDescription()

    for argument in cybership_utilities.launch.COMMON_ARGUMENTS:
        ld.add_action(argument)

    subsitution_rviz_config_path = launch.substitutions.PathJoinSubstitution(
        [
            launch_ros.substitutions.FindPackageShare("cybership_viz"),
            "config",
            "multi.rviz",
        ]
    )

    node_rviz = launch_ros.actions.Node(
        package="rviz2",
        executable="rviz2",
        name=f"rviz_{cybership_utilities.launch.anon()}",
        namespace=launch.substitutions.LaunchConfiguration("vessel_name"),
        output="screen",
        arguments=["-d", subsitution_rviz_config_path],
    )

    ld.add_action(node_rviz)

    return ld
