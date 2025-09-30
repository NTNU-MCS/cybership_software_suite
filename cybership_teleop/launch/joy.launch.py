import launch
import launch_ros

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
from cybership_utilities.launch import anon
from cybership_utilities.launch import COMMON_ARGUMENTS as ARGUMENTS


def generate_launch_description():

    ld = LaunchDescription()

    for argument in ARGUMENTS:
        ld.add_action(argument)



    arg_joy_id = launch.actions.DeclareLaunchArgument(
        "joy_id",
        default_value="0",
        description="Joystick id",
    )

    node_joy = Node(
        namespace=launch.substitutions.LaunchConfiguration("vessel_name"),
        package="joy",
        executable="game_controller_node",
        name=f"vessel_joy_node_{anon()}",
        parameters=[
            {"device_id": launch.substitutions.LaunchConfiguration("joy_id")},
            {"deadzone": 0.1},
            {"autorepeat_rate": 20.0},
            {"coalesce_interval": 0.01},
        ],
        output="screen",
    )


    ld.add_action(arg_joy_id)
    ld.add_action(node_joy)

    return ld
