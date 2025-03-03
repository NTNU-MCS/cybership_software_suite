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

    arg_joy_config = launch.actions.DeclareLaunchArgument(
        "joy_config",
        default_value=launch.substitutions.PathJoinSubstitution(
            [
                launch_ros.substitutions.FindPackageShare("cybership_teleop"),
                "config",
                "simple",
                "ps5.yaml",
            ]
        ),
        description="Joystick configuration file",
    )

    node_joy = Node(
        namespace=launch.substitutions.LaunchConfiguration("vessel_name"),
        package="joy",
        executable="joy_node",
        name=f"vessel_joy_node_{anon()}",
        output="screen",
    )

    node_joy_teleop = Node(
        namespace=launch.substitutions.LaunchConfiguration("vessel_name"),
        package="joy_teleop",
        executable="joy_teleop",
        name=f"vessel_joy_teleop_{anon()}",
        output="screen",
        parameters=[launch.substitutions.LaunchConfiguration("joy_config")],
    )

    node_vessel_tunnel_thruster = Node(
        namespace=launch.substitutions.LaunchConfiguration("vessel_name"),
        package="cybership_teleop",
        executable="tunnel.py",
        name=f"vessel_tunnel_thruster_{anon()}",
        output="screen",
        parameters=[{"axis1": 5, "axis2": 2}],
    )

    ld.add_action(arg_joy_config)
    ld.add_action(node_joy)
    ld.add_action(node_joy_teleop)
    ld.add_action(node_vessel_tunnel_thruster)

    return ld
