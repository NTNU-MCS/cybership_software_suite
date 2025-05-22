from launch import LaunchDescription
import launch_ros.actions
import launch.substitutions
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.descriptions import ParameterValue
import random
import string

from cybership_utilities.launch import anon
from cybership_utilities.launch import COMMON_ARGUMENTS as ARGUMENTS


def generate_launch_description():
    """
    Launch description for a force command multiplexer.
    This multiplexer allows switching between different sources of force commands.
    """
    ld = LaunchDescription()

    for arg in ARGUMENTS:
        ld.add_action(arg)

    # Add topic multiplexer node
    node_topic_mux = launch_ros.actions.Node(
        namespace=launch.substitutions.LaunchConfiguration("vessel_name"),
        package="topic_tools",
        executable="mux",
        name=f"force_mux_{anon()}",
        arguments=[
            "control/force/command",  # Output topic
            "control/force/mux",      # Input topic to listen to
            "control/force/mux/position",  # First input topic
            "control/force/mux/velocity",  # Second input topic
            "--repeat-delay", "0.1"   # Optional delay for repeated messages
        ],
        output="screen",
        respawn=True,
        respawn_delay=5,
    )

    ld.add_action(node_topic_mux)

    return ld
