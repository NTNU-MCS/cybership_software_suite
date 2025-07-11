import launch
import launch.actions
import launch.substitutions
import launch_ros.actions
from cybership_utilities.launch import anon
from cybership_utilities.launch import COMMON_ARGUMENTS as ARGUMENTS


def generate_launch_description():

    ld = launch.LaunchDescription()

    node_thrust_allocation = launch_ros.actions.Node(
        namespace=launch.substitutions.LaunchConfiguration("vessel_name"),
        package="cybership_dp",
        executable="thrust_allocation_node.py",
        name=f"thrust_allocation_node",
        parameters=[
            {
                "vessel_model": launch.substitutions.LaunchConfiguration(
                    "vessel_model"
                ),
            },
            {"frequency": 20.0, }
        ],
        arguments=[
            "--vessel-model",
            launch.substitutions.LaunchConfiguration("vessel_model"),
            "--vessel-name",
            launch.substitutions.LaunchConfiguration("vessel_name"),
        ],
        remappings=[
            (
                "control/force/command",
                "control/force/command/mux",
            ),  # Remap to the multiplexer output
        ],
        output="screen",
        respawn=True,
        respawn_delay=5,
    )

    # Add topic multiplexer node
    node_topic_mux = launch_ros.actions.Node(
        namespace=launch.substitutions.LaunchConfiguration("vessel_name"),
        package="topic_tools",
        executable="mux",
        name=f"force_mux",
        arguments=[
            "control/force/command/mux",           # Output topic
            "control/force/command",               # Input topic to listen to
            "control/force/command/velocity",      # Input topic to listen to
            "control/force/command/position",      # Input topic to listen to
            "--repeat-delay", "0.1"   # Optional delay for repeated messages
            "--initial_topic", "control/force/command"  # Initial topic to publish
        ],
        output="screen",
        respawn=True,
        respawn_delay=5,
    )

    ld.add_action(node_thrust_allocation)
    ld.add_action(node_topic_mux)

    return ld
