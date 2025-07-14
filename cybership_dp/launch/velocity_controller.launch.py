import launch
import launch.actions
import launch.substitutions
import launch_ros.actions
from cybership_utilities.launch import anon
from cybership_utilities.launch import COMMON_ARGUMENTS as ARGUMENTS


def generate_launch_description():

    ld = launch.LaunchDescription()
    arg_param_file = launch.actions.DeclareLaunchArgument(
        name="param_file",
        default_value=launch.substitutions.PathJoinSubstitution(
            [
                launch_ros.substitutions.FindPackageShare("cybership_config"),
                "config",
                launch.substitutions.LaunchConfiguration("vessel_model"),
                "controller_velocity.yaml",
            ]
        ),
    )
    ld.add_action(arg_param_file)

    arguments = list(filter(lambda a: a.name not in ["param_file", "vessel_model"], ARGUMENTS))
    for arg in arguments:
        ld.add_action(arg)

    node_velocity_controller = launch_ros.actions.Node(
        namespace=launch.substitutions.LaunchConfiguration("vessel_name"),
        package="cybership_dp",
        executable="velocity_control_node.py",
        name=f"velocity_controller",
        parameters=[
            launch.substitutions.LaunchConfiguration("param_file"),
        ],
        remappings=[
            (
                "control/velocity/command",
                "control/velocity/command/mux",
            ),  # Remap to the multiplexer output
            (
                "control/force/command",
                "control/force/command/velocity",
            ),  # Remap force command to velocity controller output
        ],
        output="screen",
        respawn=True,
        respawn_delay=5,
    )

    ld.add_action(node_velocity_controller)

    # Add topic multiplexer node for velocity commands
    node_topic_mux = launch_ros.actions.Node(
        namespace=launch.substitutions.LaunchConfiguration("vessel_name"),
        package="topic_tools",
        executable="mux",
        name=f"velocity_mux",
        arguments=[
            "control/velocity/command/mux",           # Output topic
            "control/velocity/command",               # Input topic to listen to
            "--repeat-delay", "0.1",                  # Optional delay for repeated messages
            "--initial-topic", "control/velocity/command"  # Initial topic to publish
        ],
        output="screen",
        respawn=True,
        respawn_delay=5,
    )
    ld.add_action(node_topic_mux)

    return ld
