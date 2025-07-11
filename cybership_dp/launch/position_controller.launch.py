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
                "controller_position.yaml",
            ]
        ),
    )
    ld.add_action(arg_param_file)

    arguments = list(filter(lambda a: a.name not in ["param_file", "vessel_model"], ARGUMENTS))
    for arg in arguments:
        ld.add_action(arg)

    node_position_controller = launch_ros.actions.Node(
        namespace=launch.substitutions.LaunchConfiguration("vessel_name"),
        package="cybership_dp",
        executable="position_control_node.py",
        name=f"position_controller",
        parameters=[
            launch.substitutions.LaunchConfiguration("param_file"),
            {
                "vessel_model": launch.substitutions.LaunchConfiguration(
                    "vessel_model"
                ),
            },
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
                "control/force/command/position",
            ),  # Remap force command to position controller output
        ],
        output="screen",
        respawn=True,
        respawn_delay=5,
    )

    ld.add_action(node_position_controller)

    # Add argument to control whether to auto-switch the mux
    arg_auto_switch = launch.actions.DeclareLaunchArgument(
        name="auto_switch_mux",
        default_value="true",
        description="Automatically switch force mux to velocity controller output",
    )
    ld.add_action(arg_auto_switch)



    return ld
