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
                launch_ros.substitutions.FindPackageShare("cybership_dp"),
                "config",
                "velocity_controller.yaml",
            ]
        ),
    )
    ld.add_action(arg_param_file)

    arguments = list(filter(lambda a: a.name != "param_file", ARGUMENTS))
    for arg in arguments:
        ld.add_action(arg)

    node_force_controller = launch_ros.actions.Node(
        namespace=launch.substitutions.LaunchConfiguration("vessel_name"),
        package="cybership_dp",
        executable="velocity_controller.py",
        name=f"velocity_controller_{anon()}",
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
        output="screen",
        respawn=True,
        respawn_delay=5,
    )

    ld.add_action(node_force_controller)

    return ld
