import launch
import launch.actions
import launch.substitutions
import launch.launch_description_sources
import launch_ros.actions

from cybership_utilities.launch import anon
from cybership_utilities.launch import COMMON_ARGUMENTS as ARGUMENTS


def generate_launch_description():

    ld = launch.LaunchDescription()

    include_force_controller = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            launch.substitutions.PathJoinSubstitution(
                [
                    launch_ros.substitutions.FindPackageShare("cybership_dp"),
                    "launch",
                    "force_controller.launch.py",
                ]
            )
        ),
        launch_arguments=[
            ("vessel_name", launch.substitutions.LaunchConfiguration("vessel_name")),
            ("vessel_model", launch.substitutions.LaunchConfiguration("vessel_model")),
            (
                "param_file",
                launch.substitutions.PathJoinSubstitution(
                    [
                        launch_ros.substitutions.FindPackageShare("cybership_dp"),
                        "config",
                        "force_controller.yaml",
                    ]
                ),
            ),
        ],
    )

    include_velocity_controller = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            launch.substitutions.PathJoinSubstitution(
                [
                    launch_ros.substitutions.FindPackageShare("cybership_dp"),
                    "launch",
                    "velocity_controller.launch.py",
                ]
            )
        ),
        launch_arguments=[
            ("vessel_name", launch.substitutions.LaunchConfiguration("vessel_name")),
            ("vessel_model", launch.substitutions.LaunchConfiguration("vessel_model")),
            (
                "param_file",
                launch.substitutions.PathJoinSubstitution(
                    [
                        launch_ros.substitutions.FindPackageShare("cybership_dp"),
                        "config",
                        "velocity_controller.yaml",
                    ]
                ),
            ),
        ],
    )

    ld.add_action(include_force_controller)
    ld.add_action(include_velocity_controller)

    return ld
