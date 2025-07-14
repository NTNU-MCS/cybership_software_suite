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
                    "thrust_allocation.launch.py",
                ]
            )
        ),
        launch_arguments=[
            ("vessel_name", launch.substitutions.LaunchConfiguration("vessel_name")),
            ("vessel_model", launch.substitutions.LaunchConfiguration("vessel_model")),
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
        ],
    )

    include_position_controller = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            launch.substitutions.PathJoinSubstitution(
                [
                    launch_ros.substitutions.FindPackageShare("cybership_dp"),
                    "launch",
                    "position_controller.launch.py",
                ]
            )
        ),
        launch_arguments=[
            ("vessel_name", launch.substitutions.LaunchConfiguration("vessel_name")),
            ("vessel_model", launch.substitutions.LaunchConfiguration("vessel_model")),
        ],
    )

    # Add service call to switch force multiplexer to velocity controller output
    # This will switch the force_mux to use control/force/command/velocity
    service_call_switch_mux = launch.actions.ExecuteProcess(
        cmd=[
            'ros2', 'service', 'call',
            [launch.substitutions.LaunchConfiguration("vessel_name"), '/force_mux/select'],
            'topic_tools_interfaces/srv/MuxSelect',
            '{topic: "control/force/command/position"}'
        ],
        output='screen',
    )
    ld.add_action(service_call_switch_mux)

    ld.add_action(include_force_controller)
    ld.add_action(include_velocity_controller)
    ld.add_action(include_position_controller)

    return ld
