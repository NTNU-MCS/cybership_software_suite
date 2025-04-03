import launch
import launch_ros
import launch_ros.actions
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

from cybership_utilities.launch import COMMON_ARGUMENTS as ARGUMENTS
from cybership_utilities.launch import anon


def generate_launch_description():
    print(
        """\n
    **************************************
    *  WARNING: THIS LAUNCH FILE IS OLD  *
    *   AND WILL BE REMOVED SOON!        *
    *   PLEASE USE THE NEW LAUNCH FILES: *
    *   - voyager.launch.py              *
    *   - enterprise.launch.py           *
    *   - drillship.launch.py            *
    **************************************
    """
    )
    ld = launch.LaunchDescription()

    for argument in ARGUMENTS:
        ld.add_action(argument)

    description_launch = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            launch.substitutions.PathJoinSubstitution(
                [
                    launch_ros.substitutions.FindPackageShare("cybership_description"),
                    "launch",
                    "description.launch.py",
                ]
            )
        ),
        launch_arguments=[
            ("vessel_model", launch.substitutions.LaunchConfiguration("vessel_model")),
            ("vessel_name", launch.substitutions.LaunchConfiguration("vessel_name")),
        ],
    )

    sim_node = Node(
        namespace=launch.substitutions.LaunchConfiguration("vessel_name"),
        package="cybership_simulator",
        executable="voyager.py",
        parameters=[
            launch.substitutions.PathJoinSubstitution(
                [
                    launch_ros.substitutions.FindPackageShare("cybership_simulator"),
                    "config",
                    "simulation.yaml",
                ]
            )
        ],
        name=f"sim_{anon()}",
    )

    # Include the viz launch file
    viz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                launch_ros.substitutions.FindPackageShare("cybership_viz"),
                "/launch/viz.launch.py",
            ]
        ),
        launch_arguments=[
            ("vessel_model", launch.substitutions.LaunchConfiguration("vessel_model")),
            ("vessel_name", launch.substitutions.LaunchConfiguration("vessel_name")),
        ],
    )

    group_gui = launch.actions.GroupAction(
        condition=launch.conditions.IfCondition(
            launch.substitutions.LaunchConfiguration("use_gui", default="false")
        ),
        actions=[viz_launch],
    )

    # Add the actions to the launch description
    ld.add_action(group_gui)
    ld.add_action(description_launch)
    ld.add_action(sim_node)

    for argument in ARGUMENTS:
        ld.add_action(argument)

    return ld
