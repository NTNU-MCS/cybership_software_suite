import os

import launch
import launch.actions
import launch.conditions
import launch.substitutions
import launch_ros
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def generate_launch_description() -> LaunchDescription:
    ld = launch.LaunchDescription()

    # Launch args
    arg_use_gui = launch.actions.DeclareLaunchArgument(
        'use_gui', default_value='true', description='Launch visualization for both vessels'
    )

    ld.add_action(arg_use_gui)

    # Resolve parameter file path
    sim_share = get_package_share_directory('cybership_simulator')
    multi_yaml_path = os.path.join(
        sim_share, 'config', 'multi_vessel_simulation.yaml')

    # Include robot descriptions for both vessels
    description_launch_voyager = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                launch_ros.substitutions.FindPackageShare(
                    'cybership_description'),
                '/launch/description.launch.py',
            ]
        ),
        launch_arguments=[
            ('vessel_model', 'voyager'),
            ('vessel_name', 'voyager'),
        ],
    )

    description_launch_drillship = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                launch_ros.substitutions.FindPackageShare(
                    'cybership_description'),
                '/launch/description.launch.py',
            ]
        ),
        launch_arguments=[
            ('vessel_model', 'drillship'),
            ('vessel_name', 'drillship'),
        ],
    )

    # Simulator nodes (names fixed to match keys in YAML; topics live under per-vessel namespace)
    voyager_node = Node(
        namespace='voyager',
        package='cybership_simulator',
        executable='voyager.py',
        name='voyager_simulation',
    parameters=[multi_yaml_path]
    )

    drillship_node = Node(
        namespace='drillship',
        package='cybership_simulator',
        executable='drillship.py',
        name='drillship_simulation',
    parameters=[multi_yaml_path]
    )

    # Optional visualization for both (conditioned by use_gui)
    viz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                launch_ros.substitutions.FindPackageShare('cybership_viz'),
                '/launch/multi.launch.py',
            ]
        )
    )

    group_gui = launch.actions.GroupAction(
        condition=launch.conditions.IfCondition(
            launch.substitutions.LaunchConfiguration('use_gui', default='true')),
        actions=[viz],
    )

    # Order: viz (optional) -> descriptions -> simulation nodes
    ld.add_action(group_gui)
    ld.add_action(description_launch_voyager)
    ld.add_action(description_launch_drillship)
    ld.add_action(voyager_node)
    ld.add_action(drillship_node)

    return ld
