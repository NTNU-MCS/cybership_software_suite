import launch
import launch_ros
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from cybership_utilities.launch import COMMON_ARGUMENTS as ARGUMENTS


def generate_launch_description():

    ld = launch.LaunchDescription()

    for argument in ARGUMENTS:
        ld.add_action(argument)

    description_launch = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            launch.substitutions.PathJoinSubstitution(
                [launch_ros.substitutions.FindPackageShare(
                    'cybership_description'), 'launch', 'description.launch.py']
            )
        ),
        launch_arguments=[
            ('vessel_model', 'enterprise'),
            ('vessel_name', 'enterprise')
        ]
    )

    # Define the node
    sim_node = Node(
        package='cybership_simulator',
        executable='cybership_enterprise.py',
        name='sim'
    )

    # Include the viz launch file
    viz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [launch_ros.substitutions.FindPackageShare(
                'cybership_viz'), '/launch/viz.launch.py']
        )
    )

    # Add the actions to the launch description
    ld.add_action(viz_launch)
    ld.add_action(description_launch)
    ld.add_action(sim_node)

    return ld
