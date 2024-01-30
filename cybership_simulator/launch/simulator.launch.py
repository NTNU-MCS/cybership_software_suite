import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import *
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_description_sources import *


PKG_NAME = "cybership_simulator"


def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time', default=True)

    urdf_file_name = "urdf/cybership_enterprise1_base.urdf"
    urdf = os.path.join(
        get_package_share_directory('cybership_description'),
        urdf_file_name)
    with open(urdf, 'r') as infp:
        robot_desc = infp.read()

    # Description launch
    launch_cybership_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory('cybership_description'),
                'launch',
                'description.launch.py'
                )
            ]
        )
    )

    # Visualization launch
    launch_visualization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory('cybership_viz'),
                    'launch',
                    'viz.launch.py'
                )
            ]
        )
    )

    node_simulator = Node(
        package=PKG_NAME,
        executable='cybership_enterprise1_node',
        name='cybership_simulator_node',
        output='screen'
    )

    return LaunchDescription([
        use_sim_time,
        launch_cybership_description,
        node_simulator,
        launch_visualization
    ])
