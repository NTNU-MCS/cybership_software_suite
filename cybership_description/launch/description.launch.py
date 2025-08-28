import launch
import launch.actions
import launch.substitutions
import launch_ros.actions

# from ament_index_python.packages import get_package_share_directory

from cybership_utilities.launch import anon
from cybership_utilities.launch import COMMON_ARGUMENTS as ARGUMENTS


def generate_launch_description():

    xacro_file = launch.substitutions.PathJoinSubstitution([
        launch_ros.substitutions.FindPackageShare('cybership_description'),
        'urdf',
        'model',
        launch.substitutions.LaunchConfiguration('vessel_model'),
        'base.urdf.xacro']
    )

    node_robot_state_publisher = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name=f'robot_state_publisher_{anon()}',
        namespace=launch.substitutions.LaunchConfiguration('vessel_name'),
        output='screen',
        parameters=[
            {'use_sim_time': launch.substitutions.LaunchConfiguration(
                'use_sim_time')},
            {'robot_description': launch.substitutions.Command([
                'xacro', ' ', xacro_file, ' ',
                'gazebo:=ignition', ' ',
                'namespace:=', launch.substitutions.LaunchConfiguration('vessel_name')])},
            {'frame_prefix': [launch.substitutions.LaunchConfiguration('vessel_name'), '/']},
        ],
        remappings=[
            # ('/tf', 'tf'),
            # ('/tf_static', 'tf_static'),
        ]
    )

    ld = launch.LaunchDescription()

    for args in ARGUMENTS:
        ld.add_action(args)

    ld.add_action(node_robot_state_publisher)

    return ld