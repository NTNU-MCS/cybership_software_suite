import launch
import launch.actions
import launch.substitutions
import launch_ros.actions

# from ament_index_python.packages import get_package_share_directory

from cybership_utilities.utilities import anon
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
        namespace=launch.substitutions.LaunchConfiguration('vessel_name'),
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name=f'robot_state_publisher_{anon()}',
        output='screen',
        parameters=[
            {'use_sim_time': launch.substitutions.LaunchConfiguration(
                'use_sim_time')},
            {'robot_description': launch.substitutions.Command([
                'xacro', ' ', xacro_file, ' ',
                'gazebo:=ignition', ' ',
                'namespace:=', launch.substitutions.LaunchConfiguration('vessel_name')])}
        ],
        remappings=[
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static')
        ]
    )

    node_static_transform_publisher_world = launch_ros.actions.Node(
        namespace=launch.substitutions.LaunchConfiguration('vessel_name'),
        package='tf2_ros',
        executable='static_transform_publisher',
        name=f'static_transform_publisher_{anon()}',
        output='screen',
        arguments=[
            "--yaw", "1.5707963267948966",
            "--pitch", "3.141592653589793",
            "--roll", "0.0",
            "--frame-id",  "world",
            "--child-frame-id", "world_ned"
        ]
    )

    ld = launch.LaunchDescription()

    for args in ARGUMENTS:
        ld.add_action(args)

    ld.add_action(node_robot_state_publisher)
    ld.add_action(node_static_transform_publisher_world)

    return ld