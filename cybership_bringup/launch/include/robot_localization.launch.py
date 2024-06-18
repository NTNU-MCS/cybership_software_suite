import launch
import launch.actions
import launch.substitutions
import launch_ros.actions

from cybership_utilities.launch import anon
from cybership_utilities.launch import COMMON_ARGUMENTS as ARGUMENTS

def generate_launch_description():

    ld = launch.LaunchDescription()

    node_robot_localization = launch_ros.actions.Node(
        namespace=launch.substitutions.LaunchConfiguration('vessel_name'),
        package='robot_localization',
        executable='ekf_node',
        name=f'robot_localization_node_{anon()}',
        parameters=[launch.substitutions.LaunchConfiguration('param_file')],
        output='screen',
        respawn=True,
        respawn_delay=5,
        remappings=[
            ('odometry/filtered', 'measurement/odom'),
        ]
    )

    observer_pkg_dir = launch_ros.substitutions.FindPackageShare(
        'cybership_observer')

    ned_world_include = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            launch.substitutions.PathJoinSubstitution(
                [observer_pkg_dir, 'launch', 'ned_world_tf.launch.py']
            )
        ),
    )
    ld.add_action(ned_world_include)


    for arg in ARGUMENTS:
        ld.add_action(arg)

    ld.add_action(node_robot_localization)
    return ld