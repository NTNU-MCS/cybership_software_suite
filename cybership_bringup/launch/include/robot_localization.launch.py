import launch
import launch.actions
import launch.substitutions
import launch_ros.actions

from cybership_utilities.launch import anon
from cybership_utilities.launch import COMMON_ARGUMENTS as ARGUMENTS

def generate_launch_description():

    node_robot_localization = launch_ros.actions.Node(
        namespace=launch.substitutions.LaunchConfiguration('vessel_name'),
        package='robot_localization',
        executable='ekf_node',
        name=f'robot_localization_node_{anon()}',
        parameters=[launch.substitutions.LaunchConfiguration('param_file')],
        output='screen',
        respawn=True,
        respawn_delay=5
    )

    ld = launch.LaunchDescription()

    for arg in ARGUMENTS:
        ld.add_action(arg)

    ld.add_action(node_robot_localization)
    return ld