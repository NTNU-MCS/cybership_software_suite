import launch
import launch.actions
import launch.substitutions
import launch_ros.actions

from cybership_utilities.utilities import anon
from cybership_utilities.launch import COMMON_ARGUMENTS as ARGUMENTS

def generate_launch_description():

    node_mocap_transformer = launch_ros.actions.Node(
        namespace=launch.substitutions.LaunchConfiguration('vessel_name'),
        package='cybership_mocap',
        executable='cybership_mocap_node',
        name=f'mocap_transformer_node_{anon()}',
        parameters=[launch.substitutions.LaunchConfiguration('param_file')],
        output='screen',
        respawn=True,
        respawn_delay=5
    )

    ld = launch.LaunchDescription()

    for arg in ARGUMENTS:
        ld.add_action(arg)

    ld.add_action(node_mocap_transformer)
    return ld