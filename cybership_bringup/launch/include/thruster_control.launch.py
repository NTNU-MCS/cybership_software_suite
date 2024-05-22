import launch
import launch.actions
import launch.substitutions
import launch_ros.actions

from cybership_utilities.utilities import anon
from cybership_utilities.launch import COMMON_ARGUMENTS as ARGUMENTS


def generate_launch_description():

    node_thruster_control = launch_ros.actions.Node(
        namespace=launch.substitutions.LaunchConfiguration('vessel_name'),
        package='cybership_thrusters',
        executable='cybership_thrusters_node',
        name=f'thruster_control_node{anon()}',
        parameters=[launch.substitutions.LaunchConfiguration('param_file')],
        output='screen',
        respawn=True,
        respawn_delay=5
    )

    ld = launch.LaunchDescription()

    for arg in ARGUMENTS:
        ld.add_action(arg)

    ld.add_action(node_thruster_control)

    return ld