import launch
import launch.actions
import launch.substitutions
import launch_ros.actions

from cybership_utilities.utilities import anon

from cybership_utilities.launch import COMMON_ARGUMENTS as ARGUMENTS

def generate_launch_description():

    node_dynamixel_control = launch_ros.actions.Node(
        package='dynamixel_servo_ros',
        executable='dynamixel_node',
        namespace=launch.substitutions.LaunchConfiguration('vessel_name'),
        name=f'dynamixel_node_{anon()}',
        parameters=[launch.substitutions.LaunchConfiguration('param_file')],
        output='screen',
        respawn=True,
        respawn_delay=5
    )

    ld = launch.LaunchDescription()

    for arg in ARGUMENTS:
        ld.add_action(arg)

    ld.add_action(node_dynamixel_control)

    return ld