import launch
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
from cybership_utilities.utilities import anon

def generate_launch_description():

    arg_vessel_name = launch.actions.DeclareLaunchArgument(
        'vessel_name',
        default_value='CSEI',
        description='vessel_name'
    )

    node_joy = Node(
        package='joy',
        executable='joy_node',
        name=f'{anon()}_vessel_joy_node',
        namespace=launch.substitutions.LaunchConfiguration('vessel_name'),
        output='screen'
    )

    # launch.substitutions.LaunchConfiguration('asdf').perform()

    node_topic_relay = Node(
        package='topic_tools',
        executable='relay',
        name=f'{anon()}_joystick_relay_node',
        namespace=launch.substitutions.LaunchConfiguration('vessel_name'),
        output='screen',
        parameters=[
            {
            'input_topic': "/CSEI/joy",
            'output_topic': "/joy"
            }
        ]
    )

    node_joy_teleop = Node(
        package='joy_teleop',
        executable='joy_teleop',
        name=f'{anon()}_vessel_joy_teleop',
        output='screen',
        namespace=launch.substitutions.LaunchConfiguration('vessel_name'),
        parameters=[
            os.path.join(
                get_package_share_directory('cybership_teleop'),
                'config',
                'enterprise1.yaml',
            ),
        ]
    )

    node_vessel_tunnel_thruster = Node(
        package='cybership_teleop',
        executable='cybership_enterprise1_tunnel.py',
        name=f'{anon()}_vessel_tunnel_thruster',
        namespace=launch.substitutions.LaunchConfiguration('vessel_name'),
        output='screen'
    )

    return LaunchDescription([
        arg_vessel_name,
        node_joy,
        node_joy_teleop,
        node_vessel_tunnel_thruster,
        node_topic_relay
    ])