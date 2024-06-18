# Write a ROS2 launch file that launches the `ned_to_enu_transformer` node.
# Remaps the topics `in_pose` and `out_pose` to `/ned_pose` and `/enu_pose` respectively.
# It is in the `cybership_observer` package. Takes frame id as an argument.

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from cybership_utilities.launch import anon

def generate_launch_description():

    node_ned2enu = Node(
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

    return LaunchDescription([node_ned2enu])
