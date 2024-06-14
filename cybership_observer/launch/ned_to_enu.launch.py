# Write a ROS2 launch file that launches the `ned_to_enu_transformer` node.
# Remaps the topics `in_pose` and `out_pose` to `/ned_pose` and `/enu_pose` respectively.
# It is in the `cybership_observer` package. Takes frame id as an argument.

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    arg_frame_id = (DeclareLaunchArgument("frame_id", default_value="world"),)

    node_ned2enu = Node(
        package="cybership_observer",
        executable="ned_to_enu_transformer.py",
        name="ned_to_enu_transformer",
        remappings=[
            ("in_pose", "/voyager/measurement/pose"),
            ("out_pose", "/voyager/measurement/pose/enu"),
        ],
        parameters=[{"frame_id": LaunchConfiguration("frame_id")}],
    )

    return LaunchDescription([arg_frame_id, node_ned2enu])
