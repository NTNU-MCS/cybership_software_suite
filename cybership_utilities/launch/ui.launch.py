from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    http_port = LaunchConfiguration("http_port")
    ws_port = LaunchConfiguration("ws_port")

    html_dir = PathJoinSubstitution([FindPackageShare("cybership_utilities"), "html"])

    # Simple static file server for the Web UI (serves index2.html, etc.)
    http_server = ExecuteProcess(
        cmd=["python3", "-m", "http.server", http_port],
        cwd=html_dir,
        output="screen",
    )

    # Foxglove WebSocket bridge (ROS 2) for service/topic access from the Web UI
    foxglove = Node(
        package="foxglove_bridge",
        executable="foxglove_bridge",
        name="foxglove_bridge",
        output="screen",
        arguments=["--port", ws_port],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "http_port", default_value="8001", description="Port for the static HTTP server"
            ),
            DeclareLaunchArgument(
                "ws_port", default_value="8765", description="WebSocket port for foxglove_bridge"
            ),
            LogInfo(
                msg=[
                    TextSubstitution(text="Serving UI at http://localhost:"),
                    http_port,
                    TextSubstitution(text="/index.html | Foxglove WS ws://localhost:"),
                    ws_port,
                ]
            ),
            http_server,
            foxglove,
        ]
    )
