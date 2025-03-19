import launch
import launch.actions
import launch.substitutions
import launch_ros.actions
import os

import uuid
import socket
import re

import cybership_utilities.utilities

def sanitize_hostname_for_ros2(hostname: str) -> str:
    """ """

    # ROS 2 node names must follow specific rules. Here are the main ones:
    # - Must start with an alphabetical character or an underscore
    # - Must only contain alphanumeric characters and underscores
    # - Must not end with a forward slash or tilde
    # - Must not contain double underscores
    # - Should not exceed 256 characters in length

    # Replace invalid characters with underscores
    sanitized = re.sub(r"[^a-zA-Z0-9_]", "_", hostname)

    # Ensure the name doesn't start with a number or underscore
    if sanitized[0].isdigit() or sanitized[0] == "_":
        sanitized = "node_" + sanitized

    # Remove trailing underscores
    sanitized = sanitized.rstrip("_")

    # Ensure no double underscores
    sanitized = re.sub(r"__+", "_", sanitized)

    # Truncate to 256 characters if necessary
    if len(sanitized) > 256:
        sanitized = sanitized[:256]

    return sanitized


def anon(host: bool = True) -> str:
    """ """
    hostname = socket.gethostname()
    ros2_node_name = sanitize_hostname_for_ros2(hostname)
    if host:
        return f"{ros2_node_name}_{str(uuid.uuid4().hex[:12])}"
    else:
        return f"anon_{str(uuid.uuid4().hex[:12])}"


COMMON_ARGUMENTS = [
    launch.actions.DeclareLaunchArgument(
        "vessel_model",
        default_value="any",
        choices=cybership_utilities.utilities.VESSEL_MODELS,
        description="Vessel model",
    ),
    launch.actions.DeclareLaunchArgument(
        "vessel_name", default_value="cybership", description="Vessel name"
    ),
    launch.actions.DeclareLaunchArgument(
        "param_file",
        default_value="/dev/null",
        description="Path to the parameter file",
    ),
    launch.actions.DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use simulation clock if true",
    ),
]
