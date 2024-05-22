#!/usr/bin/env python3
#
# This file is part of CyberShip Enterpries Suite.
#
# CyberShip Enterpries Suite software is free software: you can redistribute it
# and/or modify it under the terms of the GNU General Public License as
# published by the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# CyberShip Enterpries Suite is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
# or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
# more details.
#
# You should have received a copy of the GNU General Public License along with
# CyberShip Enterpries Suite. If not, see <https://www.gnu.org/licenses/>.
#
# Maintainer: Emir Cem Gezer
# Email: emir.cem.gezer@ntnu.no, emircem.gezer@gmail.com, me@emircem.com
# Year: 2024
# Copyright (C) 2024 NTNU Marine Cybernetics Laboratory

import uuid

import socket
import re

def get_hostname():
    return socket.gethostname()

def sanitize_hostname_for_ros2(hostname):
    # ROS 2 node names must follow specific rules. Here are the main ones:
    # - Must start with an alphabetical character or an underscore
    # - Must only contain alphanumeric characters and underscores
    # - Must not end with a forward slash or tilde
    # - Must not contain double underscores
    # - Should not exceed 256 characters in length

    # Replace invalid characters with underscores
    sanitized = re.sub(r'[^a-zA-Z0-9_]', '_', hostname)

    # Ensure the name doesn't start with a number or underscore
    if sanitized[0].isdigit() or sanitized[0] == '_':
        sanitized = 'node_' + sanitized

    # Remove trailing underscores
    sanitized = sanitized.rstrip('_')

    # Ensure no double underscores
    sanitized = re.sub(r'__+', '_', sanitized)

    # Truncate to 256 characters if necessary
    if len(sanitized) > 256:
        sanitized = sanitized[:256]

    return sanitized

def anon(host=True):
    hostname = get_hostname()
    ros2_node_name = sanitize_hostname_for_ros2(hostname)
    if host:
        return f"{ros2_node_name}_{str(uuid.uuid4().hex[:12])}"
    else:
        return f"anon_{str(uuid.uuid4().hex[:12])}"

