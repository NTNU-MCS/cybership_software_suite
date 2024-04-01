#!/usr/bin/env python3

import launch
import launch.actions
import launch.substitutions
import launch_ros.actions
import os
from ament_index_python.packages import get_package_share_directory

import lifecycle_msgs.msg

from launch.actions import SetEnvironmentVariable
from launch_ros.events.lifecycle import ChangeState


def generate_launch_description():

    print('Bringup file for Jonny hasnt implemented yet!')

    # TODO:
    # - mocap driver
    # - servo driver
    # - dynamixel driver

    raise NotImplementedError()
