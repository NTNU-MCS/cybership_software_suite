import launch
import launch.actions
import launch.substitutions
import launch_ros.actions
import os
from cybership_utilities.utilities import anon
from ament_index_python.packages import get_package_share_directory

COMMON_ARGUMENTS = [
    launch.actions.DeclareLaunchArgument('vessel_model', default_value='any',
                                         choices=['enterprise', 'voyager', 'any'], description='Vessel model'),
    launch.actions.DeclareLaunchArgument('vessel_name', default_value='cybership',
                                         description='Vessel name'),
    launch.actions.DeclareLaunchArgument('param_file', default_value="/dev/null",
                                         description='Path to the parameter file'),
    launch.actions.DeclareLaunchArgument('use_sim_time', default_value='false',
                                         description='Use simulation clock if true'),
]