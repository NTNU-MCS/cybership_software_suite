import launch_ros
import launch
import launch.actions
import launch.substitutions
import launch.launch_description_sources
import launch.launch_description

from cybership_utilities.launch import include_launch_action_with_config

from cybership_utilities.launch import anon
from cybership_utilities.launch import COMMON_ARGUMENTS as ARGUMENTS

def generate_launch_description():


    ld = launch.LaunchDescription()

    for argument in ARGUMENTS:
        ld.add_action(argument)

    ld.add_action(
        include_launch_action_with_config(
            launch.substitutions.LaunchConfiguration("vessel_model"),
            launch.substitutions.LaunchConfiguration("vessel_name"),
            'robot_localization.launch.py', 'robot_localization.yaml'
        )
    )

    ld.add_action(
        include_launch_action_with_config(
            launch.substitutions.LaunchConfiguration("vessel_model"),
            launch.substitutions.LaunchConfiguration("vessel_name"),
            'urdf_description.launch.py'
        )
    )

    return ld
