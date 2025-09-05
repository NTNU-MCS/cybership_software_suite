import launch_ros
import launch
import launch.actions
import launch.substitutions
import launch.launch_description_sources

from cybership_utilities.launch import include_launch_action_with_config



def generate_launch_description():

    vessel_name = 'voyager'

    vessel_model = 'voyager'

    ld = launch.LaunchDescription()

    ld.add_action(
        include_launch_action_with_config(
            vessel_model, vessel_name,
            'robot_localization.launch.py', 'robot_localization.yaml'
        )
    )

    ld.add_action(
        include_launch_action_with_config(
            vessel_model, vessel_name,
            'urdf_description.launch.py'
        )
    )

    return ld
