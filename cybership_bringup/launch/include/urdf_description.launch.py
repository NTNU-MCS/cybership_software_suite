import launch
import launch.actions
import launch.substitutions
import launch_ros.actions

from cybership_utilities.launch import COMMON_ARGUMENTS as ARGUMENTS


def generate_launch_description():
    ld = launch.LaunchDescription()

    for argument in ARGUMENTS:
        ld.add_action(argument)

    include_cs_description = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            launch.substitutions.PathJoinSubstitution(
                [launch_ros.substitutions.FindPackageShare(
                    'cybership_description'), 'launch', 'description.launch.py']
            )
        ),
        launch_arguments=[
            ('vessel_model', launch.substitutions.LaunchConfiguration('vessel_model')),
            ('vessel_name', launch.substitutions.LaunchConfiguration('vessel_name'))
        ]
    )

    ld.add_action(include_cs_description)

    return ld
