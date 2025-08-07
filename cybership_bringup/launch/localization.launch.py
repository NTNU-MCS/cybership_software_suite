import launch_ros
import launch
import launch.actions
import launch.substitutions
import launch.launch_description_sources


def include_launch_action_with_config(
        vessel_model,
        vessel_name,
        launch_file,
        config_file=""):

    bringup_pkg_dir = launch_ros.substitutions.FindPackageShare(
        'cybership_bringup')
    config_pkg_dir = launch_ros.substitutions.FindPackageShare(
        'cybership_config')

    launch_arguments = [
        ('vessel_name', vessel_name),
        ('vessel_model', vessel_model)
    ]

    if len(config_file) != 0:
        launch_arguments.append(
            (
                'param_file',
                launch.substitutions.PathJoinSubstitution(
                    [config_pkg_dir, 'config', vessel_model, config_file]
                )
            )
        )

    return launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            launch.substitutions.PathJoinSubstitution(
                [bringup_pkg_dir, 'launch', 'include', launch_file]
            )
        ),
        launch_arguments=launch_arguments
    )


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
