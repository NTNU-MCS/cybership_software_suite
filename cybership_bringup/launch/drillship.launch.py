import launch_ros
import launch
import launch.actions
import launch.substitutions
import launch.launch_description_sources


def include_launch_action_with_config(
        vessel_model,
        vessel_name,
        launch_file,
        param_file=""):

    bringup_pkg_dir = launch_ros.substitutions.FindPackageShare(
        'cybership_bringup')
    config_pkg_dir = launch_ros.substitutions.FindPackageShare(
        'cybership_config')

    launch_arguments = [
        ('vessel_name', vessel_name),
        ('vessel_model', vessel_model)
    ]

    if len(param_file) != 0:
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

    vessel_name = 'drillship'

    vessel_model = 'drillship'

    ld = launch.LaunchDescription()

    ld.add_action(
        include_launch_action_with_config(
            vessel_model, vessel_name,
            'azimuth_controller.launch.py', 'azimuth_controller.yaml'
        )
    )

    ld.add_action(
        include_launch_action_with_config(
            vessel_model, vessel_name,
            'motion_capture_system_connector.launch.py', 'mocap_connector.yaml'
        )
    )

    ld.add_action(
        include_launch_action_with_config(
            vessel_model, vessel_name,
            'motion_capture_system_transformer.launch.py', 'mocap_transformer.yaml'
        )
    )

    # ld.add_action(
    #     include_launch_action_with_config(
    #         vessel_model, vessel_name,
    #         'robot_localization.launch.py', 'robot_localization.yaml'
    #     )
    # )

    ld.add_action(
        include_launch_action_with_config(
            vessel_model, vessel_name,
            'servo_driver.launch.py', 'servo_driver.yaml'
        )
    )

    ld.add_action(
        include_launch_action_with_config(
            vessel_model, vessel_name,
            'thruster_control.launch.py', 'thruster_control.yaml'
        )
    )

    ld.add_action(
        include_launch_action_with_config(
            vessel_model, vessel_name,
            'imu_bno055.launch.py', 'imu_bno055.yaml'
        )
    )

    # ld.add_action(
    #     include_launch_action_with_config(
    #         vessel_model, vessel_name,
    #         'urdf_description.launch.py', 'empty.yaml'
    #     )
    # )

    return ld
