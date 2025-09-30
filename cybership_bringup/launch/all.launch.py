import launch
import launch.actions
import launch.conditions
import launch.launch_description_sources
import launch.substitutions
import launch_ros.substitutions


def generate_launch_description():

    ld = launch.LaunchDescription()

    vessel_model = launch.substitutions.LaunchConfiguration("vessel_model")
    vessel_name = vessel_model

    ld.add_action(
        launch.actions.DeclareLaunchArgument(
            "vessel_model",
            description="Vessel model to bring up (drillship|enterprise|voyager)",
        )
    )

    include_drillship = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            launch.substitutions.PathJoinSubstitution(
                [
                    launch_ros.substitutions.FindPackageShare("cybership_bringup"),
                    "launch",
                    "drillship.launch.py",
                ]
            )
        ),
        condition=launch.conditions.IfCondition(
            launch.substitutions.PythonExpression(["'", vessel_model, "' == 'drillship'"])
        ),
    )

    include_enterprise = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            launch.substitutions.PathJoinSubstitution(
                [
                    launch_ros.substitutions.FindPackageShare("cybership_bringup"),
                    "launch",
                    "enterprise.launch.py",
                ]
            )
        ),
        condition=launch.conditions.IfCondition(
            launch.substitutions.PythonExpression(["'", vessel_model, "' == 'enterprise'"])
        ),
    )

    include_voyager = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            launch.substitutions.PathJoinSubstitution(
                [
                    launch_ros.substitutions.FindPackageShare("cybership_bringup"),
                    "launch",
                    "voyager.launch.py",
                ]
            )
        ),
        condition=launch.conditions.IfCondition(
            launch.substitutions.PythonExpression(["'", vessel_model, "' == 'voyager'"])
        ),
    )

    include_localization = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            launch.substitutions.PathJoinSubstitution(
                [
                    launch_ros.substitutions.FindPackageShare("cybership_bringup"),
                    "launch",
                    "localization.launch.py",
                ]
            )
        ),
        launch_arguments={
            "vessel_model": vessel_model,
            "vessel_name": vessel_name,
        }.items(),
    )

    include_dp = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            launch.substitutions.PathJoinSubstitution(
                [
                    launch_ros.substitutions.FindPackageShare("cybership_dp"),
                    "launch",
                    "dp.launch.py",
                ]
            )
        ),
        launch_arguments={
            "vessel_model": vessel_model,
            "vessel_name": vessel_name,
        }.items(),
    )

    ld.add_action(include_drillship)
    ld.add_action(include_enterprise)
    ld.add_action(include_voyager)
    ld.add_action(include_localization)
    ld.add_action(include_dp)

    return ld
