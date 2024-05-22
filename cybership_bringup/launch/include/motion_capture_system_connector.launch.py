import launch
import launch.actions
import launch.substitutions
import launch_ros.actions

import lifecycle_msgs.msg

from launch.actions import SetEnvironmentVariable
from launch_ros.events.lifecycle import ChangeState

from cybership_utilities.utilities import anon
from cybership_utilities.launch import COMMON_ARGUMENTS as ARGUMENTS

def generate_launch_description():

    node_mocap_connector = launch_ros.actions.LifecycleNode(
        name=f'mocap_connector_node_{anon()}',
        namespace=launch.substitutions.LaunchConfiguration('vessel_name'),
        package='qualisys_driver',
        executable='qualisys_driver_main',
        output='screen',
        parameters=[launch.substitutions.LaunchConfiguration('param_file')],
        respawn=True,
        respawn_delay=5
    )

    driver_configure_trans_event = launch.actions.EmitEvent(
        event=ChangeState(
          lifecycle_node_matcher=launch.events.matchers.matches_action(node_mocap_connector),
          transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,
        )
    )

    driver_activate_trans_event = launch.actions.EmitEvent(
       event=launch_ros.events.lifecycle.ChangeState(
          lifecycle_node_matcher=launch.events.matchers.matches_action(node_mocap_connector),
          transition_id=lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE,
        )
    )


    ld = launch.LaunchDescription()

    for arg in ARGUMENTS:
        ld.add_action(arg)

    ld.add_action(node_mocap_connector)
    ld.add_action(driver_configure_trans_event)
    ld.add_action(driver_activate_trans_event)

    return ld