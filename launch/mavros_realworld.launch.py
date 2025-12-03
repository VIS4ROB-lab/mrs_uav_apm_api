#!/usr/bin/env python3

import launch
import os
import launch_ros

from launch_ros.actions import Node, PushROSNamespace, SetParameter
from launch.actions import GroupAction, IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import (
    LaunchConfiguration,
    IfElseSubstitution,
    PythonExpression,
    PathJoinSubstitution,
    EnvironmentVariable,
)
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    ld = launch.LaunchDescription()

    pkg_name = "mrs_uav_apm_api"

    this_pkg_path = get_package_share_directory(pkg_name)

    uav_name = os.getenv("UAV_NAME", "uav")

    fcu_url = "/dev/ttyACM0:57600"
    gcs_url = ""

    use_sim_time = False
    respawn_mavros = False

    tgt_system = 1
    namespace = uav_name

    apm_launch_arguments = {
        "fcu_url": fcu_url,
        "gcs_url": gcs_url,
        "tgt_system": str(tgt_system),
        "tgt_component": str(1),
        "log_output": "screen",
        "fcu_protocol": "v2.0",
        "respawn_mavros": str(respawn_mavros),
        "namespace": "mavros",
        "pluginlists_yaml":  this_pkg_path + "/config/mavros_plugins.yaml",
        "config_yaml": this_pkg_path + "/config/mavros_apm_config.yaml",
    }

    print(apm_launch_arguments.items())

    launch_xml_include_with_namespace = GroupAction(
        actions=[
            PushROSNamespace(uav_name),
            IncludeLaunchDescription(
                XMLLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory('mrs_uav_apm_api'),
                        'launch/mavros.launch')
                    ),
                    launch_arguments=apm_launch_arguments.items()
            ),
        ],
    )

    ld.add_action(launch_xml_include_with_namespace)

    # ld.add_action(
    #     launch_ros.actions.Node(
    #         package='tf2_ros',
    #         namespace='',
    #         executable='static_transform_publisher',
    #         name='fcu_to_garmin',
    #         arguments=["0.0", "0.0", "-0.05", "0", "1.57", "0", uav_name+"/fcu", "garmin"],
    #     )
    # )

    return ld


