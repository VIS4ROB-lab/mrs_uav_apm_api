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

    # launch.logging.get_logger().setLevel(launch.logging.logging.DEBUG)

    ld = launch.LaunchDescription()

    pkg_name = "mrs_uav_px4_api"

    this_pkg_path = get_package_share_directory(pkg_name)

    # #{ args from ENV

    uav_name = os.getenv("UAV_NAME", "uav")
    OLD_PX4_FW = os.getenv("OLD_PX4_FW", "false") == "true"

    if OLD_PX4_FW:
      fcu_url = "/dev/pixhawk:921600"
    else:
      fcu_url = "/dev/pixhawk:2000000"

    gcs_url = "tcp-l://"
    use_sim_time=os.getenv('USE_SIM_TIME', "false") == "true"
    respawn_mavros=os.getenv('respawn_mavros', "false") == "true"

    # #} end of args from ENV

    # the first one has the priority
    # configs = [
    #     this_pkg_path + '/config/mavros_px4_config.yaml',
    #     get_package_share_directory("mavros") + "/launch/px4_config.yaml",
    # ]

    tgt_system = 1
    namespace = uav_name

    px4_launch_arguments = {
        "fcu_url": fcu_url,
        "gcs_url": gcs_url,
        "tgt_system": str(tgt_system),
        "tgt_component": str(1),
        "log_output": "screen",
        "fcu_protocol": "v2.0",
        "respawn_mavros": str(respawn_mavros),
        "namespace": "mavros",
        "pluginlists_yaml":  this_pkg_path + "/config/mavros_plugins.yaml",
        "config_yaml": this_pkg_path + "/config/mavros_px4_config_old_fw.yaml" if OLD_PX4_FW else this_pkg_path + "/config/mavros_px4_config.yaml",
    }

    print(px4_launch_arguments.items())

    launch_xml_include_with_namespace = GroupAction(
        actions=[
            # push_ros_namespace first to set namespace of included nodes for following actions
            PushROSNamespace(uav_name),
            IncludeLaunchDescription(
                XMLLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory('mrs_uav_px4_api'),
                        'launch/mavros.launch')
                    ),
                    launch_arguments=px4_launch_arguments.items()
            ),
        ],
        # parameters=[
        #     {"config/garmin/frame_id": uav_name + "/garmin"},
        # ],
        # parameters=[
        #     {"fcu_url": fcu_url},
        #     {"gcs_url": gcs_url},
        #     {"tgt_system": tgt_system},
        #     {"tgt_component": 1},
        #     {"log_output": "screen"},
        #     {"fcu_protocol": "v2.0"},
        #     {"respawn_mavros": respawn_mavros},
        #     {"namespace": f"{namespace}/mavros"},
        #     {"pluginlists_yaml": get_package_share_directory("mavros") + "/launch/px4_pluginlists.yaml"},
        #     {"config_yaml": configs},
        # ],
    )

    ld.add_action(launch_xml_include_with_namespace)

    ld.add_action(
        # Nodes under test
        launch_ros.actions.Node(
            package='tf2_ros',
            namespace='',
            executable='static_transform_publisher',
            name='fcu_to_garmin',
            arguments=["0.0", "0.0", "-0.05", "0", "1.57", "0", uav_name+"/fcu", "garmin"],
        )
    )

    # ld.add_action(

    #     Node(
    #         namespace=namespace,
    #         name='mavros',
    #         package='mavros',
    #         executable='mavros_node',

    #         parameters=[
    #             {"fcu_url": fcu_url},
    #             {"gcs_url": gcs_url},
    #             {"tgt_system": tgt_system},
    #             {"tgt_component": 1},
    #             {"log_output": "screen"},
    #             {"fcu_protocol": "v2.0"},
    #             {"respawn_mavros": respawn_mavros},
    #             {"namespace": f"{namespace}/mavros"},
    #             {"pluginlists_yaml": get_package_share_directory("mavros") + "/launch/px4_pluginlists.yaml"},
    #             {"config_yaml": configs},
    #        ],

    #        remappings=[
    #            ("/diagnostics", f"/uav{ID}/diagnostics")
    #         ],
    #     )
    # )

    return ld


