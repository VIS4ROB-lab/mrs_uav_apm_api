#!/usr/bin/env python3

import launch
import os
import launch_ros
import tempfile
from jinja2 import Environment, FileSystemLoader

from launch_ros.actions import Node, SetParameter
from launch.actions import GroupAction, IncludeLaunchDescription, DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration

from launch_xml.launch_description_sources import XMLLaunchDescriptionSource

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    ld = launch.LaunchDescription()

    pkg_name = "mrs_uav_apm_api"
    this_pkg_path = get_package_share_directory(pkg_name)

    uav_name = os.getenv("UAV_NAME", "uav")
    uav_id = os.getenv("UAV_ID", "1")
    use_sim_time=os.getenv('USE_SIM_TIME', "false") == "true"
    respawn_mavros=os.getenv('respawn_mavros', "false") == "true"

    # Declare fcu_url as a launch argument
    ld.add_action(DeclareLaunchArgument(
        'fcu_url',
        default_value="/dev/pixhawk:921600",
        description='FCU connection URL (e.g., /dev/pixhawk:921600)'
    ))

    fcu_url = LaunchConfiguration('fcu_url')
    gcs_url = "tcp-l://"

    tgt_system = int(uav_id)
    
    # Process Jinja template for config
    config_dir = os.path.join(this_pkg_path, "config")
    env = Environment(loader=FileSystemLoader(config_dir))
    template = env.get_template("mavros_apm_config.jinja.yaml")
    
    template_vars = {
        "uav_name": uav_name,
        "uav_id": uav_id,
    }
    
    rendered_config = template.render(template_vars)
    
    # Write rendered config to temporary file
    with tempfile.NamedTemporaryFile(mode='w', suffix='.yaml', delete=False) as f:
        f.write(rendered_config)
        config_yaml_path = f.name
    
    apm_launch_arguments = {
        "fcu_url": fcu_url,
        "gcs_url": gcs_url,
        "tgt_system": str(tgt_system),
        "tgt_component": str(1),
        "log_output": "screen",
        "fcu_protocol": "v2.0",
        "respawn_mavros": str(respawn_mavros),
        "namespace": uav_name + "/mavros",
        "pluginlists_yaml":  this_pkg_path + "/config/mavros_plugins.yaml",
        "config_yaml": config_yaml_path,
        "base_link_frame_id": uav_name + "/base_link",
        "odom_frame_id": uav_name + "/odom",
        "map_frame_id": uav_name + "/map",
    }

    ld.add_action(
        IncludeLaunchDescription(
            XMLLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('mrs_uav_apm_api'),
                    'launch/mavros.launch')
                ),
                launch_arguments=apm_launch_arguments.items()
        )
    )

    # Delay stream enforcer so MAVROS is fully connected
    ld.add_action(
        TimerAction(
            period=5.0,
            actions=[
                Node(
                    package='mrs_uav_apm_api',
                    executable='mavros_stream_enforcer',
                    namespace=uav_name,
                    name='mavros_stream_enforcer',
                    output='screen',
                )
            ]
        )       
    )

    # Delay GP origin setter so MAVROS is fully connected and has a GPS fix (if available)
    ld.add_action(
        Node(
            package='mrs_uav_apm_api',
            executable='auto_set_home.py',
            namespace=uav_name,
            name='auto_set_home',
            output='screen',
        )   
    )

    return ld


