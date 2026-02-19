#!/usr/bin/env python3
"""
MAVROS launch file for APM SITL (Software-In-The-Loop) simulation.

This launch file configures MAVROS to connect to APM SITL via UDP,
unlike the hardware version which uses serial ports.
"""

import launch
import os
import launch_ros
import tempfile
from jinja2 import Environment, FileSystemLoader

from launch_ros.actions import Node, SetParameter
from launch.actions import GroupAction, IncludeLaunchDescription, DeclareLaunchArgument

from launch_xml.launch_description_sources import XMLLaunchDescriptionSource

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    """Generate launch description for MAVROS with APM SITL."""

    ld = launch.LaunchDescription()

    pkg_name = "mrs_uav_apm_api"
    this_pkg_path = get_package_share_directory(pkg_name)

    uav_name = os.getenv("UAV_NAME", "uav1")
    uav_id = os.getenv("UAV_ID", "1")
    use_sim_time = os.getenv('USE_SIM_TIME', "true") == "true"
    respawn_mavros = os.getenv('respawn_mavros', "true") == "true"

    fcu_url = "udp://127.0.0.1:14540@127.0.0.1:14550"
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
        "pluginlists_yaml": this_pkg_path + "/config/mavros_plugins.yaml",
        "config_yaml": config_yaml_path,
        "use_sim_time": str(use_sim_time),
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

    return ld
