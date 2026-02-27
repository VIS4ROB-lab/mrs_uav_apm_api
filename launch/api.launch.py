#!/usr/bin/env python3

import launch
import os

from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
from launch.actions import DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription, GroupAction, SetEnvironmentVariable
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import (
    LaunchConfiguration,
    IfElseSubstitution,
    PythonExpression,
    PathJoinSubstitution,
    EnvironmentVariable,
)

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    ld = launch.LaunchDescription()

    pkg_name = "mrs_uav_apm_api"

    this_pkg_path = get_package_share_directory(pkg_name)

    # #{ custom_config

    custom_config = LaunchConfiguration('custom_config')

    # this adds the args to the list of args available for this launch files
    # these args can be listed at runtime using -s flag
    # default_value is required to if the arg is supposed to be optional at launch time
    ld.add_action(DeclareLaunchArgument(
        'custom_config',
        default_value="",
        description="Path to the custom configuration file. The path can be absolute, starting with '/' or relative to the current working directory",
    ))

    # behaviour:
    #     custom_config == "" => custom_config: ""
    #     custom_config == "/<path>" => custom_config: "/<path>"
    #     custom_config == "<path>" => custom_config: "$(pwd)/<path>"
    custom_config = IfElseSubstitution(
            condition=PythonExpression(['"', custom_config, '" != "" and ', 'not "', custom_config, '".startswith("/")']),
            if_value=PathJoinSubstitution([EnvironmentVariable('PWD'), custom_config]),
            else_value=custom_config
    )

    # #} end of custom_config

    # #{ simulation

    simulation = LaunchConfiguration('simulation')

    declare_simulation = DeclareLaunchArgument(
        'simulation',
        default_value="true" if os.getenv('RUN_TYPE', "simulation") == "simulation" else "false",
        description='Whether to start a as a simulation or load into an existing container.'
    )

    ld.add_action(declare_simulation)

    # #} end of simulation

    # #{ fcu_url

    fcu_url = LaunchConfiguration('fcu_url')

    ld.add_action(DeclareLaunchArgument(
        'fcu_url',
        default_value="/dev/pixhawk:921600",
        description='FCU connection URL (e.g., /dev/pixhawk:921600)'
    ))

    # #} end of fcu_url

    # #{ args from ENV

    uav_name=os.getenv('UAV_NAME', "uav1")
    use_sim_time=os.getenv('USE_SIM_TIME', "false") == "true"

    # #} end of args from ENV
    
    # the first one has the priority
    configs = [
        this_pkg_path + '/config/apm_api.yaml',
        get_package_share_directory("mrs_uav_hw_api") + "/config/hw_api.yaml",
    ]

    namespace = uav_name

    ld.add_action(ComposableNodeContainer(

        namespace=namespace,
        name=namespace+'_container',
        package='rclcpp_components',
        executable='component_container_mt',
        output="screen",

        composable_node_descriptions=[

            ComposableNode(

                package="mrs_uav_hw_api",
                plugin="mrs_uav_hw_api::HwApiManager",
                namespace=uav_name,
                name="hw_api",
                parameters=[

                    {"uav_name": uav_name},
                    {"topic_prefix": "/" + uav_name},
                    {"use_sim_time": use_sim_time},
                    {"configs": configs},
                    {"custom_config": custom_config},
                    {"simulation": simulation},

                ],

                remappings=[

                  ("~/ground_truth_in", "ground_truth" if simulation else "rtk/bestpos"),
                  ("~/mavros_state_in", "mavros/state"),
                  ("~/mavros_extended_state_in", "mavros/extended_state"),
                  ("~/mavros_local_position_in", "mavros/local_position/odom"),
                  ("~/mavros_odometry_in", "mavros/odometry/in"),
                  ("~/mavros_global_position_in", "mavros/global_position/global"),
                  ("~/mavros_garmin_in", "mavros/garmin"),
                  ("~/mavros_imu_in", "mavros/imu/data"),
                  ("~/mavros_magnetometer_in", "mavros/global_position/compass_hdg"),
                  ("~/mavros_magnetic_field_in", "mavros/imu/mag"),
                  ("~/mavros_rc_in", "mavros/rc/in"),
                  ("~/mavros_altitude_in", "mavros/altitude"),
                  ("~/mavros_battery_in", "mavros/battery"),
                  ("~/mavros_gps_status_raw_in", "mavros/gpsstatus/gps1/raw"),
                  ("~/landed_state", "hw_api/landed_state"),

                  ("~/mavros_arming_out", "mavros/cmd/arming"),
                  ("~/mavros_takeoff_out", "mavros/cmd/takeoff"),
                  ("~/mavros_set_mode_out", "mavros/set_mode"),
                  ("~/mavros_position_setpoint_out", "mavros/setpoint_raw/local"),
                  ("~/mavros_attitude_setpoint_out", "mavros/setpoint_raw/attitude"),
                  ("~/mavros_actuator_control_out", "mavros/actuator_control"),

                ],
            )

        ],

    ))

    # Determine which MAVROS launch file to include based on simulation parameter
    # Only include one to avoid parsing both launch files
    simulation_value = os.getenv('RUN_TYPE', "simulation") == "simulation"
    
    if simulation_value:
        # Include MAVROS for simulation (SITL with UDP)
        ld.add_action(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([
                        FindPackageShare('mrs_uav_apm_api'),
                        'launch',
                        'mavros_sitl.launch.py'
                    ])
                )
            )
        )
    else:
        # Include MAVROS for real-world (hardware via serial)
        ld.add_action(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([
                        FindPackageShare('mrs_uav_apm_api'),
                        'launch',
                        'mavros_realworld.launch.py'
                    ])
                ),
                launch_arguments={
                    'fcu_url': fcu_url,
                }.items()
            )
        )

    return ld


