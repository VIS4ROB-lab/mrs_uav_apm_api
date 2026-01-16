#!/usr/bin/env python3

import launch
import os
import launch_ros

from launch_ros.actions import Node, SetParameter
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

    uav_name = os.getenv("UAV_NAME", "uav")

    vision_pose_topic = LaunchConfiguration('vision_pose_topic')

    declare_vis = DeclareLaunchArgument(
        'vision_pose_topic',
        default_value="/vicon/x4/x4/pose",
        description='Topic for vision pose input.'
    )

    ld.add_action(declare_vis)

    apm_launch_arguments = {
        "vision_pose_topic": vision_pose_topic,
    }

    print(apm_launch_arguments.items())

    ld.add_action(
        IncludeLaunchDescription(
            XMLLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('eagleeye_tools'),
                    'launch/mavros.launch')
                ),
                launch_arguments=apm_launch_arguments.items()
        )
    )

    return ld


