# Copyright 2021 PickNik Inc.
# All rights reserved.
#
# Unauthorized copying of this code base via any medium is strictly prohibited.
# Proprietary and confidential.
import os
import sys

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription, LaunchService
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

from moveit_studio_utils_py.generate_camera_frames import (
    generate_camera_frames,
)

from moveit_studio_utils_py.system_config import SystemConfigParser


def generate_launch_description():
    nodes_to_start = []
    included_launch_files = []
    system_config_parser = SystemConfigParser()
    cameras_config = system_config_parser.get_cameras_config()
    
    frame_pair_params = [
        {
            "world_frame": "world",
            "camera_frames": generate_camera_frames(cameras_config),
        }
    ]

    # Load the capture config
    capture_config = os.path.join(
        get_package_share_directory("picknik_kinova_gen3_config"),
        "calibration",
        "capture.yaml",
    )

    # Load the calibration config
    calibration_config = os.path.join(
        get_package_share_directory("picknik_kinova_gen3_config"),
        "calibration",
        "calibrate.yaml",
    )

    # Load the calibration poses YAML
    calibration_poses = os.path.join(
        get_package_share_directory("picknik_kinova_gen3_config"),
        "calibration",
        "calibration_poses.yaml",
    )

    nodes_to_start.append(
        Node(
            package="moveit_studio_agent",
            executable="camera_transforms_node",
            name="camera_transforms_node",
            output="both",
            parameters=frame_pair_params,
        )
    )

    nodes_to_start.append(
        Node(
                name="robot_calibration",
                package="robot_calibration",
                executable="calibrate_server",
                arguments=[calibration_poses],
                parameters=[capture_config, calibration_config],
                output="screen",
            )
    )      

    return LaunchDescription(nodes_to_start + included_launch_files)
