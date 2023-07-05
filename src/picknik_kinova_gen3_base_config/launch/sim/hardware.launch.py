# Copyright 2021-2023 PickNik Inc.
# All rights reserved.
#
# Unauthorized copying of this code base via any medium is strictly prohibited.
# Proprietary and confidential.

from launch import LaunchDescription
from launch_ros.actions import Node

from moveit_studio_utils_py.system_config import (
    SystemConfigParser,
)

from moveit_studio_utils_py.generate_camera_frames import (
    generate_camera_frames,
)


system_config_parser = SystemConfigParser()
cameras_config = system_config_parser.get_cameras_config()


def generate_launch_description():
    nodes_to_launch = []

    # Do not launch any nodes if there are no configured cameras.
    if not cameras_config:
        print(
            "No camera configuration found. Not launching any camera transform nodes."
        )
    else:
        frame_pair_params = [
            {
                "world_frame": "world",
                "camera_frames": generate_camera_frames(cameras_config),
            }
        ]
        camera_transforms_node = Node(
            package="moveit_studio_agent",
            executable="camera_transforms_node",
            name="camera_transforms_node",
            output="both",
            parameters=frame_pair_params,
        )
        nodes_to_launch.append(camera_transforms_node)

    return LaunchDescription(nodes_to_launch)
