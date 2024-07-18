# Copyright (c) 2023 José Miguel Guerrero Hernández
#
# This file is licensed under the terms of the MIT license.
# See the LICENSE file in the root of this repository.

import os

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import (IncludeLaunchDescription)
from ament_index_python.packages import (get_package_share_directory)
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():

    # Launch the yolo node
    yolo_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('yolov8_bringup'),
            'launch'), '/yolov8.launch.py']),
        launch_arguments={
                            'model': 'yolov8m-seg.pt',
                            'input_image_topic': '/head_front_camera/rgb/image_raw',
                            }.items()

    )

    # Launch the computer vision node
    cv_node = Node(
        package='ros2_semantic_mapping',
        namespace='ros2_semantic_mapping',
        executable='mapper',
        output='both',
        emulate_tty=True,
        # Use topics from robot
        remappings=[
            ('/camera_info', '/head_front_camera/rgb/camera_info'),
            ('/image_depth_in', '/head_front_camera/depth_registered/image_raw'),
        ],
    )

    ld = LaunchDescription()
    ld.add_action(yolo_node)
    ld.add_action(cv_node)

    return ld