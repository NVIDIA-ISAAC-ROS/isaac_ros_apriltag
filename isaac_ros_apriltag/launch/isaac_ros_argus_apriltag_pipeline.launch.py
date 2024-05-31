# SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
# Copyright (c) 2021-2024 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# SPDX-License-Identifier: Apache-2.0

import launch
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    argus_mono_node = ComposableNode(
        name='argus_mono',
        package='isaac_ros_argus_camera',
        plugin='nvidia::isaac_ros::argus::ArgusMonoNode'
    )

    rectify_node = ComposableNode(
        name='rectify_node',
        package='isaac_ros_image_proc',
        plugin='nvidia::isaac_ros::image_proc::RectifyNode',
        parameters=[{
            'output_width': 1920,
            'output_height': 1200,
        }],
        remappings=[
            ('image_raw', 'left/image_raw'),
            ('camera_info', 'left/camera_info')
        ]
    )

    apriltag_node = ComposableNode(
        name='apriltag',
        package='isaac_ros_apriltag',
        plugin='nvidia::isaac_ros::apriltag::AprilTagNode',
        remappings=[
            ('image', 'image_rect'),
            ('camera_info', 'camera_info_rect')
        ]
    )

    apriltag_container = ComposableNodeContainer(
        name='apriltag_container',
        package='rclcpp_components',
        namespace='',
        executable='component_container_mt',
        composable_node_descriptions=[
            argus_mono_node,
            rectify_node,
            apriltag_node
        ],
        output='screen'
    )

    return launch.LaunchDescription([apriltag_container])
