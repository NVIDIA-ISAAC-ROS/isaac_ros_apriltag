# SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
# Copyright (c) 2024 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

from typing import Any, Dict

from isaac_ros_examples import IsaacROSLaunchFragment
import launch
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


class IsaacROSAprilTagLaunchFragment(IsaacROSLaunchFragment):

    @staticmethod
    def get_composable_nodes(interface_specs: Dict[str, Any]) -> Dict[str, ComposableNode]:
        # Apriltag parameters
        tag_family = LaunchConfiguration('tag_family')
        backends = LaunchConfiguration('backends')
        return {
            'apriltag_node': ComposableNode(
                package='isaac_ros_apriltag',
                plugin='nvidia::isaac_ros::apriltag::AprilTagNode',
                name='apriltag',
                namespace='',
                parameters=[{
                    'size': 0.22,
                    'max_tags': 64,
                    'tile_size': 4,
                    'tag_family': tag_family,
                    'backends': backends}
                ],
                remappings=[
                    ('image', 'image_rect'),
                    ('camera_info', 'camera_info_rect')
                ]
            )
        }

    @staticmethod
    def get_launch_actions(interface_specs: Dict[str, Any]) -> \
            Dict[str, launch.actions.OpaqueFunction]:
        return {
            'tag_family': DeclareLaunchArgument(
                'tag_family',
                default_value='tag36h11',
                description='Tag family of Apriltags to detect.'
            ),
            'backends': DeclareLaunchArgument(
                'backends',
                default_value='CUDA',
                description='VPI compute backends to perform detection with.'
            ),
        }


def generate_launch_description():
    apriltag_container = ComposableNodeContainer(
        package='rclcpp_components',
        name='apriltag_container',
        namespace='',
        executable='component_container_mt',
        composable_node_descriptions=IsaacROSAprilTagLaunchFragment
        .get_composable_nodes().values(),
        output='screen'
    )

    return launch.LaunchDescription(
        [apriltag_container] + IsaacROSAprilTagLaunchFragment.get_launch_actions().values())
