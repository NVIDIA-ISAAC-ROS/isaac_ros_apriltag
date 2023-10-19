# SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
# Copyright (c) 2021-2023 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

import os

from ament_index_python.packages import get_package_share_directory

import launch
from launch.substitutions import Command
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode


def generate_launch_description():

    apriltag_node = ComposableNode(
        package='isaac_ros_apriltag',
        plugin='nvidia::isaac_ros::apriltag::AprilTagNode',
        name='apriltag',
        namespace='',
        remappings=[
            ('image', 'zed_node/left/image_rect_color_rgb'),
            ('camera_info', 'zed_node/left/camera_info')
        ]
    )

    image_format_converter_node_left = ComposableNode(
        package='isaac_ros_image_proc',
        plugin='nvidia::isaac_ros::image_proc::ImageFormatConverterNode',
        name='image_format_node_left',
        parameters=[{
                'encoding_desired': 'rgb8',
        }],
        remappings=[
            ('image_raw', 'zed_node/left/image_rect_color'),
            ('image', 'zed_node/left/image_rect_color_rgb')]
    )

    apriltag_container = ComposableNodeContainer(
        package='rclcpp_components',
        name='apriltag_container',
        namespace='',
        executable='component_container_mt',
        composable_node_descriptions=[
            apriltag_node,
            image_format_converter_node_left
        ],
        output='screen'
    )

    # The zed camera mode name. zed, zed2, zed2i, zedm, zedx or zedxm
    camera_model = 'zed2i'

    # URDF/xacro file to be loaded by the Robot State Publisher node
    xacro_path = os.path.join(
        get_package_share_directory('zed_wrapper'),
        'urdf', 'zed_descr.urdf.xacro'
    )

    # ZED Configurations to be loaded by ZED Node
    config_common = os.path.join(
        get_package_share_directory('isaac_ros_apriltag'),
        'config',
        'zed.yaml'
    )

    config_camera = os.path.join(
        get_package_share_directory('zed_wrapper'),
        'config',
        camera_model + '.yaml'
    )

    # Robot State Publisher node
    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='zed_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': Command(
                [
                    'xacro', ' ', xacro_path, ' ',
                    'camera_name:=', camera_model, ' ',
                    'camera_model:=', camera_model
                ])
        }]
    )

    # ZED node using manual composition
    zed_node = Node(
        package='zed_wrapper',
        executable='zed_wrapper',
        output='screen',
        parameters=[
            config_common,  # Common parameters
            config_camera,  # Camera related parameters
        ]
    )

    rviz_config_path = os.path.join(get_package_share_directory(
        'isaac_ros_apriltag'), 'config', 'isaac_ros_apriltag_zed.rviz')

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config_path],
        output='screen')

    # Add nodes and containers to LaunchDescription
    return launch.LaunchDescription([apriltag_container, rviz, rsp_node, zed_node])
