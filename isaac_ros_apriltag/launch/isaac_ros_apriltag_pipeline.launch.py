# Copyright (c) 2021, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.

import launch
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    rectify_node = ComposableNode(
        package='isaac_ros_image_proc',
        plugin='isaac_ros::image_proc::RectifyNode',
        name='rectify_node',
    )

    rectify_container = ComposableNodeContainer(
        name='rectify_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[rectify_node],
        output='screen'
    )

    apriltag_exe = Node(
        package='isaac_ros_apriltag',
        executable='isaac_ros_apriltag',
        name='apriltag_exe',
    )

    realsense_camera_node = Node(
        package='realsense2_camera',
        node_executable='realsense2_camera_node',
        parameters=[{
            'color_height': 1080,
            'color_width': 1920,
            'enable_infra1': False,
            'enable_infra2': False,
            'enable_depth': False,
        }],
        remappings=[('/color/image_raw', '/image'),
                    ('/color/camera_info', '/camera_info')]
    )

    return launch.LaunchDescription([rectify_container, apriltag_exe, realsense_camera_node])
