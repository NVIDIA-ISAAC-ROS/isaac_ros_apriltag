# Copyright (c) 2022, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.

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
            ('camera_info', 'left/camerainfo')
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
