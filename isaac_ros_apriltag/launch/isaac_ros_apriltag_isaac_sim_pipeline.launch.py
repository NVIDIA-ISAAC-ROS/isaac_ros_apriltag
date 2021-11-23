# Copyright (c) 2021, NVIDIA CORPORATION.  All rights reserved.
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
    apriltag_node = ComposableNode(
        name='isaac_ros_apriltag',
        package='isaac_ros_apriltag',
        plugin='isaac_ros::apriltag::AprilTagNode',
        remappings=[('camera/image_rect', '/rgb_left'),
                    ('camera/camera_info', '/camera_info_left')],
        parameters=[{'family': '36h11',
                     'size': 0.32,
                     'max_tags': 20}]
    )

    apriltag_container = ComposableNodeContainer(
        name='apriltag_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[apriltag_node],
        output='screen'
    )

    return launch.LaunchDescription([apriltag_container])
