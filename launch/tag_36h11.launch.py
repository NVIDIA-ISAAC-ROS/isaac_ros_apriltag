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

# detect all 36h11 tags
cfg_36h11 = {
    "image_transport": "raw",
    "family": "36h11",
    "size": 0.162
}

def generate_launch_description():
    composable_node = ComposableNode(
        name='apriltag',
        package='nvapriltags_ros2', plugin='AprilTagNode',
        remappings=[("/apriltag/image", "/camera/image"),
                    ("/apriltag/camera_info", "/camera/camera_info")],
        parameters=[cfg_36h11])
    container = ComposableNodeContainer(
        name='tag_container',
        namespace='apriltag',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[composable_node],
        output='screen'
    )

    return launch.LaunchDescription([container])
