# Copyright (c) 2021, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.

import os
import pathlib
import time

from isaac_ros_apriltag_interfaces.msg import AprilTagDetectionArray
from isaac_ros_test import IsaacROSBaseTest, JSONConversion
import launch_ros
import pytest
import rclpy
from sensor_msgs.msg import CameraInfo, Image


@pytest.mark.rostest
def generate_test_description():
    """Generate launch description with all ROS2 nodes for testing."""
    rectify_node = launch_ros.descriptions.ComposableNode(
        package='isaac_ros_image_proc',
        plugin='isaac_ros::image_proc::RectifyNode',
        name='rectify_node',
        namespace=IsaacROSAprilTagPipelineTest.generate_namespace(),
    )

    rectify_container = launch_ros.actions.ComposableNodeContainer(
        name='rectify_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[rectify_node],
        output='screen'
    )

    apriltag_exe = launch_ros.actions.Node(
        package='isaac_ros_apriltag',
        executable='isaac_ros_apriltag',
        namespace=IsaacROSAprilTagPipelineTest.generate_namespace(),
    )

    return IsaacROSAprilTagPipelineTest.generate_test_description([
        rectify_container,
        apriltag_exe
    ])


class IsaacROSAprilTagPipelineTest(IsaacROSBaseTest):
    """Test for Isaac ROS Apriltag Pipeline."""

    filepath = pathlib.Path(os.path.dirname(__file__))

    @IsaacROSBaseTest.for_each_test_case()
    def test_apriltag_pipeline(self, test_folder) -> None:
        """Expect the pipeline to produce apriltag detections from images."""
        self.generate_namespace_lookup(
            ['image', 'camera_info', 'tag_detections'])

        image_pub = self.node.create_publisher(
            Image, self.namespaces['image'], self.DEFAULT_QOS)
        camera_info_pub = self.node.create_publisher(
            CameraInfo, self.namespaces['camera_info'], self.DEFAULT_QOS)

        received_messages = {}
        tag_detections_sub, = self.create_logging_subscribers(
            [('tag_detections', AprilTagDetectionArray)], received_messages)

        try:
            image = JSONConversion.load_image_from_json(
                test_folder / 'image.json')
            camera_info = JSONConversion.load_camera_info_from_json(
                test_folder / 'camera_info.json')

            # Publish test case over both topics
            image_pub.publish(image)
            camera_info_pub.publish(camera_info)

            # Wait at most TIMEOUT seconds for subscriber to respond
            TIMEOUT = 20
            end_time = time.time() + TIMEOUT

            done = False
            while time.time() < end_time:
                rclpy.spin_once(self.node, timeout_sec=0.1)

                # If we have received exactly one message on the output topic, break
                if 'tag_detections' in received_messages:
                    done = True
                    break

            print(f'done = {done}')
            # TODO: Fix  https://nvbugs/3424817 before uncommenting.
            # self.assertTrue(
            #     done, "Didn't receive output on tag_detections topic!")

            # # Collect received detections
            # tag_detections_actual = received_messages['tag_detections']

            # TODO: Fix  https://nvbugs/3424817 before uncommenting.
            # Make sure that at least one detection was found
            # self.assertGreaterEqual(len(tag_detections_actual.detections), 1,
            #                         "Didn't find at least 1 detection in image!")

            # TODO(jaiveers): Store and check precise tag location?

        finally:
            pass
            # TODO: Fix  https://nvbugs/3424817 before uncommenting.
            # self.assertTrue(self.node.destroy_subscription(tag_detections_sub))
            # self.assertTrue(self.node.destroy_publisher(image_pub))
            # self.assertTrue(self.node.destroy_publisher(camera_info_pub))
