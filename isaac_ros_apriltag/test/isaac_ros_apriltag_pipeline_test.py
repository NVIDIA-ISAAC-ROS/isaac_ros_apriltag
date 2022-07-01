# Copyright (c) 2021-2022, NVIDIA CORPORATION.  All rights reserved.
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
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

import pytest
import rclpy
from sensor_msgs.msg import CameraInfo, Image


@pytest.mark.rostest
def generate_test_description():
    """Generate launch description with all ROS2 nodes for testing."""
    apriltag_node = ComposableNode(
        package='isaac_ros_apriltag',
        plugin='nvidia::isaac_ros::apriltag::AprilTagNode',
        name='apriltag',
        namespace=IsaacROSAprilTagPipelineTest.generate_namespace(),
        parameters=[{'size': 0.22,
                     'max_tags': 20}]
    )

    apriltag_container = ComposableNodeContainer(
        package='rclcpp_components',
        name='apriltag_container',
        namespace='',
        executable='component_container_mt',
        composable_node_descriptions=[
            apriltag_node,
        ],
        output='screen'
    )
    return IsaacROSAprilTagPipelineTest.generate_test_description([
        apriltag_container
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

            # Wait at most TIMEOUT seconds for subscriber to respond
            TIMEOUT = 20
            end_time = time.time() + TIMEOUT

            done = False
            while time.time() < end_time:
                # Publish test case multiple times
                # This is required because frames might be dropped
                # since QoS of this image publisher and subscriber(rectify) is
                # Reliability = Best effort and
                # Durability = Volatile
                camera_info_pub.publish(camera_info)
                image_pub.publish(image)
                rclpy.spin_once(self.node, timeout_sec=0.1)

                # If we have received exactly one message on the output topic, break
                if 'tag_detections' in received_messages:
                    done = True
                    break

            self.assertTrue(
                done, "Didn't receive output on tag_detections topic!")

            # Collect received detections
            tag_detections_actual = received_messages['tag_detections']

            # Make sure that at least one detection was found
            self.assertGreaterEqual(len(tag_detections_actual.detections), 1,
                                    "Didn't find at least 1 detection in image!")

            for tag_detection in tag_detections_actual.detections:
                self.assertEqual(tag_detection.id, 0)
                self.assertEqual(tag_detection.family, 'tag36h11')

                corners_message = 'Corners detection is not accurate'
                center_message = 'Center detection is not accurate'
                # Allow for 2 pixels of error in detection
                delta = 2
                # Ground truth data was obtained from generated image
                # found at ../image.png and ../test_cases/apritlag0
                self.assertAlmostEqual(tag_detection.center.x, 926.0, None,
                                       center_message, delta)
                self.assertAlmostEqual(tag_detection.center.y, 547.0, None,
                                       center_message, delta)
                self.assertAlmostEqual(tag_detection.corners[0].x, 1044.0, None,
                                       corners_message, delta)
                self.assertAlmostEqual(tag_detection.corners[0].y, 665.0, None,
                                       corners_message, delta)
                self.assertAlmostEqual(tag_detection.corners[1].x, 808.0, None,
                                       corners_message, delta)
                self.assertAlmostEqual(tag_detection.corners[1].y, 665.0, None,
                                       corners_message, delta)
                self.assertAlmostEqual(tag_detection.corners[2].x, 808.0, None,
                                       corners_message, delta)
                self.assertAlmostEqual(tag_detection.corners[2].y, 429.0, None,
                                       corners_message, delta)
                self.assertAlmostEqual(tag_detection.corners[3].x, 1044.0, None,
                                       corners_message, delta)
                self.assertAlmostEqual(tag_detection.corners[3].y, 429.0, None,
                                       corners_message, delta)

        finally:
            pass
            self.assertTrue(self.node.destroy_subscription(tag_detections_sub))
            self.assertTrue(self.node.destroy_publisher(image_pub))
            self.assertTrue(self.node.destroy_publisher(camera_info_pub))
