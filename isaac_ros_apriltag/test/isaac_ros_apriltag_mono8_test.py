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

import os
import pathlib
import time

from cv_bridge import CvBridge

from isaac_ros_apriltag_interfaces.msg import AprilTagDetectionArray
from isaac_ros_test import IsaacROSBaseTest, JSONConversion

from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

import pytest
import rclpy
from sensor_msgs.msg import CameraInfo, Image

BACKEND_FLAGS = ['CPU']

# Include PVA backend on Jetson Orin
if os.path.isfile('/etc/nv_tegra_release'):
    BACKEND_FLAGS.append('PVA')

TAG_DETECTION_TOPICS = [f'tag_detections_{b}' for b in BACKEND_FLAGS]


@pytest.mark.rostest
def generate_test_description():
    """Generate launch description with all ROS 2 nodes for testing."""
    containers = []
    for i in range(len(BACKEND_FLAGS)):
        apriltag_node = ComposableNode(
            package='isaac_ros_apriltag',
            plugin='nvidia::isaac_ros::apriltag::AprilTagNode',
            name=f'apriltag_{BACKEND_FLAGS[i]}',
            namespace=IsaacROSAprilTagPipelineTest.generate_namespace(),
            parameters=[{'size': 0.22,
                         'max_tags': 64,
                         'backends': BACKEND_FLAGS[i],
                         'tile_size': 4}],
            remappings=[('tag_detections', TAG_DETECTION_TOPICS[i])],
        )

        apriltag_container = ComposableNodeContainer(
            package='rclcpp_components',
            name=f'apriltag_container_{BACKEND_FLAGS[i]}',
            namespace='',
            executable='component_container_mt',
            composable_node_descriptions=[
                apriltag_node,
            ],
            output='screen'
        )
        containers.append(apriltag_container)

    return IsaacROSAprilTagPipelineTest.generate_test_description(containers)


class IsaacROSAprilTagPipelineTest(IsaacROSBaseTest):
    """Test for Isaac ROS Apriltag Pipeline."""

    filepath = pathlib.Path(os.path.dirname(__file__))

    @IsaacROSBaseTest.for_each_test_case()
    def test_apriltag_pipeline_with_mono8(self, test_folder) -> None:
        """Expect the pipeline to produce apriltag detections from images."""
        print(TAG_DETECTION_TOPICS)

        self.generate_namespace_lookup(
            ['image', 'camera_info'] + TAG_DETECTION_TOPICS)

        image_pub = self.node.create_publisher(
            Image, self.namespaces['image'], self.DEFAULT_QOS)
        camera_info_pub = self.node.create_publisher(
            CameraInfo, self.namespaces['camera_info'], self.DEFAULT_QOS)

        received_messages = {}
        tag_detections_sub = self.create_logging_subscribers(
            [(topic, AprilTagDetectionArray) for topic in TAG_DETECTION_TOPICS],
            received_messages,
            accept_multiple_messages=True,
        )

        try:
            raw_image = JSONConversion.load_image_from_json(
                test_folder / 'image.json')

            # Convert to mono8
            image = CvBridge().cv2_to_imgmsg(
                CvBridge().imgmsg_to_cv2(raw_image, desired_encoding='mono8'))
            image.encoding = 'mono8'

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

                # If we have received  one message on the output topic, break
                if all(topic in received_messages and len(received_messages[topic]) > 0
                       for topic in TAG_DETECTION_TOPICS):
                    done = True
                    break

            self.assertTrue(
                done, "Didn't receive output on tag_detections topic!")

            # Collect received detections
            for i in range(len(TAG_DETECTION_TOPICS)):
                tag_detections_actual = received_messages[TAG_DETECTION_TOPICS[i]][0]

                # Make sure that at least one detection was found
                self.assertGreaterEqual(len(tag_detections_actual.detections), 1,
                                        "Didn't find at least 1 detection in image!")

        finally:
            self.assertTrue(self.node.destroy_subscription(i) for i in tag_detections_sub)
            self.assertTrue(self.node.destroy_publisher(image_pub))
            self.assertTrue(self.node.destroy_publisher(camera_info_pub))
