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
import os.path
import pathlib
import time

from isaac_ros_apriltag_interfaces.msg import AprilTagDetectionArray
from isaac_ros_test import IsaacROSBaseTest, JSONConversion
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

import pytest
import rclpy
from sensor_msgs.msg import CameraInfo, Image


def get_backends_id(backends_flag):
    return backends_flag.replace(',', '_')


BACKEND_FLAGS = ['CPU', 'CUDA']


# Include PVA backend on Jetson Orin
if os.path.isfile('/etc/nv_tegra_release'):
    BACKEND_FLAGS.append('PVA')


TAG_DETECTION_TOPICS = [f'tag_detections_{get_backends_id(b)}' for b in BACKEND_FLAGS]


@pytest.mark.rostest
def generate_test_description():
    """Generate launch description with all ROS 2 nodes for testing."""
    containers = []
    for i in range(len(BACKEND_FLAGS)):
        backends_flag = BACKEND_FLAGS[i]
        tag_detection_topic = TAG_DETECTION_TOPICS[i]
        backends_id = get_backends_id(backends_flag)

        apriltag_node = ComposableNode(
            package='isaac_ros_apriltag',
            plugin='nvidia::isaac_ros::apriltag::AprilTagNode',
            name=f'apriltag_{backends_id}',
            namespace=IsaacROSBackendsCompareTest.generate_namespace(),
            parameters=[
                {
                    'backends': backends_flag,
                    'size': 0.22,
                    'max_tags': 64,
                    'tile_size': 4,
                }
            ],
            remappings=[('tag_detections', tag_detection_topic)],
        )

        apriltag_container = ComposableNodeContainer(
            package='rclcpp_components',
            name=f'apriltag_container_{backends_id}',
            namespace='',
            executable='component_container_mt',
            composable_node_descriptions=[
                apriltag_node,
            ],
            output='screen',
        )
        containers.append(apriltag_container)

    return IsaacROSBackendsCompareTest.generate_test_description(containers)


class IsaacROSBackendsCompareTest(IsaacROSBaseTest):
    """Test for Isaac ROS Apriltag Pipeline."""

    filepath = pathlib.Path(os.path.dirname(__file__))

    @IsaacROSBaseTest.for_each_test_case()
    def test_apriltag_backends(self, test_folder) -> None:
        """Expect the pipeline to produce apriltag detections from images."""
        self.generate_namespace_lookup(['image', 'camera_info'] + TAG_DETECTION_TOPICS)

        image_pub = self.node.create_publisher(
            Image, self.namespaces['image'], self.DEFAULT_QOS
        )
        camera_info_pub = self.node.create_publisher(
            CameraInfo, self.namespaces['camera_info'], self.DEFAULT_QOS
        )

        received_messages = {}
        detections_subs = self.create_logging_subscribers(
            [(topic, AprilTagDetectionArray) for topic in TAG_DETECTION_TOPICS],
            received_messages,
            accept_multiple_messages=True,
        )

        try:
            image = JSONConversion.load_image_from_json(test_folder / 'image.json')
            camera_info = JSONConversion.load_camera_info_from_json(
                test_folder / 'camera_info.json'
            )

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
                if all(topic in received_messages and len(received_messages[topic]) > 0
                       for topic in TAG_DETECTION_TOPICS):
                    done = True
                    break

            self.assertTrue(
                done, 'Did not receive output on tag_detections topic for all backends!'
            )

            # Make sure that at least one detection was found
            for topic in TAG_DETECTION_TOPICS:
                self.assertGreaterEqual(
                    len(received_messages[topic][0].detections),
                    1,
                    f'Did not find at least 1 detection in {topic}!',
                )

            first_tag_detections = received_messages[TAG_DETECTION_TOPICS[0]][0]
            for i in range(1, len(TAG_DETECTION_TOPICS)):
                tag_detections = received_messages[TAG_DETECTION_TOPICS[i]][0]
                self.assertEqual(
                    len(tag_detections.detections), len(first_tag_detections.detections)
                )

                for j in range(len(tag_detections.detections)):
                    first_tag_detection = first_tag_detections.detections[j]
                    tag_detection = tag_detections.detections[j]

                    self.assertEqual(tag_detection.id, first_tag_detection.id)
                    self.assertEqual(tag_detection.family, first_tag_detection.family)

                    delta = 2  # pixels of error
                    delta_translation = 0.01  # 1 cm threshold
                    delta_quaternion = 0.01  # numerical precision only

                    # Compare image center
                    center_message = 'Center detection does not match'
                    self.assertAlmostEqual(
                        tag_detection.center.x,
                        first_tag_detection.center.x,
                        None,
                        center_message,
                        delta,
                    )
                    self.assertAlmostEqual(
                        tag_detection.center.y,
                        first_tag_detection.center.y,
                        None,
                        center_message,
                        delta,
                    )

                    # Compare corners in image space
                    corners_message = 'Corners detections do not match'
                    for i in range(4):
                        self.assertAlmostEqual(
                            tag_detection.corners[i].x,
                            first_tag_detection.corners[i].x,
                            None,
                            corners_message,
                            delta,
                        )
                        self.assertAlmostEqual(
                            tag_detection.corners[i].y,
                            first_tag_detection.corners[i].y,
                            None,
                            corners_message,
                            delta,
                        )

                    # Comapre SE(3) pose - translation
                    tag_translation = [
                        tag_detection.pose.pose.pose.position.x,
                        tag_detection.pose.pose.pose.position.y,
                        tag_detection.pose.pose.pose.position.z,
                    ]
                    first_tag_translation = [
                        first_tag_detection.pose.pose.pose.position.x,
                        first_tag_detection.pose.pose.pose.position.y,
                        first_tag_detection.pose.pose.pose.position.z,
                    ]
                    translation_message = f'Translations do not match: [{tag_translation}], \
                        expected=[{first_tag_translation}]'

                    for i in range(3):
                        self.assertAlmostEqual(
                            tag_translation[i],
                            first_tag_translation[i],
                            None,
                            translation_message,
                            delta_translation,
                        )

                    # Comapre SE(3) pose - orientation
                    tag_orientation = [
                        tag_detection.pose.pose.pose.orientation.w,
                        tag_detection.pose.pose.pose.orientation.x,
                        tag_detection.pose.pose.pose.orientation.y,
                        tag_detection.pose.pose.pose.orientation.z,
                    ]
                    first_tag_orientation = [
                        first_tag_detection.pose.pose.pose.orientation.w,
                        first_tag_detection.pose.pose.pose.orientation.x,
                        first_tag_detection.pose.pose.pose.orientation.y,
                        first_tag_detection.pose.pose.pose.orientation.z,
                    ]
                    orientation_message = f'Orientation does not match: [{tag_orientation}], \
                          expected=[{first_tag_orientation}]'
                    for i in range(4):
                        self.assertAlmostEqual(
                            tag_orientation[i],
                            first_tag_orientation[i],
                            None,
                            orientation_message,
                            delta_quaternion,
                        )
        finally:
            self.assertTrue(all(self.node.destroy_subscription(sub) for sub in detections_subs))
            self.assertTrue(self.node.destroy_publisher(image_pub))
            self.assertTrue(self.node.destroy_publisher(camera_info_pub))
