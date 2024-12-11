// SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
// Copyright (c) 2021-2024 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
// http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// SPDX-License-Identifier: Apache-2.0

#ifndef ISAAC_ROS_APRILTAG__APRILTAG_NODE_HPP_
#define ISAAC_ROS_APRILTAG__APRILTAG_NODE_HPP_

#include <memory>
#include <string>
#include <chrono>
#include <utility>
#include <vector>

#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "isaac_ros_apriltag_interfaces/msg/april_tag_detection_array.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "tf2_ros/transform_broadcaster.h"

#include "message_filters/subscriber.h"
#include "message_filters/synchronizer.h"
#include "message_filters/sync_policies/exact_time.h"

#include "isaac_ros_managed_nitros/managed_nitros_message_filters_subscriber.hpp"
#include "isaac_ros_nitros_image_type/nitros_image_view.hpp"

namespace nvidia
{
namespace isaac_ros
{
namespace apriltag
{

class AprilTagNode : public rclcpp::Node
{
public:
  explicit AprilTagNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  ~AprilTagNode();
  AprilTagNode(const AprilTagNode &) = delete;
  AprilTagNode & operator=(const AprilTagNode &) = delete;

private:
  void CameraImageCallback(
    const nvidia::isaac_ros::nitros::NitrosImage::ConstSharedPtr & nitros_image,
    const sensor_msgs::msg::CameraInfo::ConstSharedPtr & camera_info
  );

  // Parameters
  const int max_tags_;
  const double size_;
  const uint32_t tile_size_;
  const std::string tag_family_;
  const uint32_t backends_;

  // Subscribers
  template<typename T>
  using NitrosMessageFilterSubscriber = nvidia::isaac_ros::nitros::message_filters::Subscriber<T>;

  NitrosMessageFilterSubscriber<nvidia::isaac_ros::nitros::NitrosImageView> image_sub_;
  message_filters::Subscriber<sensor_msgs::msg::CameraInfo> camera_info_sub_;

  using ExactPolicy = message_filters::sync_policies::ExactTime<
    nvidia::isaac_ros::nitros::NitrosImage,
    sensor_msgs::msg::CameraInfo
  >;
  message_filters::Synchronizer<ExactPolicy> camera_image_sync_;

  // Publishers
  const rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr tf_pub_;
  const rclcpp::Publisher<isaac_ros_apriltag_interfaces::msg::AprilTagDetectionArray>::SharedPtr
    detections_pub_;

  struct CUAprilTagImpl;
  struct VPIAprilTagImpl;
  struct AprilTagImpl;
  std::unique_ptr<AprilTagImpl> impl_;
};

}  // namespace apriltag
}  // namespace isaac_ros
}  // namespace nvidia

#endif  // ISAAC_ROS_APRILTAG__APRILTAG_NODE_HPP_
