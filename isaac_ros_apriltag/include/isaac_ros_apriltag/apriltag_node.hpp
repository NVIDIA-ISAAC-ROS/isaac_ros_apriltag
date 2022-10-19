// SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
// Copyright (c) 2021-2022 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
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

#include <string>
#include <chrono>
#include <utility>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "isaac_ros_nitros/nitros_node.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "isaac_ros_nitros/types/nitros_type_base.hpp"

namespace nvidia
{
namespace isaac_ros
{
namespace apriltag
{

class AprilTagNode : public nitros::NitrosNode
{
public:
  explicit AprilTagNode(const rclcpp::NodeOptions &);

  ~AprilTagNode();

  AprilTagNode(const AprilTagNode &) = delete;

  AprilTagNode & operator=(const AprilTagNode &) = delete;

  // The callback to be implemented by users for any required initialization
  void preLoadGraphCallback() override;
  void postLoadGraphCallback() override;

  // Callback linked to AprilTagDetectionArray publisher
  // used to publish detected tag poses to ros tf tree
  void AprilTagDetectionsCallback(const gxf_context_t, nitros::NitrosTypeBase &);

private:
  const double size_;
  const int max_tags_;
};

}  // namespace apriltag
}  // namespace isaac_ros
}  // namespace nvidia

#endif  // ISAAC_ROS_APRILTAG__APRILTAG_NODE_HPP_
