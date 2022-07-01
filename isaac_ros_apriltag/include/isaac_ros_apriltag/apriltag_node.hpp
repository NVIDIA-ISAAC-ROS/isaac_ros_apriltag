/**
 * Copyright (c) 2021-2022, NVIDIA CORPORATION.  All rights reserved.
 *
 * NVIDIA CORPORATION and its licensors retain all intellectual property
 * and proprietary rights in and to this software, related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA CORPORATION is strictly prohibited.
 */

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
