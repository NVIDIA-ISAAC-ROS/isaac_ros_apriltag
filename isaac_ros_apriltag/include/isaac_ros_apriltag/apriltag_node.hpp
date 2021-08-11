/**
 * Copyright (c) 2021, NVIDIA CORPORATION.  All rights reserved.
 *
 * NVIDIA CORPORATION and its licensors retain all intellectual property
 * and proprietary rights in and to this software, related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA CORPORATION is strictly prohibited.
 */

#ifndef ISAAC_ROS_APRILTAG__APRILTAG_NODE_HPP_
#define ISAAC_ROS_APRILTAG__APRILTAG_NODE_HPP_

#include <memory>
#include <string>

#include "eigen3/Eigen/Core"
#include "image_transport/camera_subscriber.hpp"
#include "isaac_ros_apriltag_interfaces/msg/april_tag_detection.hpp"
#include "isaac_ros_apriltag_interfaces/msg/april_tag_detection_array.hpp"
#include "message_filters/subscriber.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "tf2_msgs/msg/tf_message.hpp"


namespace isaac_ros
{
namespace apriltag
{

class AprilTagNode : public rclcpp::Node
{
public:
  explicit AprilTagNode(const rclcpp::NodeOptions options = rclcpp::NodeOptions());

  ~AprilTagNode();

private:
  void onCameraFrame(
    const sensor_msgs::msg::Image::ConstSharedPtr & msg_img,
    const sensor_msgs::msg::CameraInfo::ConstSharedPtr & msg_ci);

  const std::string tag_family_;
  const double tag_edge_size_;
  const int max_tags_;
  const int queue_size_;
  const int32_t max_interval_duration_;  // Unit in seconds

  message_filters::Subscriber<sensor_msgs::msg::Image> image_sub_;
  message_filters::Subscriber<sensor_msgs::msg::CameraInfo> camera_info_sub_;

  const rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr pub_tf_;
  const rclcpp::Publisher<isaac_ros_apriltag_interfaces::msg::AprilTagDetectionArray>::SharedPtr
    pub_detections_;

  struct AprilTagsImpl;
  std::unique_ptr<AprilTagsImpl> impl_;
};

}  // namespace apriltag
}  // namespace isaac_ros

#endif  // ISAAC_ROS_APRILTAG__APRILTAG_NODE_HPP_
