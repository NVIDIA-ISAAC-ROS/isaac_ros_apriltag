/**
 * Copyright (c) 2021, NVIDIA CORPORATION.  All rights reserved.
 *
 * NVIDIA CORPORATION and its licensors retain all intellectual property
 * and proprietary rights in and to this software, related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA CORPORATION is strictly prohibited.
 */

#include <memory>

#include "isaac_ros_apriltag/apriltag_node.hpp"
#include "isaac_ros_image_proc/image_format_converter_node.hpp"
#include "rclcpp/rclcpp.hpp"


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  rclcpp::executors::MultiThreadedExecutor exec;

  rclcpp::NodeOptions format_options;
  format_options.arguments(
  {
    "--ros-args",
    "-p", "encoding_desired:=rgba8",
    "-p", "backends:=CUDA",
    "-r", "image_raw:=image_rect",  // Input topic: image_rect
    "-r", "image:=apriltag/image_rect"  // Output topic: apriltag/image_rect
  });
  auto format_node =
    std::make_shared<isaac_ros::image_proc::ImageFormatConverterNode>(format_options);
  exec.add_node(format_node);

  rclcpp::NodeOptions apriltag_options;
  apriltag_options.arguments(
  {
    "--ros-args",
    "-p", "image_transport:=raw",
    "-p", "family:=36h11",
    "-p", "size:=0.162",
    "-r", "camera/image_rect:=apriltag/image_rect",  // Input topic: apriltag/image_rect
    "-r", "camera/camera_info:=camera_info",  // Input topic: camera_info
  });
  auto apriltag_node = std::make_shared<isaac_ros::apriltag::AprilTagNode>(apriltag_options);
  exec.add_node(apriltag_node);

  exec.spin();

  rclcpp::shutdown();
  return 0;
}
