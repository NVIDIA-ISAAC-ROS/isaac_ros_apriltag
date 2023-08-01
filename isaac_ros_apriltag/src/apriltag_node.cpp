// SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
// Copyright (c) 2021-2023 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

#include <cstdio>
#include <memory>
#include <string>
#include <utility>

#include "isaac_ros_apriltag/apriltag_node.hpp"
#include "isaac_ros_nitros_april_tag_detection_array_type/nitros_april_tag_detection_array.hpp"
#include "isaac_ros_nitros_camera_info_type/nitros_camera_info.hpp"
#include "isaac_ros_nitros_image_type/nitros_image.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wmissing-field-initializers"
#pragma GCC diagnostic ignored "-Wpedantic"
#include "gxf/std/timestamp.hpp"
#include "extensions/fiducials/messages/fiducial_message.hpp"
#pragma GCC diagnostic pop


namespace nvidia
{
namespace isaac_ros
{
namespace apriltag
{

using nvidia::gxf::optimizer::GraphIOGroupSupportedDataTypesInfoList;

#define INPUT_COMPONENT_KEY_CAM                     "image_converter/camera_image_in"
#define INPUT_DEFAULT_TENSOR_FORMAT_CAM             "nitros_image_bgr8"
#define INPUT_TOPIC_NAME_CAM                        "image"

#define INPUT_COMPONENT_KEY_CAM_INFO                "camera_info_broadcaster/camera_info_in"
#define INPUT_DEFAULT_TENSOR_FORMAT_CAM_INFO        "nitros_camera_info"
#define INPUT_TOPIC_NAME_CAM_INFO                   "camera_info"

#define OUTPUT_COMPONENT_KEY_TAG_DETECTIONS         "sink/sink"
#define OUTPUT_DEFAULT_TENSOR_FORMAT_TAG_DETECTIONS "nitros_april_tag_detection_array"
#define OUTPUT_TOPIC_NAME_TAG_DETECTIONS            "tag_detections"

constexpr char APP_YAML_FILENAME[] = "config/apriltag_node.yaml";
constexpr char PACKAGE_NAME[] = "isaac_ros_apriltag";

const std::vector<std::pair<std::string, std::string>> EXTENSIONS = {
  {"isaac_ros_gxf", "gxf/lib/std/libgxf_std.so"},
  {"isaac_ros_gxf", "gxf/lib/multimedia/libgxf_multimedia.so"},
  {"isaac_ros_gxf", "gxf/lib/serialization/libgxf_serialization.so"},
  {"isaac_ros_gxf", "gxf/lib/cuda/libgxf_cuda.so"},
  {"isaac_ros_gxf", "gxf/lib/libgxf_gxf_helpers.so"},
  {"isaac_ros_gxf", "gxf/lib/libgxf_sight.so"},
  {"isaac_ros_gxf", "gxf/lib/libgxf_atlas.so"},
  {"isaac_ros_gxf", "gxf/lib/libgxf_isaac_messages.so"},
  {"isaac_ros_gxf", "gxf/lib/libgxf_message_compositor.so"},
  {"isaac_ros_image_proc", "gxf/lib/image_proc/libgxf_tensorops.so"},
  {"isaac_ros_apriltag", "gxf/lib/fiducials/libgxf_fiducials.so"}
};
const std::vector<std::string> PRESET_EXTENSION_SPEC_NAMES = {
  "isaac_ros_apriltag",
};
const std::vector<std::string> EXTENSION_SPEC_FILENAMES = {};
const std::vector<std::string> GENERATOR_RULE_FILENAMES = {
  "config/namespace_injector_rule_apriltag.yaml",
  "config/image_format_converter_substitution.yaml",
};
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
const nitros::NitrosPublisherSubscriberConfigMap CONFIG_MAP = {
  {INPUT_COMPONENT_KEY_CAM,
    {
      .type = nitros::NitrosPublisherSubscriberType::NEGOTIATED,
      .qos = rclcpp::QoS(10),
      .compatible_data_format = INPUT_DEFAULT_TENSOR_FORMAT_CAM,
      .topic_name = INPUT_TOPIC_NAME_CAM,
    }
  },
  {INPUT_COMPONENT_KEY_CAM_INFO,
    {
      .type = nitros::NitrosPublisherSubscriberType::NEGOTIATED,
      .qos = rclcpp::QoS(10),
      .compatible_data_format = INPUT_DEFAULT_TENSOR_FORMAT_CAM_INFO,
      .topic_name = INPUT_TOPIC_NAME_CAM_INFO,
    }
  },
  {OUTPUT_COMPONENT_KEY_TAG_DETECTIONS,
    {
      .type = nitros::NitrosPublisherSubscriberType::NEGOTIATED,
      .qos = rclcpp::QoS(10),
      .compatible_data_format = OUTPUT_DEFAULT_TENSOR_FORMAT_TAG_DETECTIONS,
      .topic_name = OUTPUT_TOPIC_NAME_TAG_DETECTIONS,
      .frame_id_source_key = INPUT_COMPONENT_KEY_CAM,
    }
  }
};
#pragma GCC diagnostic pop

void AprilTagNode::AprilTagDetectionsCallback(
  const gxf_context_t context,
  nitros::NitrosTypeBase & msg)
{
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_ =
    std::make_unique<tf2_ros::TransformBroadcaster>(*this);
  geometry_msgs::msg::TransformStamped transform_stamped;

  auto msg_entity = nvidia::gxf::Entity::Shared(context, msg.handle);

  // Populate timestamp information back into AprilTagDetectionArray header
  auto maybe_input_timestamp = msg_entity->get<nvidia::gxf::Timestamp>();
  if (!maybe_input_timestamp) {    // Fallback to label 'timestamp'
    maybe_input_timestamp = msg_entity->get<nvidia::gxf::Timestamp>("timestamp");
  }
  if (maybe_input_timestamp) {
    transform_stamped.header.stamp.sec = static_cast<int32_t>(
      maybe_input_timestamp.value()->acqtime / static_cast<uint64_t>(1e9));
    transform_stamped.header.stamp.nanosec = static_cast<uint32_t>(
      maybe_input_timestamp.value()->acqtime % static_cast<uint64_t>(1e9));
  } else {
    RCLCPP_WARN(
      get_logger(),
      "[AprilTagNode] Failed to get timestamp");
  }

  //  Extract apriltags array to a struct type defined in fiducial_message.hpp
  auto apriltags_detections_array_expected = nvidia::isaac::GetFiducialListMessage(
    msg_entity.value());
  if (!apriltags_detections_array_expected) {
    RCLCPP_ERROR(
      get_logger(),
      "[AprilTagNode] Failed to get apriltags detections data from message entity");
    return;
  }
  auto apriltags_detections_array = apriltags_detections_array_expected.value();

  // Extract number of tags detected
  size_t tags_count = apriltags_detections_array.count;

  for (size_t i = 0; i < tags_count; i++) {
    // struct is defined in fiducial_message.hpp
    auto info = apriltags_detections_array.info.at(i).value();
    auto pose = apriltags_detections_array.pose.at(i).value();

    // For AprilTag, the id is of the format <TagFamily_ID>
    // Ex. If the decoded tag ID is 14 and belongs to TagFamily tag36h11, the id is tag36h11_14
    // This struct in defined in fiducial_info.hpp
    std::string tag_family_id = info->id;
    // change format from <TagFamily_ID> to <TagFamily:ID>
    std::replace(tag_family_id.begin(), tag_family_id.end(), '_', ':');

    transform_stamped.header.frame_id = msg.frame_id;
    transform_stamped.child_frame_id = tag_family_id;
    transform_stamped.transform.translation.x = pose->translation.x();
    transform_stamped.transform.translation.y = pose->translation.y();
    transform_stamped.transform.translation.z = pose->translation.z();
    transform_stamped.transform.rotation.x = pose->rotation.quaternion().x();
    transform_stamped.transform.rotation.y = pose->rotation.quaternion().y();
    transform_stamped.transform.rotation.z = pose->rotation.quaternion().z();
    transform_stamped.transform.rotation.w = pose->rotation.quaternion().w();

    tf_broadcaster_->sendTransform(transform_stamped);
  }
}

AprilTagNode::AprilTagNode(const rclcpp::NodeOptions & options)
: nitros::NitrosNode(options,
    APP_YAML_FILENAME,
    CONFIG_MAP,
    PRESET_EXTENSION_SPEC_NAMES,
    EXTENSION_SPEC_FILENAMES,
    GENERATOR_RULE_FILENAMES,
    EXTENSIONS,
    PACKAGE_NAME),
  size_(declare_parameter<double>("size", 0.22)),
  max_tags_(declare_parameter<int>("max_tags", 64)),
  tile_size_(declare_parameter<uint16_t>("tile_size", 4))
{
  // add callback function for Apriltag Detection array to broadcast to ros tf tree
  config_map_[OUTPUT_COMPONENT_KEY_TAG_DETECTIONS].callback =
    std::bind(
    &AprilTagNode::AprilTagDetectionsCallback, this,
    std::placeholders::_1, std::placeholders::_2);
  RCLCPP_DEBUG(get_logger(), "[AprilTagNode] Constructor");

  registerSupportedType<nvidia::isaac_ros::nitros::NitrosAprilTagDetectionArray>();
  registerSupportedType<nvidia::isaac_ros::nitros::NitrosCameraInfo>();
  registerSupportedType<nvidia::isaac_ros::nitros::NitrosImage>();

  startNitrosNode();
}

void AprilTagNode::preLoadGraphCallback()
{
  RCLCPP_DEBUG(get_logger(), "[AprilTagNode] preLoadGraphCallback().");
}

void AprilTagNode::postLoadGraphCallback()
{
  RCLCPP_DEBUG(get_logger(), "[AprilTagNode] postLoadGraphCallback().");
  getNitrosContext().setParameterInt32(
    "cuda_april_tag_component", "nvidia::isaac::CudaAprilTagDetector", "max_tags", max_tags_);
  getNitrosContext().setParameterFloat64(
    "cuda_april_tag_component", "nvidia::isaac::CudaAprilTagDetector", "tag_dimensions", size_);
  getNitrosContext().setParameterUInt32(
    "cuda_april_tag_component", "nvidia::isaac::CudaAprilTagDetector", "tile_size", tile_size_);
}

AprilTagNode::~AprilTagNode() {}

}  // namespace apriltag
}  // namespace isaac_ros
}  // namespace nvidia

RCLCPP_COMPONENTS_REGISTER_NODE(nvidia::isaac_ros::apriltag::AprilTagNode)
