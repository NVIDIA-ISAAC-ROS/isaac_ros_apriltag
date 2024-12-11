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

#include "isaac_ros_apriltag/apriltag_node.hpp"

#include <vpi/VPI.h>
#include <vpi/algo/AprilTags.h>
#include <vpi/algo/ConvertImageFormat.h>
#include <vpi/Types.h>

#include <cuda_runtime.h>

#include <algorithm>
#include <cstdio>
#include <iterator>
#include <memory>
#include <string>
#include <utility>

#include "cuAprilTags.h"
#include "eigen3/Eigen/Dense"
#include "isaac_ros_common/vpi_utilities.hpp"

namespace nvidia
{
namespace isaac_ros
{
namespace apriltag
{

namespace
{
// Map from human-friendly string to VPIAprilTagFamily
const std::map<std::string, VPIAprilTagFamily> g_str_to_vpi_apriltag_family({
          {"tag36h11", VPI_APRILTAG_36H11},
          {"tag16h5", VPI_APRILTAG_16H5},
          {"tag25h9", VPI_APRILTAG_25H9},
          {"tag36h10", VPI_APRILTAG_36H10},
          {"tag36h11", VPI_APRILTAG_36H11},
          {"circle21h7", VPI_APRILTAG_CIRCLE21H7},
          {"circle49h12", VPI_APRILTAG_CIRCLE49H12},
          {"custom48h12", VPI_APRILTAG_CUSTOM48H12},
          {"standard41h12", VPI_APRILTAG_STANDARD41H12},
          {"standard52h13", VPI_APRILTAG_STANDARD52H13}
        });

VPIAprilTagFamily ToVPIAprilTagFamily(const std::string & family)
{
  auto it = g_str_to_vpi_apriltag_family.find(family);
  if (it == g_str_to_vpi_apriltag_family.end()) {return VPI_APRILTAG_INVALID;}
  return it->second;
}

std::string ToString(const VPIAprilTagFamily family)
{
  auto it = std::find_if(
    std::begin(g_str_to_vpi_apriltag_family), std::end(g_str_to_vpi_apriltag_family),
    [family](auto && p) {return p.second == family;});
  if (it == std::end(g_str_to_vpi_apriltag_family)) {return "";}
  return {it->first};
}

std::unordered_map<std::string, VPIImageFormat> g_encoding_str_to_vpi_image_format({
          {"rgb8", VPI_IMAGE_FORMAT_RGB8},
          {"bgr8", VPI_IMAGE_FORMAT_BGR8},
          {"rgba8", VPI_IMAGE_FORMAT_RGBA8},
          {"bgra8", VPI_IMAGE_FORMAT_BGRA8},
          {"mono8", VPI_IMAGE_FORMAT_U8}
        });

VPIImageFormat ToVPIImageFormatFromROSEncoding(const std::string & encoding)
{
  auto it = g_encoding_str_to_vpi_image_format.find(encoding);
  if (it == g_encoding_str_to_vpi_image_format.end()) {return VPI_IMAGE_FORMAT_INVALID;}
  return it->second;
}

}  // namespace

struct AprilTagNode::AprilTagImpl
{
  bool IsInitialized() const {return initialized_;}

  virtual std::unordered_set<VPIAprilTagFamily> SupportedTagFamilies() const = 0;

  virtual void Initialize(
    const AprilTagNode & node,
    const nvidia::isaac_ros::nitros::NitrosImageView & nitros_image_view,
    const sensor_msgs::msg::CameraInfo::ConstSharedPtr & camera_info)
  {
    (void)nitros_image_view;
    (void)camera_info;
    assert(!IsInitialized() && "Already initialized.");

    initialized_ = true;

    tag_family_str_ = node.tag_family_;
    tag_family_ = ToVPIAprilTagFamily(tag_family_str_);

    // Check tag family specified is supported
    const auto supported_tag_families = SupportedTagFamilies();
    assert(
      supported_tag_families.find(tag_family_) != supported_tag_families.end() && \
      "Tag family not supported by this implementation.");
  }

  virtual void OnCameraFrame(
    const AprilTagNode & node,
    const nvidia::isaac_ros::nitros::NitrosImageView & nitros_image_view,
    const sensor_msgs::msg::CameraInfo::ConstSharedPtr & camera_info) = 0;

  bool initialized_{false};

  // Supported tag families
  VPIAprilTagFamily tag_family_{VPI_APRILTAG_INVALID};
  std::string tag_family_str_{};
};


struct AprilTagNode::VPIAprilTagImpl : AprilTagNode::AprilTagImpl
{
  VPIStream stream_{};
  VPIAprilTagDecodeParams params_{};
  VPIPayload detector_{};
  VPICameraIntrinsic cam_intrinsics_{};

  VPIImageData input_image_data_{};
  VPIImage input_image_{};
  VPIImage input_monochrome_image_{};

  geometry_msgs::msg::Transform ToTransformMsg(const VPIPose & pose)
  {
    // VPIPose stores a transform matrix as 3x4 [R | T]
    // [r00 r01 r02 x]
    // [r10 r11 r12 y]
    // [r20 r21 r22 z]

    // Extract translation from VPIPose
    geometry_msgs::msg::Transform t;
    t.translation.x = pose.transform[0][3];
    t.translation.y = pose.transform[1][3];
    t.translation.z = pose.transform[2][3];

    // Extract rotation matrix from VPIPose
    float rotation[3][3];
    for (size_t i = 0; i < 3; i++) {
      for (size_t j = 0; j < 3; j++) {
        rotation[i][j] = pose.transform[i][j];
      }
    }

    // Rotation matrix from VPIPose is row major
    const Eigen::Map<const Eigen::Matrix<float, 3, 3, Eigen::RowMajor>>
    orientation((const float *)&rotation);
    Eigen::Quaternion<float> q(orientation);
    q.normalize();

    t.rotation.w = q.w();
    t.rotation.x = q.x();
    t.rotation.y = q.y();
    t.rotation.z = q.z();

    return t;
  }

  std::unordered_set<VPIAprilTagFamily> SupportedTagFamilies() const override
  {
    // All tag families are supported on all backends
    std::unordered_set<VPIAprilTagFamily> tag_families;
    std::transform(
      g_str_to_vpi_apriltag_family.begin(), g_str_to_vpi_apriltag_family.end(),
      std::inserter(tag_families, tag_families.end()),
      [](auto && key_value_pair) {return key_value_pair.second;});
    return tag_families;
  }

  void Initialize(
    const AprilTagNode & node,
    const nvidia::isaac_ros::nitros::NitrosImageView & nitros_image_view,
    const sensor_msgs::msg::CameraInfo::ConstSharedPtr & camera_info) override
  {
    AprilTagNode::AprilTagImpl::Initialize(node, nitros_image_view, camera_info);

    try {
      CHECK_STATUS(vpiStreamCreate(node.backends_ | VPI_BACKEND_CPU | VPI_BACKEND_CUDA, &stream_));
    } catch (std::runtime_error & e) {
      RCLCPP_ERROR(node.get_logger(), "Error while initializing: %s", e.what());
    }

    CHECK_STATUS(vpiInitAprilTagDecodeParams(&params_));

    params_.family = tag_family_;

    // Camera intrinsics
    const double * k = camera_info->k.data();
    const float fx = static_cast<float>(k[0]);
    const float fy = static_cast<float>(k[4]);
    const float cx = static_cast<float>(k[2]);
    const float cy = static_cast<float>(k[5]);
    const float sk = static_cast<float>(k[1]);
    VPICameraIntrinsic cam_intrinsics = {
      {fx, sk, cx},
      {0, fy, cy}
    };
    std::memcpy(&cam_intrinsics_, &cam_intrinsics, sizeof(VPICameraIntrinsic));

    // Detector
    CHECK_STATUS(
      vpiCreateAprilTagDetector(
        node.backends_, camera_info->width,
        camera_info->height, &params_, &detector_));


    // Input image placeholder
    VPIImageData * data = &input_image_data_;
    data->bufferType = VPI_IMAGE_BUFFER_CUDA_PITCH_LINEAR;
    data->buffer.pitch.format = ToVPIImageFormatFromROSEncoding(nitros_image_view.GetEncoding());
    data->buffer.pitch.numPlanes = 1;
    data->buffer.pitch.planes[0].data =
      const_cast<void *>(reinterpret_cast<const void *>(nitros_image_view.GetGpuData()));
    data->buffer.pitch.planes[0].height = camera_info->height;
    data->buffer.pitch.planes[0].width = camera_info->width;
    data->buffer.pitch.planes[0].pixelType = VPI_PIXEL_TYPE_DEFAULT;
    data->buffer.pitch.planes[0].pitchBytes = nitros_image_view.GetStride();
    CHECK_STATUS(vpiImageCreateWrapper(data, nullptr, VPI_BACKEND_CUDA, &input_image_));

    RCLCPP_INFO(
      node.get_logger(), "Initialized with input image (%dx%d), encoding=%s, pitch=%d",
      data->buffer.pitch.planes[0].width,
      data->buffer.pitch.planes[0].height,
      nitros_image_view.GetEncoding().c_str(),
      data->buffer.pitch.planes[0].pitchBytes);

    // Input monochrome image
    CHECK_STATUS(
      vpiImageCreate(
        camera_info->width, camera_info->height,
        VPI_IMAGE_FORMAT_U8, 0, &input_monochrome_image_));
  }

  void OnCameraFrame(
    const AprilTagNode & node,
    const nvidia::isaac_ros::nitros::NitrosImageView & nitros_image_view,
    const sensor_msgs::msg::CameraInfo::ConstSharedPtr & camera_info) override
  {
    // Update input image buffer in wrapper
    input_image_data_.buffer.pitch.planes[0].data =
      const_cast<void *>(reinterpret_cast<const void *>(nitros_image_view.GetGpuData()));
    CHECK_STATUS(vpiImageSetWrapper(input_image_, &input_image_data_))

    // Submit image conversion to U8
    VPIConvertImageFormatParams convert_params;
    CHECK_STATUS(vpiInitConvertImageFormatParams(&convert_params));

    CHECK_STATUS(
      vpiSubmitConvertImageFormat(
        stream_, VPI_BACKEND_CUDA, input_image_,
        input_monochrome_image_, &convert_params));

    // Perform AprilTag detection
    VPIArray detections_array;
    CHECK_STATUS(
      vpiArrayCreate(
        node.max_tags_, VPI_ARRAY_TYPE_APRILTAG_DETECTION, 0,
        &detections_array));
    CHECK_STATUS(
      vpiSubmitAprilTagDetector(
        stream_, node.backends_, detector_, node.max_tags_,
        input_monochrome_image_, detections_array));

    VPIArray detections_pose_array;
    CHECK_STATUS(vpiArrayCreate(node.max_tags_, VPI_ARRAY_TYPE_POSE, 0, &detections_pose_array));
    CHECK_STATUS(
      vpiSubmitAprilTagPoseEstimation(
        stream_, VPI_BACKEND_CPU, detections_array,
        cam_intrinsics_, node.size_, detections_pose_array));

    CHECK_STATUS(vpiStreamSync(stream_));

    int32_t num_detections = 0;
    CHECK_STATUS(vpiArrayGetSize(detections_array, &num_detections));

    VPIArrayData detections_data;
    CHECK_STATUS(
      vpiArrayLockData(
        detections_array, VPI_LOCK_READ, VPI_ARRAY_BUFFER_HOST_AOS,
        &detections_data));
    VPIAprilTagDetection * detections =
      reinterpret_cast<VPIAprilTagDetection *>(detections_data.buffer.aos.data);

    VPIArrayData detections_pose_data;
    CHECK_STATUS(
      vpiArrayLockData(
        detections_pose_array, VPI_LOCK_READ, VPI_ARRAY_BUFFER_HOST_AOS,
        &detections_pose_data));
    VPIPose * poses = reinterpret_cast<VPIPose *>(detections_pose_data.buffer.aos.data);

    // Parse detections into published messages
    isaac_ros_apriltag_interfaces::msg::AprilTagDetectionArray msg_detections;
    msg_detections.header = camera_info->header;
    tf2_msgs::msg::TFMessage tfs;
    for (int32_t i = 0; i < num_detections; i++) {
      const VPIAprilTagDetection & detection = detections[i];
      const VPIPose & pose = poses[i];

      // detection
      isaac_ros_apriltag_interfaces::msg::AprilTagDetection msg_detection;
      msg_detection.family = tag_family_str_;
      msg_detection.id = detection.id;

      // corners
      for (int corner_idx = 0; corner_idx < 4; corner_idx++) {
        // Corners are in reverse order from expected
        int dest_corner_idx = 3 - corner_idx;
        msg_detection.corners.data()[corner_idx].x =
          detection.corners[dest_corner_idx].x;
        msg_detection.corners.data()[corner_idx].y =
          detection.corners[dest_corner_idx].y;
      }

      // center
      msg_detection.center.x = detection.center.x;
      msg_detection.center.y = detection.center.y;

      // Timestamped Pose3 transform
      geometry_msgs::msg::TransformStamped tf;
      tf.header = camera_info->header;
      tf.child_frame_id =
        std::string(tag_family_str_) + ":" + std::to_string(detection.id);
      tf.transform = ToTransformMsg(pose);
      tfs.transforms.push_back(tf);

      // Pose
      msg_detection.pose.pose.pose.position.x = tf.transform.translation.x;
      msg_detection.pose.pose.pose.position.y = tf.transform.translation.y;
      msg_detection.pose.pose.pose.position.z = tf.transform.translation.z;
      msg_detection.pose.pose.pose.orientation = tf.transform.rotation;
      msg_detections.detections.push_back(msg_detection);
    }

    node.detections_pub_->publish(msg_detections);
    node.tf_pub_->publish(tfs);

    // Cleanup
    CHECK_STATUS(vpiArrayUnlock(detections_pose_array));
    vpiArrayDestroy(detections_pose_array);

    CHECK_STATUS(vpiArrayUnlock(detections_array));
    vpiArrayDestroy(detections_array);
  }

  ~VPIAprilTagImpl()
  {
    if (IsInitialized()) {
      vpiImageDestroy(input_image_);
      vpiImageDestroy(input_monochrome_image_);
      vpiPayloadDestroy(detector_);
      vpiStreamDestroy(stream_);
    }
  }
};

struct AprilTagNode::CUAprilTagImpl : AprilTagNode::AprilTagImpl
{
  // Handle used to interface with the cuapriltags library.
  cuAprilTagsHandle detector_ = nullptr;

  // Camera intrinsics
  cuAprilTagsCameraIntrinsics_t cam_intrinsics_;

  // CUDA stream
  cudaStream_t stream_ = {};


  cuAprilTagsFamily ToCuAprilTagsFamily(const VPIAprilTagFamily & family)
  {
    switch (family) {
      case VPI_APRILTAG_36H11: return cuAprilTagsFamily::NVAT_TAG36H11;
      default: return cuAprilTagsFamily::NVAT_ENUM_SIZE;
    }
  }

  geometry_msgs::msg::Transform ToTransformMsg(const cuAprilTagsID_t & detection)
  {
    geometry_msgs::msg::Transform t;
    t.translation.x = detection.translation[0];
    t.translation.y = detection.translation[1];
    t.translation.z = detection.translation[2];

    // Rotation matrix from cuAprilTags is column major
    const Eigen::Map<const Eigen::Matrix<float, 3, 3, Eigen::ColMajor>>
    orientation(detection.orientation);
    const Eigen::Quaternion<float> q(orientation);

    t.rotation.w = q.w();
    t.rotation.x = q.x();
    t.rotation.y = q.y();
    t.rotation.z = q.z();

    return t;
  }

  std::unordered_set<VPIAprilTagFamily> SupportedTagFamilies() const override
  {
    return {VPI_APRILTAG_36H11};
  }

  void Initialize(
    const AprilTagNode & node,
    const nvidia::isaac_ros::nitros::NitrosImageView & nitros_image_view,
    const sensor_msgs::msg::CameraInfo::ConstSharedPtr & camera_info) override
  {
    AprilTagNode::AprilTagImpl::Initialize(node, nitros_image_view, camera_info);

    // Get camera intrinsics
    const double * k = camera_info->k.data();
    const float fx = static_cast<float>(k[0]);
    const float fy = static_cast<float>(k[4]);
    const float cx = static_cast<float>(k[2]);
    const float cy = static_cast<float>(k[5]);
    cam_intrinsics_ = {fx, fy, cx, cy};

    // Create AprilTags detector instance and get handle
    const int error = nvCreateAprilTagsDetector(
      &detector_, camera_info->width, camera_info->height, node.tile_size_,
      ToCuAprilTagsFamily(tag_family_), &cam_intrinsics_, node.size_);
    if (error != 0) {
      throw std::runtime_error(
              "Failed to create cuAprilTags detector (error code " +
              std::to_string(error) + ")");
    }

    // Create stream for detection
    cudaStreamCreate(&stream_);
  }

  void OnCameraFrame(
    const AprilTagNode & node,
    const nvidia::isaac_ros::nitros::NitrosImageView & nitros_image_view,
    const sensor_msgs::msg::CameraInfo::ConstSharedPtr & camera_info) override
  {
    // Check that the input image is of the expected encoding
    if (nitros_image_view.GetEncoding() != "rgb8" && nitros_image_view.GetEncoding() != "bgr8") {
      RCLCPP_ERROR(
        node.get_logger(),
        "Unsupported image encoding: %s (only 'rgb8' or 'bgr8' supported)",
        nitros_image_view.GetEncoding().c_str());
      throw std::runtime_error(
              "cuAprilTags detector only supports 'rgb8' or 'bgr8' image input");
    }

    // CUDA buffers to store the input image.
    cuAprilTagsImageInput_t input_image;
    input_image.width = nitros_image_view.GetWidth();
    input_image.height = nitros_image_view.GetHeight();
    input_image.dev_ptr =
      const_cast<uchar3 *>(reinterpret_cast<const uchar3 *>(nitros_image_view.GetGpuData()));
    input_image.pitch = nitros_image_view.GetStride();

    // Perform detection
    uint32_t num_detections;
    std::vector<cuAprilTagsID_t> tags(node.max_tags_);
    const int error = cuAprilTagsDetect(
      detector_, &input_image, tags.data(),
      &num_detections, node.max_tags_, stream_);
    if (error != 0) {
      RCLCPP_ERROR(node.get_logger(), "Failed to run AprilTags detector (error code %d)", error);
      return;
    }

    // Parse detections into published protos
    isaac_ros_apriltag_interfaces::msg::AprilTagDetectionArray msg_detections;
    msg_detections.header = camera_info->header;
    tf2_msgs::msg::TFMessage tfs;
    for (uint32_t i = 0; i < num_detections; i++) {
      const cuAprilTagsID_t & detection = tags[i];

      // detection
      isaac_ros_apriltag_interfaces::msg::AprilTagDetection msg_detection;
      msg_detection.family = tag_family_str_;
      msg_detection.id = detection.id;

      // corners
      for (int corner_idx = 0; corner_idx < 4; corner_idx++) {
        msg_detection.corners.data()[corner_idx].x =
          detection.corners[corner_idx].x;
        msg_detection.corners.data()[corner_idx].y =
          detection.corners[corner_idx].y;
      }

      // center
      const float slope_1 = (detection.corners[2].y - detection.corners[0].y) /
        (detection.corners[2].x - detection.corners[0].x);
      const float slope_2 = (detection.corners[3].y - detection.corners[1].y) /
        (detection.corners[3].x - detection.corners[1].x);
      const float intercept_1 = detection.corners[0].y -
        (slope_1 * detection.corners[0].x);
      const float intercept_2 = detection.corners[3].y -
        (slope_2 * detection.corners[3].x);
      msg_detection.center.x = (intercept_2 - intercept_1) / (slope_1 - slope_2);
      msg_detection.center.y = (slope_2 * intercept_1 - slope_1 * intercept_2) /
        (slope_2 - slope_1);

      // Timestamped Pose3 transform
      geometry_msgs::msg::TransformStamped tf;
      tf.header = camera_info->header;
      tf.child_frame_id =
        std::string(tag_family_str_) + ":" + std::to_string(detection.id);
      tf.transform = ToTransformMsg(detection);
      tfs.transforms.push_back(tf);

      // Pose
      msg_detection.pose.pose.pose.position.x = tf.transform.translation.x;
      msg_detection.pose.pose.pose.position.y = tf.transform.translation.y;
      msg_detection.pose.pose.pose.position.z = tf.transform.translation.z;
      msg_detection.pose.pose.pose.orientation = tf.transform.rotation;
      msg_detections.detections.push_back(msg_detection);
    }

    node.detections_pub_->publish(msg_detections);
    node.tf_pub_->publish(tfs);
  }

  ~CUAprilTagImpl()
  {
    if (IsInitialized()) {
      cudaStreamDestroy(stream_);
      cuAprilTagsDestroy(detector_);
    }
  }
};


AprilTagNode::AprilTagNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("apriltag_node", options),
  max_tags_(declare_parameter<int>("max_tags", 64)),
  size_(declare_parameter<double>("size", 0.22)),
  tile_size_(declare_parameter<uint16_t>("tile_size", 4)),
  tag_family_(declare_parameter<std::string>("tag_family", "tag36h11")),
  backends_{::isaac_ros::common::DeclareVPIBackendParameter(this, VPI_BACKEND_CUDA)},
  image_sub_{},
  camera_info_sub_{},
  camera_image_sync_{ExactPolicy{3}, image_sub_, camera_info_sub_},
  tf_pub_(create_publisher<tf2_msgs::msg::TFMessage>("tf", rclcpp::QoS(100))),
  detections_pub_{create_publisher<isaac_ros_apriltag_interfaces::msg::AprilTagDetectionArray>(
      "tag_detections", rclcpp::QoS(1))}
{
  // If exactly only CUDA backend was requested, use cuAprilTag, otherwise, VPI
  if (backends_ == VPI_BACKEND_CUDA) {
    RCLCPP_INFO(get_logger(), "Using cuAprilTag implementation.");
    impl_ = std::make_unique<AprilTagNode::CUAprilTagImpl>();
  } else {
    RCLCPP_INFO(get_logger(), "Using VPI implementation.");
    impl_ = std::make_unique<AprilTagNode::VPIAprilTagImpl>();
  }

  auto supported_tag_families = impl_->SupportedTagFamilies();
  if (supported_tag_families.find(ToVPIAprilTagFamily(tag_family_)) ==
    supported_tag_families.end())
  {
    std::ostringstream os{};
    os << "Tag family not supported by specified backend: '" << tag_family_ << "'" << std::endl;
    os << "'tag_family' parameter must be one of:" << std::endl;
    std::for_each(
      supported_tag_families.begin(), supported_tag_families.end(), [&os](auto & entry) {
        os << ToString(entry) << std::endl;
      });
    RCLCPP_FATAL(get_logger(), os.str().c_str());
    throw std::runtime_error(os.str());
  }

  // Setup subscripts
  camera_image_sync_.registerCallback(
    std::bind(
      &AprilTagNode::CameraImageCallback, this, std::placeholders::_1,
      std::placeholders::_2));
  image_sub_.subscribe(this, "image");
  camera_info_sub_.subscribe(this, "camera_info");
}

void AprilTagNode::CameraImageCallback(
  const nvidia::isaac_ros::nitros::NitrosImage::ConstSharedPtr & nitros_image,
  const sensor_msgs::msg::CameraInfo::ConstSharedPtr & camera_info
)
{
  auto nitros_image_view = nvidia::isaac_ros::nitros::NitrosImageView(*nitros_image);
  if (!impl_->IsInitialized()) {
    impl_->Initialize(*this, nitros_image_view, camera_info);
  }

  impl_->OnCameraFrame(*this, nitros_image_view, camera_info);
}

AprilTagNode::~AprilTagNode() = default;

}  // namespace apriltag
}  // namespace isaac_ros
}  // namespace nvidia

// Register as component
#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(nvidia::isaac_ros::apriltag::AprilTagNode)
