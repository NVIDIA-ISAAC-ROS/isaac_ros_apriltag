/*
 * Copyright (c) 2021, NVIDIA CORPORATION.  All rights reserved.
 *
 * NVIDIA CORPORATION and its licensors retain all intellectual property
 * and proprietary rights in and to this software, related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA CORPORATION is strictly prohibited.
 */

#include <cv_bridge/cv_bridge.h>

#include <AprilTagNode.hpp>
#include <Eigen/Dense>
#include <image_transport/image_transport.hpp>

#include "cuda.h"          // NOLINT
#include "cuda_runtime.h"  // NOLINT
#include "nvAprilTags.h"

namespace {
geometry_msgs::msg::Transform ToTransformMsg(const nvAprilTagsID_t &detection) {
  geometry_msgs::msg::Transform t;
  t.translation.x = detection.translation[0];
  t.translation.y = detection.translation[1];
  t.translation.z = detection.translation[2];

  // Rotation matrix from nvAprilTags is column major
  const Eigen::Map<const Eigen::Matrix<float, 3, 3, Eigen::ColMajor>>
      orientation(detection.orientation);
  const Eigen::Quaternion<float> q(orientation);

  t.rotation.w = q.w();
  t.rotation.x = q.x();
  t.rotation.y = q.y();
  t.rotation.z = q.z();

  return t;
}
}  // namespace

struct AprilTagNode::AprilTagsImpl {
  // Handle used to interface with the stereo library.
  nvAprilTagsHandle april_tags_handle = nullptr;

  // Camera intrinsics
  nvAprilTagsCameraIntrinsics_t cam_intrinsics;

  // Output vector of detected Tags
  std::vector<nvAprilTagsID_t> tags;

  // CUDA stream
  cudaStream_t main_stream = {};

  // CUDA buffers to store the input image.
  nvAprilTagsImageInput_t input_image;

  // CUDA memory buffer container for RGBA images.
  char *input_image_buffer = nullptr;

  // Size of image buffer
  size_t input_image_buffer_size = 0;

  void initialize(const AprilTagNode &node, const uint32_t width,
                  const uint32_t height, const size_t image_buffer_size,
                  const size_t pitch_bytes,
                  const sensor_msgs::msg::CameraInfo::ConstSharedPtr &msg_ci) {
    assert(april_tags_handle == nullptr && "Already initialized.");

    // Get camera intrinsics
    const double *k = msg_ci->k.data();
    const float fx = static_cast<float>(k[0]);
    const float fy = static_cast<float>(k[4]);
    const float cx = static_cast<float>(k[2]);
    const float cy = static_cast<float>(k[5]);
    cam_intrinsics = {fx, fy, cx, cy};

    // Create AprilTags detector instance and get handle
    const int error = nvCreateAprilTagsDetector(
        &april_tags_handle, width, height, nvAprilTagsFamily::NVAT_TAG36H11,
        &cam_intrinsics, node.tag_edge_size_);
    if (error != 0) {
      throw std::runtime_error(
          "Failed to create NV April Tags detector (error code " +
          std::to_string(error) + ")");
    }

    // Create stream for detection
    cudaStreamCreate(&main_stream);

    // Allocate the output vector to contain detected AprilTags.
    tags.resize(node.max_tags_);

    // Setup input image CUDA buffer.
    const cudaError_t cuda_error =
        cudaMalloc(&input_image_buffer, image_buffer_size);
    if (cuda_error != cudaSuccess) {
      throw std::runtime_error("Could not allocate CUDA memory (error code " +
                               std::to_string(cuda_error) + ")");
    }

    // Setup input image.
    input_image_buffer_size = image_buffer_size;
    input_image.width = width;
    input_image.height = height;
    input_image.dev_ptr = reinterpret_cast<uchar4 *>(input_image_buffer);
    input_image.pitch = pitch_bytes;
  }

  ~AprilTagsImpl() {
    if (april_tags_handle != nullptr) {
      cudaStreamDestroy(main_stream);
      nvAprilTagsDestroy(april_tags_handle);
      cudaFree(input_image_buffer);
    }
  }
};

AprilTagNode::AprilTagNode(rclcpp::NodeOptions options)
    : Node("apriltag", "apriltag", options.use_intra_process_comms(true)),
      // parameter
      tag_family_(declare_parameter<std::string>("family", "36h11")),
      tag_edge_size_(declare_parameter<double>("size", 2.0)),
      max_tags_(declare_parameter<int>("max_tags", 20)),
      // topics
      sub_cam_(image_transport::create_camera_subscription(
          this, "image",
          std::bind(&AprilTagNode::onCameraFrame, this, std::placeholders::_1,
                    std::placeholders::_2),
          declare_parameter<std::string>("image_transport", "raw"),
          rmw_qos_profile_sensor_data)),
      pub_tf_(
          create_publisher<tf2_msgs::msg::TFMessage>("/tf", rclcpp::QoS(100))),
      pub_detections_(
          create_publisher<apriltag_msgs::msg::AprilTagDetectionArray>(
              "detections", rclcpp::QoS(1))),
      impl_(std::make_unique<AprilTagsImpl>()) {}

void AprilTagNode::onCameraFrame(
    const sensor_msgs::msg::Image::ConstSharedPtr &msg_img,
    const sensor_msgs::msg::CameraInfo::ConstSharedPtr &msg_ci) {
  // Convert frame to 8-bit RGBA image
  const cv::Mat img_rgba8 = cv_bridge::toCvShare(msg_img, "rgba8")->image;

  // Setup detector on first frame
  if (impl_->april_tags_handle == nullptr) {
    impl_->initialize(*this, img_rgba8.cols, img_rgba8.rows,
                      img_rgba8.total() * img_rgba8.elemSize(), img_rgba8.step,
                      msg_ci);
  }

  // Copy frame into CUDA buffer
  const cudaError_t cuda_error =
      cudaMemcpy(impl_->input_image_buffer, img_rgba8.ptr(),
                 impl_->input_image_buffer_size, cudaMemcpyHostToDevice);
  if (cuda_error != cudaSuccess) {
    throw std::runtime_error(
        "Could not memcpy to device CUDA memory (error code " +
        std::to_string(cuda_error) + ")");
  }

  // Perform detection
  uint32_t num_detections;
  const int error = nvAprilTagsDetect(
      impl_->april_tags_handle, &(impl_->input_image), impl_->tags.data(),
      &num_detections, max_tags_, impl_->main_stream);
  if (error != 0) {
    throw std::runtime_error("Failed to run AprilTags detector (error code " +
                             std::to_string(error) + ")");
  }

  // Parse detections into published protos
  apriltag_msgs::msg::AprilTagDetectionArray msg_detections;
  msg_detections.header = msg_img->header;
  tf2_msgs::msg::TFMessage tfs;
  for (int i = 0; i < num_detections; i++) {
    const nvAprilTagsID_t &detection = impl_->tags[i];

    // detection
    apriltag_msgs::msg::AprilTagDetection msg_detection;
    msg_detection.family = tag_family_;
    msg_detection.id = detection.id;
    msg_detection.hamming = detection.hamming_error;

    // corners
    for (int corner_idx = 0; corner_idx < 4; corner_idx++) {
      msg_detection.corners.data()[corner_idx].x =
          detection.corners[corner_idx].x;
      msg_detection.corners.data()[corner_idx].y =
          detection.corners[corner_idx].y;
    }
    msg_detections.detections.push_back(msg_detection);

    // Timestamped Pose3 transform
    geometry_msgs::msg::TransformStamped tf;
    tf.header = msg_img->header;
    tf.child_frame_id =
        std::string(tag_family_) + ":" + std::to_string(detection.id);
    tf.transform = ToTransformMsg(detection);
    tfs.transforms.push_back(tf);
  }

  pub_detections_->publish(msg_detections);
  pub_tf_->publish(tfs);
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(AprilTagNode)
