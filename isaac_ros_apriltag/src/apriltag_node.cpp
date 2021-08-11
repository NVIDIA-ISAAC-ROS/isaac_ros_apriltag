/**
 * Copyright (c) 2021, NVIDIA CORPORATION.  All rights reserved.
 *
 * NVIDIA CORPORATION and its licensors retain all intellectual property
 * and proprietary rights in and to this software, related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA CORPORATION is strictly prohibited.
 */

#include "isaac_ros_apriltag/apriltag_node.hpp"

#include <memory>
#include <string>
#include <vector>

#include "cuda.h"  // NOLINT - include .h without directory
#include "cuda_runtime.h"  // NOLINT - include .h without directory
#include "cv_bridge/cv_bridge.h"
#include "eigen3/Eigen/Dense"
#include "image_transport/image_transport.hpp"
#include "message_filters/subscriber.h"
#include "message_filters/synchronizer.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "nvAprilTags.h"
#include "rclcpp/logger.hpp"

namespace
{
geometry_msgs::msg::Transform ToTransformMsg(const nvAprilTagsID_t & detection)
{
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

namespace isaac_ros
{
namespace apriltag
{

struct AprilTagNode::AprilTagsImpl
{
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
  char * input_image_buffer = nullptr;

  // Size of image buffer
  size_t input_image_buffer_size = 0;

  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image,
      sensor_msgs::msg::CameraInfo> ApproximatePolicy;
  typedef message_filters::Synchronizer<ApproximatePolicy> Synchronizer;
  std::shared_ptr<Synchronizer> sync;

  void initialize(
    const AprilTagNode & node, const uint32_t width,
    const uint32_t height, const size_t image_buffer_size,
    const size_t pitch_bytes,
    const sensor_msgs::msg::CameraInfo::ConstSharedPtr & msg_ci)
  {
    assert(april_tags_handle == nullptr && "Already initialized.");

    // Get camera intrinsics
    const double * k = msg_ci->k.data();
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
      throw std::runtime_error(
              "Could not allocate CUDA memory (error code " +
              std::to_string(cuda_error) + ")");
    }

    // Setup input image.
    input_image_buffer_size = image_buffer_size;
    input_image.width = width;
    input_image.height = height;
    input_image.dev_ptr = reinterpret_cast<uchar4 *>(input_image_buffer);
    input_image.pitch = pitch_bytes;
  }

  ~AprilTagsImpl()
  {
    if (april_tags_handle != nullptr) {
      cudaStreamDestroy(main_stream);
      nvAprilTagsDestroy(april_tags_handle);
      cudaFree(input_image_buffer);
    }
  }
};


AprilTagNode::AprilTagNode(rclcpp::NodeOptions options)
: Node("apriltag", options.use_intra_process_comms(true)),
  // parameter
  tag_family_(declare_parameter<std::string>("family", "36h11")),
  tag_edge_size_(declare_parameter<double>("size", 2.0)),
  max_tags_(declare_parameter<int>("max_tags", 20)),
  queue_size_(declare_parameter<int>("queue_size", 100)),
  // Setting max interval to 100s to account for sync issues.
  max_interval_duration_(declare_parameter<int32_t>("max_interval_duration", 100)),
  // topics
  image_sub_(message_filters::Subscriber<sensor_msgs::msg::Image>(
      this,
      "camera/image_rect")),
  camera_info_sub_(message_filters::Subscriber<sensor_msgs::msg::CameraInfo>(
      this,
      "camera/camera_info")),
  pub_tf_(
    create_publisher<tf2_msgs::msg::TFMessage>("tf", rclcpp::QoS(100))),
  pub_detections_(
    create_publisher<isaac_ros_apriltag_interfaces::msg::AprilTagDetectionArray>(
      "tag_detections", rclcpp::QoS(1))),
  impl_(std::make_unique<AprilTagsImpl>())
{
  impl_->sync.reset(
    new AprilTagsImpl::Synchronizer(
      AprilTagsImpl::ApproximatePolicy(queue_size_),
      image_sub_, camera_info_sub_));
  impl_->sync->setMaxIntervalDuration(rclcpp::Duration(max_interval_duration_, 0));
  impl_->sync->registerCallback(
    std::bind(
      &AprilTagNode::onCameraFrame, this, std::placeholders::_1,
      std::placeholders::_2));
}

void AprilTagNode::onCameraFrame(
  const sensor_msgs::msg::Image::ConstSharedPtr & msg_img,
  const sensor_msgs::msg::CameraInfo::ConstSharedPtr & msg_ci)
{
  // Convert frame to 8-bit RGBA image
  const cv::Mat img_rgba8 = cv_bridge::toCvShare(msg_img, "rgba8")->image;

  // Setup detector on first frame
  if (impl_->april_tags_handle == nullptr) {
    impl_->initialize(
      *this, img_rgba8.cols, img_rgba8.rows,
      img_rgba8.total() * img_rgba8.elemSize(), img_rgba8.step,
      msg_ci);
  }

  // Copy frame into CUDA buffer
  const cudaError_t cuda_error =
    cudaMemcpy(
    impl_->input_image_buffer, img_rgba8.ptr(),
    impl_->input_image_buffer_size, cudaMemcpyHostToDevice);
  if (cuda_error != cudaSuccess) {
    RCLCPP_ERROR(
      this->get_logger(),
      "Could not memcpy to device CUDA memory (error code %s)", std::to_string(cuda_error));
    return;
  }

  // Perform detection
  uint32_t num_detections;
  const int error = nvAprilTagsDetect(
    impl_->april_tags_handle, &(impl_->input_image), impl_->tags.data(),
    &num_detections, max_tags_, impl_->main_stream);
  if (error != 0) {
    RCLCPP_ERROR(this->get_logger(), "Failed to run AprilTags detector (error code %d)", error);
    return;
  }

  // Parse detections into published protos
  isaac_ros_apriltag_interfaces::msg::AprilTagDetectionArray msg_detections;
  msg_detections.header = msg_img->header;
  tf2_msgs::msg::TFMessage tfs;
  for (uint32_t i = 0; i < num_detections; i++) {
    const nvAprilTagsID_t & detection = impl_->tags[i];

    // detection
    isaac_ros_apriltag_interfaces::msg::AprilTagDetection msg_detection;
    msg_detection.family = tag_family_;
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
    tf.header = msg_img->header;
    tf.child_frame_id =
      std::string(tag_family_) + ":" + std::to_string(detection.id);
    tf.transform = ToTransformMsg(detection);
    tfs.transforms.push_back(tf);

    // Pose
    msg_detection.pose.pose.pose.position.x = tf.transform.translation.x;
    msg_detection.pose.pose.pose.position.y = tf.transform.translation.y;
    msg_detection.pose.pose.pose.position.z = tf.transform.translation.z;
    msg_detection.pose.pose.pose.orientation = tf.transform.rotation;
    msg_detections.detections.push_back(msg_detection);
  }

  pub_detections_->publish(msg_detections);
  pub_tf_->publish(tfs);
}

AprilTagNode::~AprilTagNode() = default;

}  // namespace apriltag
}  // namespace isaac_ros

// Register as a component
#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(isaac_ros::apriltag::AprilTagNode)
