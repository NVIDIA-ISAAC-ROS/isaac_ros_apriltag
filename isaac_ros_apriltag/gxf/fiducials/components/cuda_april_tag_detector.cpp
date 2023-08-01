// SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
// Copyright (c) 2021-2023 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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
#include "extensions/fiducials/components/cuda_april_tag_detector.hpp"

#include <cuda_runtime.h>
#include <string>
#include <utility>
#include <vector>

#include "cuAprilTags.h"
#include "engine/core/image/image.hpp"
#include "engine/gems/image/utils.hpp"
#include "extensions/fiducials/messages/fiducial_message.hpp"
#include "gxf/multimedia/video.hpp"
#include "gxf/std/timestamp.hpp"

namespace nvidia {
namespace isaac {

namespace {

// Gets corner points as a tensor
gxf::Expected<gxf::Tensor> CornersToTensor(const float2 corners[4],
                                           gxf::Handle<gxf::Allocator> allocator) {
  gxf::Tensor tensor;
  return tensor.reshape<double>(gxf::Shape{4, 2}, gxf::MemoryStorageType::kHost, allocator)
      .and_then([&]() { return tensor.data<double>(); })
      .map([&](double* points) {
        const int stride = tensor.shape().dimension(0);
        for (int i = 0; i < stride; i++) {
          points[i] = corners[i].y;
          points[i + stride] = corners[i].x;
        }
        return std::move(tensor);
      });
}

// Converts pose from april tag detection library to Pose3
::isaac::Pose3d DetectionToPose3d(const float translation[3], const float rot_flat[9]) {
  // Rotation matrix from cuAprilTags is column major
  ::isaac::Matrix3d rot_matrix;
  rot_matrix << rot_flat[0], rot_flat[3], rot_flat[6],
                rot_flat[1], rot_flat[4], rot_flat[7],
                rot_flat[2], rot_flat[5], rot_flat[8];
  return ::isaac::Pose3d{
    ::isaac::SO3d::FromQuaternion(::isaac::Quaterniond(rot_matrix)),
    ::isaac::Vector3d(translation[0], translation[1], translation[2])
  };
}

}  // namespace

struct CudaAprilTagDetector::AprilTagData {
  // Handle used to interface with the stereo library.
  cuAprilTagsHandle april_tags_handle;
  // CUDA buffers to store the input image.
  cuAprilTagsImageInput_t input_image;
  // Camera intrinsics
  cuAprilTagsCameraIntrinsics_t cam_intrinsics;
  // Output vector of detected Tags
  std::vector<cuAprilTagsID_t> tags;
  // CUDA stream
  cudaStream_t main_stream;
};

CudaAprilTagDetector::CudaAprilTagDetector() {}

CudaAprilTagDetector::~CudaAprilTagDetector() {}

gxf_result_t CudaAprilTagDetector::registerInterface(gxf::Registrar* registrar) {
  if (!registrar) { return GXF_ARGUMENT_NULL; }
  gxf::Expected<void> result;
  result &= registrar->parameter(
      camera_image_, "camera_image", "Camera Image",
      "Undistorted RGB camera image with intrinsics");
  result &= registrar->parameter(
      april_tags_, "april_tags", "April Tags",
      "AprilTag fiducial");
  result &= registrar->parameter(
      allocator_, "allocator", "Allocator",
      "Memory allocator for keypoints tensor");
  result &= registrar->parameter(
      max_tags_, "max_tags", "Max Tags",
      "Maximum number of AprilTags that can be detected",
      50);
  result &= registrar->parameter(
      tile_size_, "tile_size", "Tile Size",
      "Tile/window size used for adaptive thresholding in pixels",
      4U);
  result &= registrar->parameter(
      tag_dimensions_, "tag_dimensions", "Tag Dimensions",
      "AprilTag tag dimensions",
      0.18);
  result &= registrar->parameter(
      tag_family_, "tag_family", "Tag Family",
      "AprilTag tag family",
      std::string("tag36h11"));
  result &= registrar->parameter(
      video_buffer_name_, "video_buffer_name", "Video Buffer Name",
      "Name of the VideoBuffer component to use",
      gxf::Registrar::NoDefaultParameter(), GXF_PARAMETER_FLAGS_OPTIONAL);
  result &= registrar->parameter(
      camera_model_name_, "camera_model_name", "Camera Model Name",
      "Name of the CameraModel component to use",
      gxf::Registrar::NoDefaultParameter(), GXF_PARAMETER_FLAGS_OPTIONAL);
  return gxf::ToResultCode(result);
}

gxf_result_t CudaAprilTagDetector::initialize() {
  if (tag_family_.get() != "tag36h11") {
    GXF_LOG_ERROR("CudaAprilTagDetector only supports tag36h11 AprilTags");
    return GXF_PARAMETER_OUT_OF_RANGE;
  }
  impl_ = MakeUniqueNoThrow<AprilTagData>();
  return impl_ != nullptr ? GXF_SUCCESS : GXF_OUT_OF_MEMORY;
}

gxf_result_t CudaAprilTagDetector::deinitialize() {
  impl_ = nullptr;
  return GXF_SUCCESS;
}

gxf_result_t CudaAprilTagDetector::tick() {
  // Receive message entity
  auto entity = camera_image_->receive();
  if (!entity) {
    return gxf::ToResultCode(entity);
  }

  // Get frame
  auto video_buffer_name = video_buffer_name_.try_get();
  const char* frame_name = video_buffer_name ? video_buffer_name->c_str() : nullptr;
  auto frame = entity->get<gxf::VideoBuffer>(frame_name);
  if (!frame) {
    return gxf::ToResultCode(frame);
  }

  // Get intrinsics
  auto camera_model_name = camera_model_name_.try_get();
  const char* intrinsics_name = camera_model_name ? camera_model_name->c_str() : nullptr;
  auto intrinsics = entity->get<gxf::CameraModel>(intrinsics_name);
  if (!intrinsics) {
    return gxf::ToResultCode(intrinsics);
  }

  if (impl_->april_tags_handle == nullptr) {
    // Initalize AprilTag library parameters
    auto result = createAprilTagDetector(intrinsics.value());
    if (!result) {
      return gxf::ToResultCode(result);
    }
  }

  if (frame.value()->video_frame_info().color_format !=
      gxf::VideoFormat::GXF_VIDEO_FORMAT_RGB) {
    GXF_LOG_ERROR("CudaAprilTagDetector only supports RGB images");
    return GXF_FAILURE;
  }

  if (frame.value()->storage_type() != gxf::MemoryStorageType::kDevice) {
    GXF_LOG_ERROR("CudaAprilTagDetector only supports CUDA images");
    return GXF_FAILURE;
  }

  // Avoid memory copy by running detection on image view
  impl_->input_image.width = frame.value()->video_frame_info().width;
  impl_->input_image.height = frame.value()->video_frame_info().height;
  impl_->input_image.pitch = frame.value()->video_frame_info().color_planes[0].stride;
  impl_->input_image.dev_ptr = reinterpret_cast<uchar3*>(frame.value()->pointer());

  // Run AprilTags detection
  uint32_t num_tags;
  const int error = cuAprilTagsDetect(impl_->april_tags_handle, &(impl_->input_image),
                                      impl_->tags.data(), &num_tags, max_tags_, impl_->main_stream);
  if (error != 0) {
    GXF_LOG_ERROR("Call to cuAprilTagsDetect failed with error code %d", error);
    return GXF_FAILURE;
  }

  return gxf::ToResultCode(
      CreateFiducialListMessage(context(), num_tags)
      .map([&](FiducialListMessageParts message) -> gxf::Expected<void> {
        for (uint32_t i = 0; i < num_tags; i++) {
          const cuAprilTagsID_t& tag = impl_->tags[i];
          message.info[i].value()->type = FiducialInfo::Type::kAprilTag;
          message.info[i].value()->id = tag_family_.get() + "_" + std::to_string(tag.id);
          auto keypoints = CornersToTensor(tag.corners, allocator_);
          if (!keypoints) {
            return gxf::ForwardError(keypoints);
          }
          *message.keypoints[i].value() = std::move(keypoints.value());
          *message.pose[i].value() = DetectionToPose3d(tag.translation, tag.orientation);
        }
        return april_tags_->publish(message.entity);
      }));
}

gxf_result_t CudaAprilTagDetector::stop() {
  if (impl_->april_tags_handle != nullptr) {
    cudaStreamDestroy(impl_->main_stream);
    cuAprilTagsDestroy(impl_->april_tags_handle);
  }
  return GXF_SUCCESS;
}

gxf::Expected<void> CudaAprilTagDetector::createAprilTagDetector(
    gxf::Handle<gxf::CameraModel> intrinsics) {
  if (!intrinsics) {
    return gxf::Unexpected{GXF_ARGUMENT_NULL};
  }

  // Get camera intrinsics
  const float fx = intrinsics->focal_length.x;
  const float fy = intrinsics->focal_length.y;
  const float cx = intrinsics->principal_point.x;
  const float cy = intrinsics->principal_point.y;
  impl_->cam_intrinsics = {fx, fy, cx, cy};

  // Create AprilTags detector instance and get handle
  const int error = nvCreateAprilTagsDetector(&(impl_->april_tags_handle),
                                              intrinsics->dimensions.x, intrinsics->dimensions.y,
                                              tile_size_, cuAprilTagsFamily::NVAT_TAG36H11,
                                              &(impl_->cam_intrinsics), tag_dimensions_);
  if (error != 0) {
    GXF_LOG_ERROR("Call to nvCreateAprilTagsDetector failed with error code %d", error);
    return gxf::Unexpected{GXF_FAILURE};
  }

  // Create stream for detection
  cudaStreamCreate(&(impl_->main_stream));

  // Allocate the output vector to contain detected AprilTags.
  impl_->tags.resize(max_tags_);

  return gxf::Success;
}

}  // namespace isaac
}  // namespace nvidia
