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
#pragma once

#include <memory>
#include <string>

#include "gxf/multimedia/camera.hpp"
#include "gxf/std/allocator.hpp"
#include "gxf/std/codelet.hpp"
#include "gxf/std/receiver.hpp"
#include "gxf/std/transmitter.hpp"

namespace nvidia {
namespace isaac {

// Detects AprilTags in images with cuAprilTags library
class CudaAprilTagDetector : public gxf::Codelet {
 public:
  // Explicitly declare constructors and destructors
  // to get around forward declaration of AprilTagsData
  CudaAprilTagDetector();
  ~CudaAprilTagDetector();

  gxf_result_t registerInterface(gxf::Registrar* registrar) override;
  gxf_result_t initialize() override;
  gxf_result_t deinitialize() override;

  gxf_result_t start() override { return GXF_SUCCESS; }
  gxf_result_t tick() override;
  gxf_result_t stop() override;

 private:
  // Initializes AprilTag detector with camera intrinsics
  gxf::Expected<void> createAprilTagDetector(gxf::Handle<gxf::CameraModel> intrinsics);

  gxf::Parameter<gxf::Handle<gxf::Receiver>> camera_image_;
  gxf::Parameter<gxf::Handle<gxf::Transmitter>> april_tags_;
  gxf::Parameter<gxf::Handle<gxf::Allocator>> allocator_;
  gxf::Parameter<int> max_tags_;
  gxf::Parameter<double> tag_dimensions_;
  gxf::Parameter<std::string> tag_family_;
  gxf::Parameter<std::string> video_buffer_name_;
  gxf::Parameter<std::string> camera_model_name_;

  // Hide the AprilTagData implementation details
  struct AprilTagData;
  std::unique_ptr<AprilTagData> impl_;
};

}  // namespace isaac
}  // namespace nvidia
