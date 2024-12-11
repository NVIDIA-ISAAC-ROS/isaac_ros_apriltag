// SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
// Copyright (c) 2024 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

#include <gtest/gtest.h>
#include "isaac_ros_apriltag/apriltag_node.hpp"
#include "rclcpp/rclcpp.hpp"

class AprilTagNodeTest : public ::testing::Test
{
protected:
  void SetUp() {rclcpp::init(0, nullptr);}
  void TearDown() {(void)rclcpp::shutdown();}
};

TEST_F(AprilTagNodeTest, test_invalid_tag_family)
{
  auto test = [] {
      rclcpp::NodeOptions options;
      options.arguments(
    {
      "--ros-args",
      "-p", "tag_family:=NOTHING",
    });
      try {
        nvidia::isaac_ros::apriltag::AprilTagNode node(options);
      } catch (const std::exception & e) {
        std::string err(e.what());
        if (err.find("Tag family not supported by specified backend") != std::string::npos) {
          _exit(1);
        }
      }
    };

  EXPECT_EXIT(test(), testing::ExitedWithCode(1), "");
}

TEST_F(AprilTagNodeTest, test_unsupported_tag_family)
{
  auto test = [] {
      rclcpp::NodeOptions options;
      options.arguments(
    {
      "--ros-args",
      "-p", "tag_family:=tag36h10",
      "-p", "backend:=CUDA",
    });
      try {
        nvidia::isaac_ros::apriltag::AprilTagNode node(options);
      } catch (const std::exception & e) {
        std::string err(e.what());
        if (err.find("Tag family not supported by specified backend") != std::string::npos) {
          _exit(1);
        }
      }
    };

  EXPECT_EXIT(test(), testing::ExitedWithCode(1), "");
}

TEST_F(AprilTagNodeTest, test_supported_tag_family)
{
  auto test = [] {
      rclcpp::NodeOptions options;
      options.arguments(
    {
      "--ros-args",
      "-p", "tag_family:=tag36h10",
      "-p", "backends:=CPU",
    });

      nvidia::isaac_ros::apriltag::AprilTagNode node(options);
    };

  EXPECT_NO_THROW(test());
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  ::testing::GTEST_FLAG(death_test_style) = "threadsafe";
  return RUN_ALL_TESTS();
}
