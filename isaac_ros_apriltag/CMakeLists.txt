# SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
# Copyright (c) 2021-2022 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.23.2)
project(isaac_ros_apriltag LANGUAGES C CXX)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# Dependencies
find_package(Eigen3 3.3 REQUIRED NO_MODULE)

# nvapriltags (resolve path from ament_index)
ament_index_get_resource(NVAPRILTAGS_RELATIVE_PATH nvapriltags isaac_ros_nitros)
ament_index_get_prefix_path(AMENT_INDEX_PREFIX_PATHS)
foreach(PREFIX_PATH IN LISTS AMENT_INDEX_PREFIX_PATHS)
    if(EXISTS "${PREFIX_PATH}/${NVAPRILTAGS_RELATIVE_PATH}")
      set(NVAPRILTAGS "${PREFIX_PATH}/${NVAPRILTAGS_RELATIVE_PATH}")
      break()
    endif()
endforeach()

message( STATUS "Found nvapriltags at ${NVAPRILTAGS}")

# ApriltagNode
ament_auto_add_library(apriltag_node SHARED src/apriltag_node.cpp)
rclcpp_components_register_nodes(apriltag_node "nvidia::isaac_ros::apriltag::AprilTagNode")
set(node_plugins "${node_plugins}nvidia::isaac_ros::apriltag::AprilTagNode;$<TARGET_FILE:apriltag_node>\n")
target_link_libraries(apriltag_node Eigen3::Eigen)

### Install extensions built from source

# Fiducials
add_subdirectory(gxf/fiducials)
install(TARGETS gxf_fiducials DESTINATION share/${PROJECT_NAME}/gxf/lib/fiducials)

### End extensions

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(launch_testing_ament_cmake REQUIRED)
  add_launch_test(test/isaac_ros_apriltag_pipeline_test.py)
endif()

ament_auto_package(INSTALL_TO_SHARE config launch)
