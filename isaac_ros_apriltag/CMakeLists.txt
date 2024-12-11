# SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
# Copyright (c) 2021-2024 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

cmake_minimum_required(VERSION 3.22.1)
project(isaac_ros_apriltag LANGUAGES C CXX)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# Dependencies
find_package(Eigen3 3.3 REQUIRED NO_MODULE)

# cuapriltags (resolve path from ament_index)
ament_index_get_resource(CUAPRILTAGS_RELATIVE_PATH cuapriltags isaac_ros_nitros)
ament_index_get_prefix_path(AMENT_INDEX_PREFIX_PATHS)
foreach(PREFIX_PATH IN LISTS AMENT_INDEX_PREFIX_PATHS)
    if(EXISTS "${PREFIX_PATH}/${CUAPRILTAGS_RELATIVE_PATH}")
      set(CUAPRILTAGS "${PREFIX_PATH}/${CUAPRILTAGS_RELATIVE_PATH}")
      break()
    endif()
endforeach()
message( STATUS "Found cuapriltags at ${CUAPRILTAGS}")
add_library(cuapriltags STATIC IMPORTED)
set_property(TARGET cuapriltags PROPERTY IMPORTED_LOCATION ${CUAPRILTAGS}/lib/libcuapriltags.a)
target_include_directories(cuapriltags INTERFACE "${CUAPRILTAGS}/cuapriltags")

# AprilTagNode
ament_auto_add_library(apriltag_node SHARED src/apriltag_node.cpp)
rclcpp_components_register_nodes(apriltag_node "nvidia::isaac_ros::apriltag::AprilTagNode")
set(node_plugins "${node_plugins}nvidia::isaac_ros::apriltag::AprilTagNode;$<TARGET_FILE:apriltag_node>\n")
target_link_libraries(apriltag_node Eigen3::Eigen cuapriltags)
set_target_properties(apriltag_node PROPERTIES
  BUILD_WITH_INSTALL_RPATH TRUE
  BUILD_RPATH_USE_ORIGIN TRUE
  INSTALL_RPATH_USE_LINK_PATH TRUE)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  # The FindPythonInterp and FindPythonLibs modules are removed
  if(POLICY CMP0148)
    cmake_policy(SET CMP0148 OLD)
  endif()

  # Gtest for AprilTag node test
  ament_add_gtest(apriltag_node_test test/apriltag_node_test.cpp)
  target_include_directories(apriltag_node_test PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )
  target_link_libraries(apriltag_node_test apriltag_node)

  find_package(launch_testing_ament_cmake REQUIRED)
  add_launch_test(test/isaac_ros_apriltag_pol_test.py)
  add_launch_test(test/isaac_ros_apriltag_mono8_test.py)
  add_launch_test(test/isaac_ros_apriltag_backends_compare_test.py)
endif()


# Embed versioning information into installed files
ament_index_get_resource(ISAAC_ROS_COMMON_CMAKE_PATH isaac_ros_common_cmake_path isaac_ros_common)
include("${ISAAC_ROS_COMMON_CMAKE_PATH}/isaac_ros_common-version-info.cmake")
generate_version_info(${PROJECT_NAME})

ament_auto_package(INSTALL_TO_SHARE config launch rviz)
