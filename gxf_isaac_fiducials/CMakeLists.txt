# SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
# Copyright (c) 2023-2024 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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
project(gxf_isaac_fiducials LANGUAGES C CXX)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-fPIC -w)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# Dependencies
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(isaac_ros_nitros_april_tag_detection_array_type REQUIRED)
find_package(yaml-cpp)

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

# Fiducials extension
ament_auto_add_library(${PROJECT_NAME} SHARED
  gxf/extensions/fiducials/components/cuda_april_tag_detector.cpp
  gxf/extensions/fiducials/fiducials.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  "${CUAPRILTAGS}/cuapriltags"
  "${CMAKE_CURRENT_SOURCE_DIR}/gxf"
  "${isaac_ros_nitros_april_tag_detection_array_type_INCLUDE_DIRS}"
)
target_link_libraries(${PROJECT_NAME}
  Eigen3::Eigen
  ${isaac_ros_nitros_april_tag_detection_array_type_LIBRARIES}
  cuapriltags
  yaml-cpp
)

set_target_properties(${PROJECT_NAME} PROPERTIES
  BUILD_WITH_INSTALL_RPATH TRUE
  BUILD_RPATH_USE_ORIGIN TRUE
  INSTALL_RPATH_USE_LINK_PATH TRUE
)

# Install the binary file
install(TARGETS ${PROJECT_NAME} DESTINATION share/${PROJECT_NAME}/gxf/lib)

ament_auto_package(INSTALL_TO_SHARE)
