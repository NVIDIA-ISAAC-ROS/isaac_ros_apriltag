# SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
# Copyright (c) 2023 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

project(gxf_fiducials LANGUAGES C CXX)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-fPIC -w)
endif()

# Dependencies
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(isaac_ros_nitros_april_tag_detection_array_type REQUIRED)
find_package(GXF ${ISAAC_ROS_GXF_VERSION} MODULE REQUIRED
    COMPONENTS
    core
    cuda
    multimedia
    serialization
    std
)
find_package(yaml-cpp)

# cuApriltags
add_library(cuapriltags STATIC IMPORTED)

execute_process(COMMAND uname -m COMMAND tr -d '\n' OUTPUT_VARIABLE ARCHITECTURE)
message( STATUS "Architecture: ${ARCHITECTURE}" )
if( ${ARCHITECTURE} STREQUAL "x86_64" )
  set(ARCH_GXF_PATH "lib_x86_64_cuda_11_8")
elseif( ${ARCHITECTURE} STREQUAL "aarch64" )
  set(ARCH_GXF_PATH "lib_aarch64_jetpack51")
endif()
set_property(TARGET cuapriltags PROPERTY IMPORTED_LOCATION ${CUAPRILTAGS}/${ARCH_GXF_PATH}/libcuapriltags.a)

# Fiducials extension
add_library(gxf_fiducials SHARED
  components/cuda_april_tag_detector.cpp
  fiducials.cpp
)
set(CMAKE_INCLUDE_CURRENT_DIR TRUE)
target_include_directories(gxf_fiducials
  PUBLIC
    include
  PRIVATE
    ${CUAPRILTAGS}/cuapriltags
    ${isaac_ros_nitros_april_tag_detection_array_type_INCLUDE_DIRS}
)
target_link_libraries(gxf_fiducials
  PRIVATE
    Eigen3::Eigen
    ${isaac_ros_nitros_april_tag_detection_array_type_LIBRARIES}
    cuapriltags
  PUBLIC
    GXF::cuda
    GXF::multimedia
    GXF::serialization
    GXF::std
    yaml-cpp
)
