# Copyright (c) 2021, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.

cmake_minimum_required(VERSION 3.5)
project(isaac_ros_apriltag LANGUAGES C CXX CUDA)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Default to Release build
if(NOT CMAKE_BUILD_TYPE OR CMAKE_BUILD_TYPE STREQUAL "")
  set(CMAKE_BUILD_TYPE "Release" CACHE STRING "" FORCE)
endif()
message( STATUS "CMAKE_BUILD_TYPE: ${CMAKE_BUILD_TYPE}" )

set(CUDA_MIN_VERSION "10.2")

execute_process(COMMAND uname -m COMMAND tr -d '\n'
  OUTPUT_VARIABLE ARCHITECTURE
)
message( STATUS "Architecture: ${ARCHITECTURE}" )

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# VPI (workaround for inherited dependency from isaac_ros_image_proc)
find_package(vpi REQUIRED)

# Eigen
find_package(Eigen3 REQUIRED)
find_package(Threads REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

# CUDA
find_package(CUDA ${CUDA_MIN_VERSION} REQUIRED)
include_directories(${CUDA_INCLUDE_DIRS})
link_directories(${CUDA_TOOLKIT_ROOT_DIR}/lib64)

# NVAprilTags
include_directories(nvapriltags/nvapriltags)
add_library(nvapriltags STATIC IMPORTED)
if( ${ARCHITECTURE} STREQUAL "x86_64" )
  set_property(TARGET nvapriltags PROPERTY IMPORTED_LOCATION ${CMAKE_CURRENT_SOURCE_DIR}/nvapriltags/lib_x86_64_cuda_11_4/libapril_tagging.a)
elseif( ${ARCHITECTURE} STREQUAL "aarch64" )
  set_property(TARGET nvapriltags PROPERTY IMPORTED_LOCATION ${CMAKE_CURRENT_SOURCE_DIR}/nvapriltags/lib_aarch64_jetpack44/libapril_tagging.a)
endif()

# apriltag_node
ament_auto_add_library(apriltag_node SHARED src/apriltag_node.cpp)
target_compile_definitions(apriltag_node
  PRIVATE "COMPOSITION_BUILDING_DLL"
)
target_link_libraries(apriltag_node nvapriltags ${CUDA_LIBRARIES})
rclcpp_components_register_nodes(apriltag_node "isaac_ros::apriltag::AprilTagNode")
set(node_plugins "${node_plugins}isaac_ros::apriltag::AprilTagNode;$<TARGET_FILE:apriltag_node>\n")

# isaac_ros_apriltag executable
ament_auto_add_executable(${PROJECT_NAME}
  src/apriltag_main.cpp
)
ament_target_dependencies(${PROJECT_NAME} isaac_ros_image_proc)
target_link_libraries(${PROJECT_NAME} apriltag_node vpi)  # explicit link VPI to workaround inherited dependency

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  # Ignore copyright notices since we use custom NVIDIA Isaac ROS Software License
  set(ament_cmake_copyright_FOUND TRUE)

  ament_lint_auto_find_test_dependencies()

  find_package(launch_testing_ament_cmake REQUIRED)
  add_launch_test(test/isaac_ros_apriltag_pipeline_test.py)

endif()

ament_auto_package(INSTALL_TO_SHARE launch)
