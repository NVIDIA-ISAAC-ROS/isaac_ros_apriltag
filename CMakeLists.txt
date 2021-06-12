# Copyright (c) 2021, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.

cmake_minimum_required(VERSION 3.5)

project(nvapriltags_ros2 LANGUAGES C CXX CUDA)

set(CMAKE_CXX_STANDARD 14)
set(CUDA_MIN_VERSION "10.2")

EXECUTE_PROCESS(COMMAND uname -m COMMAND tr -d '\n' OUTPUT_VARIABLE ARCHITECTURE)
message( STATUS "Architecture: ${ARCHITECTURE}" )

find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(cv_bridge REQUIRED)

# Eigen
find_package(Eigen3 REQUIRED)
find_package(Threads REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

# CUDA
find_package(CUDA ${CUDA_MIN_VERSION} REQUIRED)
include_directories(${CUDA_INCLUDE_DIRS})

include_directories(include)

link_directories(${CUDA_TOOLKIT_ROOT_DIR}/lib64)

# NVAprilTags
include_directories(nvapriltags/nvapriltags)
add_library(nvapriltags STATIC IMPORTED)
if( ${ARCHITECTURE} STREQUAL "x86_64" )
    set_property(TARGET nvapriltags PROPERTY IMPORTED_LOCATION ${CMAKE_CURRENT_SOURCE_DIR}/nvapriltags/lib_x86_64/libapril_tagging.a)
elseif( ${ARCHITECTURE} STREQUAL "aarch64" )
    set_property(TARGET nvapriltags PROPERTY IMPORTED_LOCATION ${CMAKE_CURRENT_SOURCE_DIR}/nvapriltags/lib_aarch64_jetpack44/libapril_tagging.a)
endif()

# Msg
find_package(rosidl_default_generators REQUIRED)

# AprilTagNode
add_library(AprilTagNode SHARED src/AprilTagNode.cpp)
add_dependencies(AprilTagNode nvapriltags)
ament_target_dependencies(AprilTagNode rclcpp rclcpp_components sensor_msgs tf2_msgs image_transport cv_bridge)
target_link_libraries(AprilTagNode nvapriltags ${CUDA_LIBRARIES})
rclcpp_components_register_nodes(AprilTagNode "AprilTagNode")

ament_environment_hooks(${ament_cmake_package_templates_ENVIRONMENT_HOOK_LIBRARY_PATH})

install(TARGETS AprilTagNode
    EXPORT export_${PROJECT_NAME}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

set(msg_files
    "msg/AprilTagDetection.msg"
    "msg/AprilTagDetectionArray.msg"
)
rosidl_generate_interfaces(${PROJECT_NAME}
    ${msg_files}
    DEPENDENCIES std_msgs geometry_msgs
)

rosidl_target_interfaces(AprilTagNode
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

ament_package()
