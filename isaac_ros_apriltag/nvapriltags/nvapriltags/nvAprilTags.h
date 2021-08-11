/**
 * Copyright (c) 2021, NVIDIA CORPORATION.  All rights reserved.
 *
 * NVIDIA CORPORATION and its licensors retain all intellectual property
 * and proprietary rights in and to this software, related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA CORPORATION is strictly prohibited.
 */

#ifndef ISAAC_ROS_APRILTAG__NVAPRILTAGS__NVAPRILTAGS__NVAPRILTAGS_H_
#define ISAAC_ROS_APRILTAG__NVAPRILTAGS__NVAPRILTAGS__NVAPRILTAGS_H_

#include <stdint.h>
#include <stddef.h>

#include <vector>

// Forward declaration for CUDA API
// CUstream and cudaStream_t are CUstream_st*
struct CUstream_st;

// Decoded AprilTag
typedef struct
{
  float2 corners[4];
  uint16_t id;
  uint8_t hamming_error;

  // Rotation transform, when expressed as a 3x3 matrix acting on a column vector, is column major.
  float orientation[9];

  // Translation vector from the camera, in the same units as used for the tag_size.
  float translation[3];
} nvAprilTagsID_t;

// Input data type for image buffer
typedef struct
{
  uchar4 * dev_ptr;     // Device pointer to the buffer
  size_t pitch;         // Pitch in bytes
  uint16_t width;       // Width in pixels
  uint16_t height;      // Buffer height
} nvAprilTagsImageInput_t;

typedef struct nvAprilTagsCameraIntrinsics_st
{
  float fx, fy, cx, cy;
} nvAprilTagsCameraIntrinsics_t;

typedef enum
{
  NVAT_TAG36H11,                   // Default, currently the only tag family supported
  NVAT_ENUM_SIZE = 0x7fffffff      // Force int32_t
}
nvAprilTagsFamily;

//! AprilTags Detector instance handle. Used to reference the detector after creation
typedef struct nvAprilTagsHandle_st * nvAprilTagsHandle;

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Creates and initializes an AprilTags detector instance that detects and decodes April tags
 *
 * @param hApriltags Pointer to the handle of newly created AprilTags detector
 * @param img_width Width of images to be fed in to AprilTags detector
 * @param img_height Height of images to be fed in to AprilTags detector
 * @param tag_family Enum representing the Tag Family to be detected; default NVAT_TAG36H11
 * @param cam Camera intrinsic parameters, or NULL, if the orientation and translation are not desired
 * @param tag_dim The linear dimension of the square tag. The translation will be expressed in the same units.
 * @return int 0 - Success, else - Failure
 */
int nvCreateAprilTagsDetector(
  // TODO(hemals): We usually return the result in the last parameter, not first.
  nvAprilTagsHandle * hApriltags,
  const uint32_t img_width, const uint32_t img_height,
  const nvAprilTagsFamily tag_family,
  const nvAprilTagsCameraIntrinsics_t * cam,
  float tag_dim
);

/**
 * @brief Runs the algorithms to detect potential April tags in the image and decodes valid April tags
 *
 * @param hApriltags AprilTags detector handle
 * @param img_input Input buffer containing the undistorted image on which to detect/decode April tags
 * @param tags_out C-array containing detected Tags, after detection and decoding
 * @param num_tags Number of tags detected
 * @param max_tags Maximum number of tags that can be returned, based on allocated size of tags_out array
 * @param input_stream CUDA stream on which the computation is to occur, or 0 to use the default stream
 * @return int 0 - Success, else - Failure
 */
int nvAprilTagsDetect(
  nvAprilTagsHandle hApriltags, const nvAprilTagsImageInput_t * img_input,
  nvAprilTagsID_t * tags_out, uint32_t * num_tags, const uint32_t max_tags,
  CUstream_st * input_stream);

/**
 * @brief Destroys an instance of AprilTags detector
 *
 * @param hApriltags AprilTags detector handle to be destroyed
 * @return int 0 - Success, else - Failure
 */
int nvAprilTagsDestroy(nvAprilTagsHandle hApriltags);

#ifdef __cplusplus
}
#endif

#endif  // ISAAC_ROS_APRILTAG__NVAPRILTAGS__NVAPRILTAGS__NVAPRILTAGS_H_
