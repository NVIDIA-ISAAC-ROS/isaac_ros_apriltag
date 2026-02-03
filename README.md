# Isaac ROS AprilTag

NVIDIA-accelerated AprilTag detection and pose estimation.

<div align="center"><a class="reference internal image-reference" href="https://media.githubusercontent.com/media/NVIDIA-ISAAC-ROS/.github/release-4.1/resources/isaac_ros_docs/repositories_and_packages/isaac_ros_apriltag/isaac_ros_apriltag_sample_crop.gif/"><img alt="image" src="https://media.githubusercontent.com/media/NVIDIA-ISAAC-ROS/.github/release-4.1/resources/isaac_ros_docs/repositories_and_packages/isaac_ros_apriltag/isaac_ros_apriltag_sample_crop.gif/" width="550px"/></a></div>

## Overview

Isaac ROS AprilTag contains a ROS 2 package for detection of
[AprilTags](https://github.com/AprilRobotics/apriltag),
a type of fiducial marker that provides a point of reference or measure.
AprilTag detections are NVIDIA-accelerated for high performance.

<div align="center"><a class="reference internal image-reference" href="https://media.githubusercontent.com/media/NVIDIA-ISAAC-ROS/.github/release-4.1/resources/isaac_ros_docs/repositories_and_packages/isaac_ros_apriltag/isaac_ros_apriltag_nodegraph.png/"><img alt="image" src="https://media.githubusercontent.com/media/NVIDIA-ISAAC-ROS/.github/release-4.1/resources/isaac_ros_docs/repositories_and_packages/isaac_ros_apriltag/isaac_ros_apriltag_nodegraph.png/" width="800px"/></a></div>

A common graph of nodes connects from an input camera through rectify
and resize to AprilTag. Rectify warps the input camera image into a
rectified, undistorted output image; this node may not be necessary if
the camera driver provides rectified camera images. Resize is often used
to downscale higher resolution cameras into the desired resolution for
AprilTags if needed. The input resolution to AprilTag is selected by the
required detection distance for the application, as a minimum number of
pixels are required to perform an AprilTag detection and classification.
For example, an 8mp input image of 3840×2160 may be much larger than
necessary and a 4:1 downscale to 1920x1080 could make more efficient
use of compute resources and satisfy the required detection distance of
the application. Each of the green nodes in the above diagram is NVIDIA
accelerated, allowing for a high-performance compute graph from camera
input to AprilTag detection.

<div align="center"><a class="reference internal image-reference" href="https://media.githubusercontent.com/media/NVIDIA-ISAAC-ROS/.github/release-4.1/resources/isaac_ros_docs/repositories_and_packages/isaac_ros_apriltag/apriltagdetection_message_illustration.png/"><img alt="image" src="https://media.githubusercontent.com/media/NVIDIA-ISAAC-ROS/.github/release-4.1/resources/isaac_ros_docs/repositories_and_packages/isaac_ros_apriltag/apriltagdetection_message_illustration.png/" width="700px"/></a></div>

As illustrated above, detections are provided in an output array for the
number of AprilTag detections in the input image. Each entry in the
array contains the ID (two-dimensional bar code) for the AprilTag, the
four corners ((x0, y0), (x1, y1), (x2, y2), (x3, y3)) and center (x, y)
of the input image, and the pose of the AprilTag.

> [!Note]
> This package provides the option through the `backend` parameter to leverage either the GPU or
> CPU on all NVIDIA-powered platforms or PVA on Jetson devices for AprilTag detection.

> [!Note]
> This package is a NVIDIA-accelerated drop-in replacement for
> the [CPU version of ROS
> AprilTag](https://github.com/christianrauch/apriltag_ros)

> [!Note]
> For more information, including the paper and the reference
> CPU implementation, refer to the [AprilTag
> repository](https://github.com/AprilRobotics/apriltag)

## Isaac ROS NITROS Acceleration

This package is powered by [NVIDIA Isaac Transport for ROS (NITROS)](https://developer.nvidia.com/blog/improve-perception-performance-for-ros-2-applications-with-nvidia-isaac-transport-for-ros/), which leverages type adaptation and negotiation to optimize message formats and dramatically accelerate communication between participating nodes.

## Performance

| Sample Graph<br/><br/>                                                                                                                                                           | Input Size<br/><br/>   | AGX Thor<br/><br/>                                                                                                                                                      | x86_64 w/ RTX 5090<br/><br/>                                                                                                                                             |
|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|------------------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| [AprilTag Node](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_benchmark/blob/release-4.1/benchmarks/isaac_ros_apriltag_benchmark/scripts/isaac_ros_apriltag_node.py)<br/><br/>   | 720p<br/><br/>         | [392 fps](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_benchmark/blob/release-4.1/results/isaac_ros_apriltag_node-agx_thor.json)<br/><br/><br/>10 ms @ 30Hz<br/><br/>  | [596 fps](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_benchmark/blob/release-4.1/results/isaac_ros_apriltag_node-x86-5090.json)<br/><br/><br/>2.3 ms @ 30Hz<br/><br/>  |
| [AprilTag Graph](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_benchmark/blob/release-4.1/benchmarks/isaac_ros_apriltag_benchmark/scripts/isaac_ros_apriltag_graph.py)<br/><br/> | 720p<br/><br/>         | [366 fps](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_benchmark/blob/release-4.1/results/isaac_ros_apriltag_graph-agx_thor.json)<br/><br/><br/>13 ms @ 30Hz<br/><br/> | [596 fps](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_benchmark/blob/release-4.1/results/isaac_ros_apriltag_graph-x86-5090.json)<br/><br/><br/>4.1 ms @ 30Hz<br/><br/> |

---

## Documentation

Please visit the [Isaac ROS Documentation](https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_apriltag/index.html) to learn how to use this repository.

---

## Packages

* [`isaac_ros_apriltag`](https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_apriltag/isaac_ros_apriltag/index.html)
  * [Quickstart](https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_apriltag/isaac_ros_apriltag/index.html#quickstart)
  * [Try More Examples](https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_apriltag/isaac_ros_apriltag/index.html#try-more-examples)
  * [Troubleshooting](https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_apriltag/isaac_ros_apriltag/index.html#troubleshooting)
  * [API](https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_apriltag/isaac_ros_apriltag/index.html#api)

## Latest

Update 2026-02-02: Support for two new Docker-optional development and deployment modes
