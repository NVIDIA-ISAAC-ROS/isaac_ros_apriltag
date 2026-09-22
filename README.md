# Isaac ROS AprilTag

NVIDIA-accelerated AprilTag detection and pose estimation.

<div align="center"><a class="reference internal image-reference" href="https://media.githubusercontent.com/media/NVIDIA-ISAAC-ROS/.github/release-5.0/resources/isaac_ros_docs/repositories_and_packages/isaac_ros_apriltag/isaac_ros_apriltag_sample_crop.gif/"><img alt="image" src="https://media.githubusercontent.com/media/NVIDIA-ISAAC-ROS/.github/release-5.0/resources/isaac_ros_docs/repositories_and_packages/isaac_ros_apriltag/isaac_ros_apriltag_sample_crop.gif/" width="550px"/></a></div>

## Overview

Isaac ROS AprilTag contains a ROS 2 package for detection of
[AprilTags](https://github.com/AprilRobotics/apriltag),
a type of fiducial marker that provides a point of reference or measure.
AprilTag detections are NVIDIA-accelerated for high performance.

<div align="center"><a class="reference internal image-reference" href="https://media.githubusercontent.com/media/NVIDIA-ISAAC-ROS/.github/release-5.0/resources/isaac_ros_docs/repositories_and_packages/isaac_ros_apriltag/isaac_ros_apriltag_nodegraph.png/"><img alt="image" src="https://media.githubusercontent.com/media/NVIDIA-ISAAC-ROS/.github/release-5.0/resources/isaac_ros_docs/repositories_and_packages/isaac_ros_apriltag/isaac_ros_apriltag_nodegraph.png/" width="800px"/></a></div>

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

<div align="center"><a class="reference internal image-reference" href="https://media.githubusercontent.com/media/NVIDIA-ISAAC-ROS/.github/release-5.0/resources/isaac_ros_docs/repositories_and_packages/isaac_ros_apriltag/apriltagdetection_message_illustration.png/"><img alt="image" src="https://media.githubusercontent.com/media/NVIDIA-ISAAC-ROS/.github/release-5.0/resources/isaac_ros_docs/repositories_and_packages/isaac_ros_apriltag/apriltagdetection_message_illustration.png/" width="700px"/></a></div>

As illustrated above, detections are provided in an output array for the
number of AprilTag detections in the input image. Each entry in the
array contains the ID (two-dimensional bar code) for the AprilTag, the
four corners ((x0, y0), (x1, y1), (x2, y2), (x3, y3)) and center (x, y)
of the input image, and the pose of the AprilTag.

This package is a NVIDIA-accelerated drop-in replacement for
the [CPU version of ROS AprilTag](https://github.com/christianrauch/apriltag_ros).
For more information, including the paper and the reference
CPU implementation, refer to the [AprilTag repository](https://github.com/AprilRobotics/apriltag).

The `backend` parameter allows you to leverage either the CPU,
GPU on all NVIDIA-powered platforms, or PVA on Jetson devices for AprilTag detection. Below is the table of supported tag families by backend.

#### AprilTag Tag Families Supported by Backend

| Tag Family      | CUDA   | CPU   | PVA   |
|-----------------|--------|-------|-------|
| `tag36h11`      | ✓      | ✓     | ✓     |
| `tag16h5`       |        | ✓     | ✓     |
| `tag25h9`       |        | ✓     | ✓     |
| `tag36h10`      |        | ✓     | ✓     |
| `circle21h7`    |        | ✓     | ✓     |
| `circle49h12`   |        | ✓     | ✓     |
| `custom48h12`   |        | ✓     | ✓     |
| `standard41h12` |        | ✓     | ✓     |
| `standard52h13` |        | ✓     | ✓     |

## ROS 2 Native `rosidl::Buffer` Acceleration

This package uses `rosidl::Buffer`, a feature built into ROS 2 Lyrical, to
avoid unnecessary copies of large payloads between CPU and accelerator
memory. The CUDA buffer backend builds on this native ROS 2 feature to provide
CUDA memory storage and transport. Most applications can use standard ROS
messages and conversion packages without depending directly on a buffer
backend. See [rosidl::Buffer and Buffer Backends](https://nvidia-isaac-ros.github.io/concepts/rosidl_buffer/index.html) for details.

## Performance

| Sample Graph<br/><br/>                                                                                                                                                           | Input Size<br/><br/>   | AGX Thor T5000<br/><br/>                                                                                                                                                 | AGX Thor T4000<br/><br/>                                                                                                                                                   | AGX Orin<br/><br/>                                                                                                                                                       | Orin Nano Super 8GB<br/><br/>                                                                                                                                            | DGX Spark<br/><br/>                                                                                                                                                       | x86_64 w/ RTX 5090<br/><br/>                                                                                                                                             | x86_64 w/ RTX 5070<br/><br/>                                                                                                                                                |
|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|------------------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| [AprilTag Node](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_benchmark/blob/release-5.0/benchmarks/isaac_ros_apriltag_benchmark/scripts/isaac_ros_apriltag_node.py)<br/><br/>   | 720p<br/><br/>         | [326 fps](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_benchmark/blob/release-5.0/results/isaac_ros_apriltag_node-agx_thor.json)<br/><br/><br/>3.2 ms @ 30Hz<br/><br/>  | [248 fps](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_benchmark/blob/release-5.0/results/isaac_ros_apriltag_node-thor-t4000.json)<br/><br/><br/>4.3 ms @ 30Hz<br/><br/>  | [189 fps](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_benchmark/blob/release-5.0/results/isaac_ros_apriltag_node-agx_orin.json)<br/><br/><br/>5.3 ms @ 30Hz<br/><br/>  | [104 fps](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_benchmark/blob/release-5.0/results/isaac_ros_apriltag_node-orin_nano.json)<br/><br/><br/>9.6 ms @ 30Hz<br/><br/> | [555 fps](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_benchmark/blob/release-5.0/results/isaac_ros_apriltag_node-dgx_spark.json)<br/><br/><br/>1.5 ms @ 30Hz<br/><br/>  | [596 fps](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_benchmark/blob/release-5.0/results/isaac_ros_apriltag_node-x86-5090.json)<br/><br/><br/>1.0 ms @ 30Hz<br/><br/>  | [591 fps](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_benchmark/blob/release-5.0/results/isaac_ros_apriltag_node-x86-rtx5070.json)<br/><br/><br/>1.4 ms @ 30Hz<br/><br/>  |
| [AprilTag Graph](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_benchmark/blob/release-5.0/benchmarks/isaac_ros_apriltag_benchmark/scripts/isaac_ros_apriltag_graph.py)<br/><br/> | 720p<br/><br/>         | [306 fps](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_benchmark/blob/release-5.0/results/isaac_ros_apriltag_graph-agx_thor.json)<br/><br/><br/>3.6 ms @ 30Hz<br/><br/> | [236 fps](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_benchmark/blob/release-5.0/results/isaac_ros_apriltag_graph-thor-t4000.json)<br/><br/><br/>5.0 ms @ 30Hz<br/><br/> | [186 fps](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_benchmark/blob/release-5.0/results/isaac_ros_apriltag_graph-agx_orin.json)<br/><br/><br/>6.5 ms @ 30Hz<br/><br/> | [100 fps](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_benchmark/blob/release-5.0/results/isaac_ros_apriltag_graph-orin_nano.json)<br/><br/><br/>12 ms @ 30Hz<br/><br/> | [539 fps](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_benchmark/blob/release-5.0/results/isaac_ros_apriltag_graph-dgx_spark.json)<br/><br/><br/>2.3 ms @ 30Hz<br/><br/> | [596 fps](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_benchmark/blob/release-5.0/results/isaac_ros_apriltag_graph-x86-5090.json)<br/><br/><br/>1.7 ms @ 30Hz<br/><br/> | [596 fps](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_benchmark/blob/release-5.0/results/isaac_ros_apriltag_graph-x86-rtx5070.json)<br/><br/><br/>2.3 ms @ 30Hz<br/><br/> |

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

Update 2026-09-21: Migrated the AprilTag detection node from NITROS to rosidl::Buffer with the CUDA buffer backend
