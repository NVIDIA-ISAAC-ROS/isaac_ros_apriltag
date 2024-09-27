# Isaac ROS AprilTag

NVIDIA-accelerated AprilTag detection and pose estimation.

<div align="center"><a class="reference internal image-reference" href="https://media.githubusercontent.com/media/NVIDIA-ISAAC-ROS/.github/main/resources/isaac_ros_docs/repositories_and_packages/isaac_ros_apriltag/isaac_ros_apriltag_sample_crop.gif/"><img alt="image" src="https://media.githubusercontent.com/media/NVIDIA-ISAAC-ROS/.github/main/resources/isaac_ros_docs/repositories_and_packages/isaac_ros_apriltag/isaac_ros_apriltag_sample_crop.gif/" width="550px"/></a></div>

## Overview

[Isaac ROS AprilTag](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_apriltag) contains a ROS 2 package for detection of
[AprilTags](https://april.eecs.umich.edu/software/apriltag),
a type of fiducial marker that provides a point of reference or measure.
AprilTag detections are GPU-accelerated for high performance.

<div align="center"><a class="reference internal image-reference" href="https://media.githubusercontent.com/media/NVIDIA-ISAAC-ROS/.github/main/resources/isaac_ros_docs/repositories_and_packages/isaac_ros_apriltag/isaac_ros_apriltag_nodegraph.png/"><img alt="image" src="https://media.githubusercontent.com/media/NVIDIA-ISAAC-ROS/.github/main/resources/isaac_ros_docs/repositories_and_packages/isaac_ros_apriltag/isaac_ros_apriltag_nodegraph.png/" width="800px"/></a></div>

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
the application. Each of the green nodes in the above diagram is GPU
accelerated, allowing for a high-performance compute graph from Argus
Camera to ApriTag. For USB and Ethernet cameras, the graph is
accelerated from Rectify through AprilTag

<div align="center"><a class="reference internal image-reference" href="https://media.githubusercontent.com/media/NVIDIA-ISAAC-ROS/.github/main/resources/isaac_ros_docs/repositories_and_packages/isaac_ros_apriltag/apriltagdetection_message_illustration.png/"><img alt="image" src="https://media.githubusercontent.com/media/NVIDIA-ISAAC-ROS/.github/main/resources/isaac_ros_docs/repositories_and_packages/isaac_ros_apriltag/apriltagdetection_message_illustration.png/" width="700px"/></a></div>

As illustrated above, detections are provided in an output array for the
number of AprilTag detections in the input image. Each entry in the
array contains the ID (two-dimensional bar code) for the AprilTag, the
four corners ((x0, y0), (x1, y1), (x2, y2), (x3, y3)) and center (x, y)
of the input image, and the pose of the AprilTag.

> [!Note]
> This package is a GPU-accelerated drop-in replacement for
> the [CPU version of ROS
> AprilTag](https://github.com/christianrauch/apriltag_ros)

> [!Note]
> For more information, including the paper and the reference
> CPU implementation, refer to the [AprilTag
> page](https://april.eecs.umich.edu/software/apriltag.html)

## Isaac ROS NITROS Acceleration

This package is powered by [NVIDIA Isaac Transport for ROS (NITROS)](https://developer.nvidia.com/blog/improve-perception-performance-for-ros-2-applications-with-nvidia-isaac-transport-for-ros/), which leverages type adaptation and negotiation to optimize message formats and dramatically accelerate communication between participating nodes.

## Performance

| Sample Graph<br/><br/>                                                                                                                                                              | Input Size<br/><br/>     | AGX Orin<br/><br/>                                                                                                                                                | Orin NX<br/><br/>                                                                                                                                               | Orin Nano 8GB<br/><br/>                                                                                                                                            | x86_64 w/ RTX 4060 Ti<br/><br/>                                                                                                                                     | x86_64 w/ RTX 4090<br/><br/>                                                                                                                                      |
|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|--------------------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| [AprilTag Node](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_benchmark/blob/main/benchmarks/isaac_ros_apriltag_benchmark/scripts/isaac_ros_apriltag_node.py)<br/><br/><br/><br/>   | 720p<br/><br/><br/><br/> | [244 fps](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_benchmark/blob/main/results/isaac_ros_apriltag_node-agx_orin.json)<br/><br/><br/>7.3 ms @ 30Hz<br/><br/>  | [114 fps](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_benchmark/blob/main/results/isaac_ros_apriltag_node-orin_nx.json)<br/><br/><br/>12 ms @ 30Hz<br/><br/>  | [79.2 fps](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_benchmark/blob/main/results/isaac_ros_apriltag_node-orin_nano.json)<br/><br/><br/>18 ms @ 30Hz<br/><br/>  | [596 fps](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_benchmark/blob/main/results/isaac_ros_apriltag_node-nuc_4060ti.json)<br/><br/><br/>2.4 ms @ 30Hz<br/><br/>  | [596 fps](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_benchmark/blob/main/results/isaac_ros_apriltag_node-x86_4090.json)<br/><br/><br/>2.1 ms @ 30Hz<br/><br/>  |
| [AprilTag Graph](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_benchmark/blob/main/benchmarks/isaac_ros_apriltag_benchmark/scripts/isaac_ros_apriltag_graph.py)<br/><br/><br/><br/> | 720p<br/><br/><br/><br/> | [241 fps](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_benchmark/blob/main/results/isaac_ros_apriltag_graph-agx_orin.json)<br/><br/><br/>9.5 ms @ 30Hz<br/><br/> | [109 fps](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_benchmark/blob/main/results/isaac_ros_apriltag_graph-orin_nx.json)<br/><br/><br/>15 ms @ 30Hz<br/><br/> | [74.3 fps](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_benchmark/blob/main/results/isaac_ros_apriltag_graph-orin_nano.json)<br/><br/><br/>21 ms @ 30Hz<br/><br/> | [596 fps](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_benchmark/blob/main/results/isaac_ros_apriltag_graph-nuc_4060ti.json)<br/><br/><br/>3.4 ms @ 30Hz<br/><br/> | [596 fps](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_benchmark/blob/main/results/isaac_ros_apriltag_graph-x86_4090.json)<br/><br/><br/>2.9 ms @ 30Hz<br/><br/> |

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

Update 2024-09-26: Update for ZED compatibility
