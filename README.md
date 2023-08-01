# Isaac ROS Apriltag

<div align="center"><img alt="Isaac ROS AprilTag Sample Output" src="resources/isaac_ros_apriltag_sample_crop.gif" width="550px"/></div>

## Overview

Isaac ROS AprilTag contains a ROS 2 package for detection of [AprilTags](https://april.eecs.umich.edu/software/apriltag#:~:text=AprilTag%20is%20a%20visual%20fiducial,tags%20relative%20to%20the%20camera.), a type of fiducial marker that provides a point of reference or measure. AprilTag detections are GPU-accelerated for high performance.

<div align="center"><img alt="graph of nodes with AprilTag" src="resources/isaac_ros_apriltag_nodegraph.png" width="800px"/></div>

A common graph of nodes connects from an input camera through rectify and resize to AprilTag. Rectify warps the input camera image into a rectified, undistorted output image; this node may not be necessary if the camera driver provides rectified camera images. Resize is often used to downscale higher resolution cameras into the desired resolution for AprilTags if needed. The input resolution to AprilTag is selected by the required detection distance for the application, as a minimum number of pixels are required to perform an AprilTag detection and classification. For example, an 8mp input image of 3840×2160 may be much larger than necessary and a 4:1 downscale to 1920x1080 could make more efficienct use of compute resources and satisfy the required detection distance of the application. Each of the green nodes in the above diagram is GPU accelerated, allowing for a high-performance compute graph from Argus Camera to ApriTag. For USB and Ethernet cameras, the graph is accelerated from Rectify through AprilTag

<div align="center"><img alt="visual of AprilTag output message information" src="resources/apriltagdetection_message_illustration.png" width="700px"/></div>

As illustrated above, detections are provided in an output array for the number of AprilTag detections in the input image.  Each entry in the array contains the ID (two-dimensional bar code) for the AprilTag, the four corners ((x0, y0), (x1, y1), (x2, y2), (x3, y3)) and center (x, y) of the input image, and the pose of the AprilTag.

> **Note**: This package is a GPU-accelerated drop-in replacement for the [CPU version of ROS Apriltag](https://github.com/christianrauch/apriltag_ros)
<!-- Split blockquote -->
> **Note**: For more information, including the paper and the reference CPU implementation, refer to the [AprilTag page](https://april.eecs.umich.edu/software/apriltag.html)

### Isaac ROS NITROS Acceleration

This package is powered by [NVIDIA Isaac Transport for ROS (NITROS)](https://developer.nvidia.com/blog/improve-perception-performance-for-ros-2-applications-with-nvidia-isaac-transport-for-ros/), which leverages type adaptation and negotiation to optimize message formats and dramatically accelerate communication between participating nodes.

## Performance

The following table summarizes the per-platform performance statistics of sample graphs that use this package, with links included to the full benchmark output. These benchmark configurations are taken from the [Isaac ROS Benchmark](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_benchmark#list-of-isaac-ros-benchmarks) collection, based on the [`ros2_benchmark`](https://github.com/NVIDIA-ISAAC-ROS/ros2_benchmark) framework.

| Sample Graph                                                                                                                     | Input Size | AGX Orin                                                                                                                                     | Orin NX                                                                                                                                      | Orin Nano 8GB                                                                                                                                  | x86_64 w/ RTX 4060 Ti                                                                                                                          |
| -------------------------------------------------------------------------------------------------------------------------------- | ---------- | -------------------------------------------------------------------------------------------------------------------------------------------- | -------------------------------------------------------------------------------------------------------------------------------------------- | ---------------------------------------------------------------------------------------------------------------------------------------------- | ---------------------------------------------------------------------------------------------------------------------------------------------- |
| [AprilTag Node](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_benchmark/blob/main/scripts//isaac_ros_apriltag_node.py)   | 720p       | [110 fps](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_benchmark/blob/main/results/isaac_ros_apriltag_node-agx_orin.json)<br>15 ms  | [69.6 fps](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_benchmark/blob/main/results/isaac_ros_apriltag_node-orin_nx.json)<br>18 ms  | [50.1 fps](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_benchmark/blob/main/results/isaac_ros_apriltag_node-orin_nano.json)<br>25 ms  | [262 fps](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_benchmark/blob/main/results/isaac_ros_apriltag_node-nuc_4060ti.json)<br>9.3 ms |
| [AprilTag Graph](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_benchmark/blob/main/scripts//isaac_ros_apriltag_graph.py) | 720p       | [143 fps](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_benchmark/blob/main/results/isaac_ros_apriltag_graph-agx_orin.json)<br>16 ms | [82.9 fps](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_benchmark/blob/main/results/isaac_ros_apriltag_graph-orin_nx.json)<br>22 ms | [58.0 fps](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_benchmark/blob/main/results/isaac_ros_apriltag_graph-orin_nano.json)<br>31 ms | [349 fps](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_benchmark/blob/main/results/isaac_ros_apriltag_graph-nuc_4060ti.json)<br>11 ms |


## Table of Contents

- [Isaac ROS Apriltag](#isaac-ros-apriltag)
  - [Overview](#overview)
    - [Isaac ROS NITROS Acceleration](#isaac-ros-nitros-acceleration)
  - [Performance](#performance)
  - [Table of Contents](#table-of-contents)
  - [Latest Update](#latest-update)
  - [Supported Platforms](#supported-platforms)
    - [Docker](#docker)
  - [Quickstart](#quickstart)
    - [Configuration](#configuration)
  - [Next Steps](#next-steps)
    - [Try More Examples](#try-more-examples)
    - [Try NITROS-Accelerated Graph with Argus](#try-nitros-accelerated-graph-with-argus)
    - [Customize your Dev Environment](#customize-your-dev-environment)
  - [Replacing `apriltag_ros` with `isaac_ros_apriltag`](#replacing-apriltag_ros-with-isaac_ros_apriltag)
    - [Supported Packages](#supported-packages)
  - [Package Reference](#package-reference)
    - [`isaac_ros_apriltag`](#isaac_ros_apriltag)
      - [Usage](#usage)
      - [ROS Parameters](#ros-parameters)
      - [ROS Topics Subscribed](#ros-topics-subscribed)
      - [ROS Topics Published](#ros-topics-published)
  - [Troubleshooting](#troubleshooting)
    - [Isaac ROS Troubleshooting](#isaac-ros-troubleshooting)
  - [Updates](#updates)

## Latest Update

Update 2023-05-25: Performance improvements.

## Supported Platforms

This package is designed and tested to be compatible with ROS 2 Humble running on [Jetson](https://developer.nvidia.com/embedded-computing) or an x86_64 system with an NVIDIA GPU.

> **Note**: Versions of ROS 2 earlier than Humble are **not** supported. This package depends on specific ROS 2 implementation features that were only introduced beginning with the Humble release.

| Platform | Hardware                                                                                                                                                                                                 | Software                                                                                                           | Notes                                                                                                                                                                                   |
| -------- | -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | ------------------------------------------------------------------------------------------------------------------ | --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| Jetson   | [Jetson Orin](https://www.nvidia.com/en-us/autonomous-machines/embedded-systems/jetson-orin/) <br> [Jetson Xavier](https://www.nvidia.com/en-us/autonomous-machines/embedded-systems/jetson-agx-xavier/) | [JetPack 5.1.1](https://developer.nvidia.com/embedded/jetpack)                                                     | For best performance, ensure that [power settings](https://docs.nvidia.com/jetson/archives/r34.1/DeveloperGuide/text/SD/PlatformPowerAndPerformance.html) are configured appropriately. |
| x86_64   | NVIDIA GPU                                                                                                                                                                                               | [Ubuntu 20.04+](https://releases.ubuntu.com/20.04/) <br> [CUDA 11.8+](https://developer.nvidia.com/cuda-downloads) |

### Docker

To simplify development, we strongly recommend leveraging the Isaac ROS Dev Docker images by following [these steps](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common/blob/main/docs/dev-env-setup.md). This will streamline your development environment setup with the correct versions of dependencies on both Jetson and x86_64 platforms.

> **Note**: All Isaac ROS Quickstarts, tutorials, and examples have been designed with the Isaac ROS Docker images as a prerequisite.

## Quickstart

1. Set up your development environment by following the instructions [here](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common/blob/main/docs/dev-env-setup.md).
2. Clone this repository and its dependencies under `~/workspaces/isaac_ros-dev/src`.

    ```bash
    cd ~/workspaces/isaac_ros-dev/src
    ```

    ```bash
    git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common
    ```

    ```bash
    git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_nitros
    ```

    ```bash
    git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_apriltag
    ```

    ```bash
    git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_image_pipeline
    ```

3. Pull down a ROS Bag of sample data:

    ```bash
    cd ~/workspaces/isaac_ros-dev/src/isaac_ros_apriltag && \ 
      git lfs pull -X "" -I "resources/rosbags/quickstart.bag"
    ```

4. Launch the Docker container using the `run_dev.sh` script:

    ```bash
    cd ~/workspaces/isaac_ros-dev/src/isaac_ros_common && \
      ./scripts/run_dev.sh
    ```

5. Inside the container, build and source the workspace:  

    ```bash
    cd /workspaces/isaac_ros-dev && \
      colcon build --symlink-install && \
      source install/setup.bash
    ```

6. (Optional) Run tests to verify complete and correct installation:  

    ```bash
    colcon test --executor sequential
    ```

7. Run the following launch files to spin up a demo of this package:

    ```bash
    ros2 launch isaac_ros_apriltag isaac_ros_apriltag.launch.py
    ```

8. Open a **second** terminal inside the docker container:

    ```bash
    cd ~/workspaces/isaac_ros-dev/src/isaac_ros_common && \
      ./scripts/run_dev.sh
    ```

9. Run the rosbag file to simulate an image stream:

    ```bash
    ros2 bag play --loop src/isaac_ros_apriltag/resources/rosbags/quickstart.bag
    ```

10. Open a **third** terminal inside the docker container:

    ```bash
    cd ~/workspaces/isaac_ros-dev/src/isaac_ros_common && \
      ./scripts/run_dev.sh
    ```

11. Observe the AprilTag detection output `/tag_detections` on a separate terminal with the command:

    ```bash
    source install/setup.bash && \
    ros2 topic echo /tag_detections
    ```

### Configuration

You will need to calibrate the intrinsics of your camera if you want the node to determine 3D poses for tags instead of just detection and corners as 2D pixel coordinates. See [here](https://navigation.ros.org/tutorials/docs/camera_calibration.html) for more details.

## Next Steps

### Try More Examples

To continue your exploration, check out the following suggested examples:

- [Tutorial with a USB camera](./docs/tutorial-usb-cam.md)
- [Tutorial with Isaac Sim](./docs/tutorial-isaac-sim.md)

### Try NITROS-Accelerated Graph with Argus

If you have an Argus-compatible camera, you can launch the NITROS-accelerated graph by following the [tutorial](./docs/tutorial-nitros-graph.md).

### Customize your Dev Environment

To customize your development environment, reference [this guide](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common/blob/main/docs/modify-dockerfile.md).

## Replacing `apriltag_ros` with `isaac_ros_apriltag`

1. Add a dependency on `isaac_ros_apriltag` to `your_package/package.xml` and `your_package/CMakeLists.txt`. The original `apriltag_ros` dependency may be removed entirely.
2. Change the package and plugin names in any `*.launch.py` launch files to use `isaac_ros_apriltag` and `nvidia::isaac_ros::apriltag::AprilTagNode`, respectively.

### Supported Packages

At this time, the packages under the standard `apriltag_ros` have the following support:

| Existing Package | Isaac ROS Alternative          |
| ---------------- | ------------------------------ |
| `apriltag_ros`   | See `isaac_ros_apriltag`       |
| `image_pipeline` | See `isaac_ros_image_pipeline` |

## Package Reference

### `isaac_ros_apriltag`

#### Usage

```bash
ros2 launch isaac_ros_apriltag isaac_ros_apriltag.launch.py --ros-args -p size:=<size of tag (meters)> -p max_tags:=<max tag count> -p tile_size:=<adaptive thresholding tile size(px)>
```

#### ROS Parameters

| ROS Parameter | Type     | Default | Description                                                              |
| ------------- | -------- | ------- | ------------------------------------------------------------------------ |
| `size`        | `double` | `0.22`  | The tag edge size in meters, assuming square markers. <br> E.g. `0.22`   |
| `max_tags`    | `int`    | `64`    | The maximum number of tags to be detected. <br> E.g. `64`                |
| `tile_size`   | `uint`   | `4`     | Tile/window size used for adaptive thresholding in pixels. <br> E.g. `4` |

#### ROS Topics Subscribed

| ROS Topic     | Interface                                                                                                      | Description                         |
| ------------- | -------------------------------------------------------------------------------------------------------------- | ----------------------------------- |
| `image`       | [sensor_msgs/Image](https://github.com/ros2/common_interfaces/blob/humble/sensor_msgs/msg/Image.msg)           | The input camera stream.            |
| `camera_info` | [sensor_msgs/CameraInfo](https://github.com/ros2/common_interfaces/blob/humble/sensor_msgs/msg/CameraInfo.msg) | The input camera intrinsics stream. |

#### ROS Topics Published

| ROS Topic        | Type                                                                                                                                                                                | Description                                                                      |
| ---------------- | ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | -------------------------------------------------------------------------------- |
| `tag_detections` | [isaac_ros_apriltag_interfaces/AprilTagDetectionArray](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common/blob/main/isaac_ros_apriltag_interfaces/msg/AprilTagDetectionArray.msg) | The detection message array.                                                     |
| `tf`             | [tf2_msgs/TFMessage](https://github.com/ros2/geometry2/blob/ros2/tf2_msgs/msg/TFMessage.msg)                                                                                        | Pose of all detected apriltags(`TagFamily:ID`) wrt to the camera topic frame_id. |

## Troubleshooting

### Isaac ROS Troubleshooting

For solutions to problems with Isaac ROS, please check [here](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common/blob/main/docs/troubleshooting.md).

## Updates

| Date       | Changes                                                                                 |
| ---------- | --------------------------------------------------------------------------------------- |
| 2023-05-25 | Performance improvements                                                                |
| 2023-04-05 | Source available GXF extensions                                                         |
| 2022-10-19 | Updated OSS licensing                                                                   |
| 2022-08-31 | Update to be compatible with JetPack 5.0.2                                              |
| 2022-06-30 | Update to use NITROS for improved performance                                           |
| 2021-11-15 | Isaac Sim HIL documentation update                                                      |
| 2021-11-15 | Added launch file to work with Isaac Sim                                                |
| 2021-10-20 | Migrated to [NVIDIA-ISAAC-ROS](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_apriltag)  |
| 2021-08-11 | Initial release to [NVIDIA-AI-IOT](https://github.com/NVIDIA-AI-IOT/isaac_ros_apriltag) |
