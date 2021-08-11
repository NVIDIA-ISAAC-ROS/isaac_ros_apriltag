# `isaac_ros_apriltag`

## Overview
This ROS2 node uses the NVIDIA GPU-accelerated AprilTags library to detect AprilTags in images and publishes their poses, IDs, and additional metadata. This has been tested on ROS2 (Foxy) and should build and run on x86_64 and aarch64 (Jetson). It is modeled after and comparable to the ROS2 node for [CPU AprilTags detection](https://github.com/christianrauch/apriltag_ros.git).

For more information on the Isaac GEM that this node is based off of, see the latest Isaac SDK documentation [here](https://docs.nvidia.com/isaac/isaac/packages/fiducials/doc/apriltags.html).

For more information on AprilTags themselves, including the paper and the reference CPU implementation, click [here](https://april.eecs.umich.edu/software/apriltag.html).

## System Requirements
This Isaac ROS package is designed and tested to be compatible with ROS2 Foxy on Jetson hardware.
### Jetson
- AGX Xavier or Xavier NX
- JetPack 4.6

### x86_64
- CUDA 10.2+ supported discrete GPU
- VPI 1.1.11
- Ubuntu 18.04+

### Docker
Precompiled ROS2 Foxy packages are not available for JetPack 4.6 (based on Ubuntu 18.04 Bionic). You can either manually compile ROS2 Foxy and required dependent packages from source or use the Isaac ROS development Docker image from [Isaac ROS Common](https://github.com/NVIDIA-AI-IOT/isaac_ros_common) based on images from [jetson-containers](https://github.com/dusty-nv/jetson-containers).

Run the following script in `isaac_ros_common` to build the image and launch the container:

`$ scripts/run_dev.sh <optional path>`

You can either provide an optional path to mirror in your host ROS workspace with Isaac ROS packages, which will be made available in the container as `/workspaces/isaac_ros-dev`, or you can setup a new workspace in the container.

### Package Dependencies
- [isaac_ros_common](https://github.com/NVIDIA-AI-IOT/isaac_ros_common)
- [isaac_ros_image_pipeline](https://github.com/NVIDIA-AI-IOT/isaac_ros_image_pipeline)
- [image_common](https://github.com/ros-perception/image_common.git)
- [vision_cv](https://github.com/ros-perception/vision_opencv.git)
- [OpenCV 4.5+](https://opencv.org/)

**Note:** `isaac_ros_common' is used for running tests and/or creating a development container. It also contains VPI Debian packages that can be installed natively on a development machine without the container.

## Quickstart
1. Create a ROS2 workspace if one is not already prepared:  
`mkdir -p your_ws/src`  
**Note:** The workspace can have any name; the quickstart assumes you name it `your_ws`.
2. Clone this package repository to `your_ws/src/isaac_ros_apriltag`. Check that you have [Git LFS](https://git-lfs.github.com/) installed before cloning to pull down all large files.  
`cd your_ws/src && git clone https://github.com/NVIDIA-AI-IOT/isaac_ros_apriltag`
3. Build and source the workspace:  
`cd your_ws && colcon build --symlink-install && source install/setup.bash`
4. (Optional) Run tests to verify complete and correct installation:  
`colcon test`
5. Start `isaac_ros_apriltag` using the prebuilt executable:  
`ros2 run isaac_ros_apriltag isaac_ros_apriltag`
6. In a separate terminal, spin up a **calibrated** camera publisher to `/image_rect` and `/camera_info` using any package (for example, `v4l2_camera`):  
`ros2 run v4l2_camera v4l2_camera_node --ros-args -r /image_raw:=/image_rect`
7. Observe the AprilTag detection output `/tag_detections` on a separate terminal with the command:   
`ros2 topic echo /tag_detections`

### Configuration
You will need to calibrate the intrinsics of your camera if you want the node to determine 3D poses for tags instead of just detection and corners as 2D pixel coordinates. See [here](https://navigation.ros.org/tutorials/docs/camera_calibration.html) for more details.

### Replacing `apriltag_ros` with `isaac_ros_apriltag`
1. Add a dependency on `isaac_ros_apriltag` to `your_package/package.xml` and `your_package/CMakeLists.txt`. The original `apriltag_ros` dependency may be removed entirely.
2. Change the package and plugin names in any `*.launch.py` launch files to use `isaac_ros_apriltag` and `AprilTagNode`, respectively.

## See Also
- `isaac_ros_image_pipeline`: Accelerated metapackage offering similar functionality to the standard CPU-based `image_pipeline` metapackage
- `isaac_ros_common`: Utilities for robust ROS2 testing, in conjunction with `launch_test`

# Isaac ROS Apriltag Pipeline Tutorial
## Objective
This tutorial will help you quickly run and experiment with the full Isaac ROS Apriltag pipeline, from camera frames to tag detections.

## Tutorial
1. Complete the Quickstart steps above.
2. Connect a compatible camera to your Jetson and set up the camera publisher stream. Your camera vendor may offer a specific ROS2-compatible camera driver package. Alternatively, many generic cameras are compatible with the `v4l2_camera` package.  
**Important:** Ensure that the camera stream publishes `Image` and `CameraInfo` pairs to the topics `/image_raw` and `/camera_info`, respectively.
3. Ensure that your workspace has been built and sourced, if you have not done so already:  
`cd your_ws && colcon build --symlink-install && source install/setup.bash`
4. Finally, launch the pre-composed pipeline launchfile:  
`ros2 launch isaac_ros_apriltag isaac_ros_apriltag_pipeline.launch.py`

Detections will show up at `/tag_detections`.

**Note** For best performance on Jetson, ensure that power settings are configured appropriately ([Power Management for Jetson](https://docs.nvidia.com/jetson/l4t/index.html#page/Tegra%20Linux%20Driver%20Package%20Development%20Guide/power_management_jetson_xavier.html#wwpID0EUHA)).

## Next Steps
Now that you have successfully launched the full Isaac ROS Apriltag pipeline, you can easily adapt the provided launchfile to integrate with your existing ROS2 environment. 

Alternatively, since the `AprilTagNode` is provided as a ROS2 Component, you can also compose the accelerated Apriltag processor directly into an existing executable.

# Package Reference
## `isaac_ros_apriltag`
### Overview
The `isaac_ros_apriltag` package offers functionality for detecting poses from AprilTags in the frame. It largely replaces the `apriltag_ros` package, though an included dependency on the `ImageFormatConverterNode` plugin of the `isaac_ros_image_proc` package also functions as a way to replace the CPU-based image format conversion in `cv_bridge`.
### Available Components
| Component      | Topics Subscribed                                                  | Topics Published                                                       | Parameters                                                                                                                                                                                                               |
| -------------- | ------------------------------------------------------------------ | ---------------------------------------------------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ |
| `AprilTagNode` | `camera/image_rect`, `camera/camera_info`: The input camera stream | `tag_detections`: The detection message array <br> `tf`: The tag poses | `family`: The tag family for the detector (this value can only be `36h11` at this time) <br> `size`: The tag edge size in meters, assuming square markers <br> `max_tags`: The maximum number of tags to be detected, which is 20 by default |
