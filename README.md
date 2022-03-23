# Isaac ROS Apriltag

<div align="center"><img src="resources/isaac_ros_apriltag_sample_crop.jpg" width="400px"/></div>
    
## Overview
This ROS2 node uses the NVIDIA GPU-accelerated AprilTags library to detect AprilTags in images and publishes their poses, IDs, and additional metadata. This has been tested on ROS2 (Foxy) and should build and run on x86_64 and aarch64 (Jetson). It is modeled after and comparable to the ROS2 node for [CPU AprilTags detection](https://github.com/christianrauch/apriltag_ros.git).

For more information on the Isaac GEM that this node is based off of, see the latest Isaac SDK documentation [here](https://docs.nvidia.com/isaac/isaac/packages/fiducials/doc/apriltags.html).

For more information on AprilTags themselves, including the paper and the reference CPU implementation, click [here](https://april.eecs.umich.edu/software/apriltag.html).

## System Requirements
This Isaac ROS package is designed and tested to be compatible with ROS2 Foxy on Jetson hardware.

### Jetson
- [Jetson AGX Xavier or Xavier NX](https://www.nvidia.com/en-us/autonomous-machines/embedded-systems/)
- [JetPack 4.6.1](https://developer.nvidia.com/embedded/jetpack)

### x86_64
- Ubuntu 20.04+
- CUDA 11.4 supported discrete GPU
- VPI 1.1.11


**Note:** For best performance on Jetson, ensure that power settings are configured appropriately ([Power Management for Jetson](https://docs.nvidia.com/jetson/l4t/index.html#page/Tegra%20Linux%20Driver%20Package%20Development%20Guide/power_management_jetson_xavier.html#wwpID0EUHA)).

### Docker
Precompiled ROS2 Foxy packages are not available for JetPack 4.6 (based on Ubuntu 18.04 Bionic). You can either manually compile ROS2 Foxy and required dependent packages from source or use the Isaac ROS development Docker image from [Isaac ROS Common](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common).

You must first install the [Nvidia Container Toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html) to make use of the Docker container development/runtime environment.

Configure `nvidia-container-runtime` as the default runtime for Docker by editing `/etc/docker/daemon.json` to include the following:
```
    "runtimes": {
        "nvidia": {
            "path": "nvidia-container-runtime",
            "runtimeArgs": []
        }
    },
    "default-runtime": "nvidia"
```
and then restarting Docker: `sudo systemctl daemon-reload && sudo systemctl restart docker`

Run the following script in `isaac_ros_common` to build the image and launch the container:

`$ scripts/run_dev.sh <optional path>`

You can either provide an optional path to mirror in your host ROS workspace with Isaac ROS packages, which will be made available in the container as `/workspaces/isaac_ros-dev`, or you can setup a new workspace in the container.

### Package Dependencies
- [isaac_ros_common](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common)
- [isaac_ros_image_pipeline](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_image_pipeline)
- [image_common](https://github.com/ros-perception/image_common.git)
- [vision_cv](https://github.com/ros-perception/vision_opencv.git)
- [OpenCV 4.5+](https://opencv.org/)

**Note:** `isaac_ros_common` is used for running tests and/or creating a development container. It also contains VPI Debian packages that can be installed natively on a development machine without the container.

## Quickstart
1. Create a ROS2 workspace if one is not already prepared:  
`mkdir -p your_ws/src`  
**Note:** The workspace can have any name; the quickstart assumes you name it `your_ws`.
2. Clone this package repository to `your_ws/src/isaac_ros_apriltag`. Check that you have [Git LFS](https://git-lfs.github.com/) installed before cloning to pull down all large files.  
`sudo apt-get install git-lfs`  
`cd your_ws/src && git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_apriltag`
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

## Tutorial with a real camera
1. Complete the Quickstart steps above.
2. Connect a compatible camera to your Jetson and set up the camera publisher stream. Your camera vendor may offer a specific ROS2-compatible camera driver package. Alternatively, many generic cameras are compatible with the `v4l2_camera` package.  
**Important:** Ensure that the camera stream publishes `Image` and `CameraInfo` pairs to the topics `/image_raw` and `/camera_info`, respectively.
3. Ensure that your workspace has been built and sourced, if you have not done so already:  
`cd your_ws && colcon build --symlink-install && source install/setup.bash`
4. Finally, launch the pre-composed pipeline launchfile:  
`ros2 launch isaac_ros_apriltag isaac_ros_apriltag_pipeline.launch.py`

## Tutorial with Isaac Sim
1. Complete the Quickstart steps above.
2. Ensure that your workspace has been built and sourced, if you have not done so already:  
`cd your_ws && colcon build --symlink-install && source install/setup.bash`
3. Launch the pre-composed pipeline launchfile:  
`ros2 launch isaac_ros_apriltag isaac_ros_apriltag_isaac_sim_pipeline.launch.py`
4. Make sure you have Isaac Sim [set up](https://docs.omniverse.nvidia.com/app_isaacsim/app_isaacsim.html#setting-up-isaac-sim) correctly and choose the appropriate working environment[[Native](https://docs.omniverse.nvidia.com/app_isaacsim/app_isaacsim/setup.html)/[Docker&Cloud](https://docs.omniverse.nvidia.com/app_isaacsim/app_isaacsim/setup.html#docker-cloud-deployment)]. For this walkthrough, we are using the native workstation setup for Isaac Sim.
5. See [Running For The First Time](https://docs.omniverse.nvidia.com/app_isaacsim/app_isaacsim/first_run.html#) section to launch Isaac Sim from the [app launcher](https://docs.omniverse.nvidia.com/app_isaacsim/app_isaacsim/user_interface_launcher.html) and click on the **Isaac Sim** button.
6. Set up the Isaac Sim ROS2 bridge as described [here](https://docs.omniverse.nvidia.com/app_isaacsim/app_isaacsim/ext_omni_isaac_ros_bridge.html#ros2-bridge).
7. Connect to the Nucleus server as shown in the [Getting Started](https://docs.omniverse.nvidia.com/app_isaacsim/app_isaacsim/sample_jetbot.html#getting-started) section if you have not done it already.
8. Open up the Isaac ROS Common USD scene located at:
   
   `omniverse://<your_nucleus_server>/Isaac/Samples/ROS/Scenario/carter_warehouse_apriltags_worker.usd`.
   
   And wait for it to load completely.
9. Press **Play** to start publishing data from Isaac Sim.
<div align="center"><img src="resources/Isaac_sim_april_tag.png" width="800px"/></div>

10. In a separate terminal, run RViz to visualize the apriltag detections:<br>
    `rviz2`
11.  Add the tf tree in the **Displays** RViz panel. <br> <div align="center"><img src="resources/Rviz_add_tf.png" width="600px"/></div>
12.  Set the **Fixed frame** in the **Global Options** to *chassis_link*. <br> <div align="center"><img src="resources/Rviz_fixed_frame.png" width="300px"/></div>
13.  You should see the pose of the tags in RVIZ: <br> <div align="center"><img src="resources/Rviz_apriltag_output.png" width="800px"/></div>
14.  If you prefer to observe the Apriltag output in a text mode, on a separate terminal, echo the contents of the `/tag_detections` topic with the following command:   
`ros2 topic echo /tag_detections` <br> <div align="center"><img src="resources/Terminal_output.png" width="600px"/></div>

## Tutorial with Isaac Sim with Hardware in the loop (HIL)

The following instructions are for a setup where we can run the sample on a Jetson device and Isaac Sim on an x86 machine. We will use the ROS_DOMAIN_ID environment variable to have a separate logical network for Isaac Sim and the sample application. 

NOTE: Before executing any of the ROS commands, make sure to set the ROS_DOMAIN_ID variable first.

1. Complete step 4 of [Tutorial with Isaac Sim](#tutorial-with-isaac-sim) section if you have not done it already.
2. Open the location of the Isaac Sim package in the terminal by clicking the [**Open in Terminal**](https://docs.omniverse.nvidia.com/app_isaacsim/app_isaacsim/user_interface_launcher.html) button.
   
   <div align="center"><img src="resources/Isaac_sim_app_launcher.png" width="400px"/></div>
3. In the terminal opened by the previous step, set the ROS_DOMAIN_ID as shown:

   `export ROS_DOMAIN_ID=<some_number>`

4. Launch Isaac Sim from the script as shown:
   
   `./isaac-sim.sh` 
   <div align="center"><img src="resources/Isaac_sim_app_terminal.png" width="600px"/></div>
5. Continue with step 6 of [Tutorial with Isaac Sim](#tutorial-with-isaac-sim) section. Make sure to set the ROS_DOMAIN_ID variable before running the sample application.


## Next Steps
Now that you have successfully launched the full Isaac ROS Apriltag pipeline, you can easily adapt the provided launchfile to integrate with your existing ROS2 environment. 

Alternatively, since the `AprilTagNode` is provided as a ROS2 Component, you can also compose the accelerated Apriltag processor directly into an existing executable.

# Package Reference
## `isaac_ros_apriltag`
### Overview
The `isaac_ros_apriltag` package offers functionality for detecting poses from AprilTags in the frame. It largely replaces the `apriltag_ros` package, though an included dependency on the `ImageFormatConverterNode` plugin of the `isaac_ros_image_proc` package also functions as a way to replace the CPU-based image format conversion in `cv_bridge`.
### Available Components
| Component      | Topics Subscribed                                                  | Topics Published                                                       | Parameters                                                                                                                                                                                                                                   |
| -------------- | ------------------------------------------------------------------ | ---------------------------------------------------------------------- | -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `AprilTagNode` | `camera/image_rect`, `camera/camera_info`: The input camera stream | `tag_detections`: The detection message array <br> `tf`: The tag poses | `family`: The tag family for the detector (this value can only be `36h11` at this time) <br> `size`: The tag edge size in meters, assuming square markers <br> `max_tags`: The maximum number of tags to be detected, which is 20 by default |

# Updates

| Date       | Changes                                                                                 |
| ---------- | --------------------------------------------------------------------------------------- |
| 2021-11-15 | Isaac Sim HIL documentation update                                                      |
| 2021-11-15 | Added launch file to work with Isaac Sim                                                |
| 2021-10-20 | Migrated to [NVIDIA-ISAAC-ROS](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_apriltag)  |
| 2021-08-11 | Initial release to [NVIDIA-AI-IOT](https://github.com/NVIDIA-AI-IOT/isaac_ros_apriltag) |
