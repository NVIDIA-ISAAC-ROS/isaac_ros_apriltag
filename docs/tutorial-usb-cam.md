# Tutorial with a Real Camera

A graph consisting of a NITROS-accelerated AprilTag node and a non-NITROS USB camera ROS2 node is provided in this package.

Follow these steps to start the graph:

1. Complete the [Quickstart section](../README.md#quickstart) in the main README.
2. Calibrate the camera following instructions in the [camera calibration tutorial](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common/blob/main/docs/camera-calibration.md). For step 11, use `isaac_ros_apriltag` in place of `<isaac_ros_package>`
3. Clone the ROS2 usb_cam package
    ```bash
    cd ~/workspaces/isaac_ros-dev/src && git clone -b ros2 https://github.com/ros-drivers/usb_cam
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
6. Finally, launch the provided launch file:  
    ```bash
    ros2 launch isaac_ros_apriltag isaac_ros_apriltag_usb_cam.launch.py
    ```
7. Launch a second terminal inside the container
    ```bash
    cd ~/workspaces/isaac_ros-dev/src/isaac_ros_common && \
        ./scripts/run_dev.sh
    ```
8. Run the rviz2 using the preset config file
    ```bash
    rviz2 -d /workspaces/isaac_ros-dev/src/isaac_ros_apriltag/isaac_ros_apriltag/rviz/usb_cam.rviz
    ```

> **Note:** Your camera vendor may offer a specific ROS2-compatible camera driver package and you can use that in place of the usb_cam package.

> **Important:** If you are using a different camera driver package ensure that the camera stream publishes `Image` and `CameraInfo` pairs to the topics `/image` and `/camera_info`, respectively.

<div align="center"><img src="../resources/rviz_usb_cam.png"/></div>
