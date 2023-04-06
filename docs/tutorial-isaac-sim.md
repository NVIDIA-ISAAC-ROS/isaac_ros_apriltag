# Tutorial for AprilTag Detection with Isaac Sim

<div align="center"><img src="../resources/Rviz_apriltag_output.png" width="800px"/></div>

## Overview

This tutorial walks you through a graph to estimate the 6DOF pose of [AprilTags](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_apriltag) using images from Isaac Sim.

Last validated with [Isaac Sim 2022.2.1](https://docs.omniverse.nvidia.com/app_isaacsim/app_isaacsim/release_notes.html#id1)

## Tutorial Walkthrough

1. Complete the [Quickstart section](../README.md#quickstart) in the main README.
2. Launch the Docker container using the `run_dev.sh` script:

    ```bash
    cd ~/workspaces/isaac_ros-dev/src/isaac_ros_common && \
      ./scripts/run_dev.sh
    ```

3. Inside the container, build and source the workspace:  

    ```bash
    cd /workspaces/isaac_ros-dev && \
      colcon build --symlink-install && \
      source install/setup.bash
    ```

4. Launch the pre-composed graph launchfile:

    ```bash
    ros2 launch isaac_ros_apriltag isaac_ros_apriltag_isaac_sim_pipeline.launch.py
    ```

5. Install and launch Isaac Sim following the steps in the [Isaac ROS Isaac Sim Setup Guide](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common/blob/main/docs/isaac-sim-sil-setup.md)
6. Open up the Isaac ROS Common USD scene (using the *Content* tab) located at:

   ```text
   http://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/2022.2.1/Isaac/Samples/ROS2/Scenario/carter_warehouse_apriltags_worker.usd
   ```

   And wait for it to load completely.
7. Press **Play** to start publishing data from Isaac Sim.
   <div align="center"><img src="../resources/Isaac_sim_play.png" width="800px"/></div>

8. In a separate terminal, run RViz to visualize the AprilTag detections:  

    ```bash
    rviz2
    ```

9. Add the tf tree in the **Displays** RViz panel.
   <div align="center"><img src="../resources/Rviz_add_tf.png" width="600px"/></div>
10. Set the **Fixed frame** in the **Global Options** to `chassis_link`.
    <div align="center"><img src="../resources/Rviz_fixed_frame.png" width="300px"/></div>
11. You should see the pose of the tags in RVIZ:
    <div align="center"><img src="../resources/Rviz_apriltag_output.png" width="800px"/></div>
12. If you prefer to observe the AprilTag output in text form, echo the contents of the `/tag_detections` topic with the following command in a separate terminal:

    ```bash
    ros2 topic echo /tag_detections
    ```

    <div align="center"><img src="../resources/Terminal_output.png" width="600px"/></div>
