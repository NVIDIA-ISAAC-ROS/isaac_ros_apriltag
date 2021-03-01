# NVAprilTags ROS2 Node

This ROS2 node uses the NVIDIA GPU-accelerated AprilTags library to detect AprilTags in images and publish their poses, ids, and additional metadata. This has been tested on ROS2 (Foxy) and should build and run on x86_64 and aarch64 (Jetson). It is modeled after and comparable to the ROS2 node for CPU AprilTags detection here: https://github.com/christianrauch/apriltag_ros.git

For more information on the Isaac GEM this node is based off of, see the Isaac SDK 2020.2 documentation here: https://docs.nvidia.com/isaac/isaac/packages/fiducials/doc/apriltags.html

For more information on AprilTags themselves, the paper and the reference CPU implementation: https://april.eecs.umich.edu/software/apriltag.html

## Topics

### Subscriptions:
The node subscribes via a `image_transport::CameraSubscriber` to `/apriltag/image`. The set of topic names depends on the type of image transport (parameter `image_transport`) selected (`raw` or `compressed`):
- `/apriltag/image` (`raw`, type: `sensor_msgs/Image`)
- `/apriltag/image/compressed` (`compressed`, type: `sensor_msgs/CompressedImage`)
- `/apriltag/camera_info` (type: `sensor_msgs/CameraInfo`)

### Publisher:
- `/tf` (type: `tf2_msgs/TFMessage`)
- `/apriltag/detections` (type: `apriltag_msgs/AprilTagDetectionArray`)

The camera intrinsics `K` in `CameraInfo` are used to compute the marker tag pose `T` from the homography `H`. The image and the camera intrinsics need to have the same timestamp.

The tag poses are published on the standard TF topic `/tf` with the header set to the image header and `child_frame_id` set to either `tag<family>:<id>` (e.g. "tag36h11:0") or the frame name selected via configuration file. Additional information about detected tags is published as `AprilTagDetectionArray` message, which contains the original homography  matrix, the `hamming` distance and the `decision_margin` of the detection.

## Configuration

The node is configured via a yaml configurations file. For the complete ROS yaml parameter file syntax, see: https://github.com/ros2/rcl/tree/master/rcl_yaml_param_parser.

The file has the format:
```YAML
apriltag:                           # namespace
    apriltag:                       # node name
        ros__parameters:
            # required
            image_transport: raw    # image format: "raw" or "compressed" (default: raw)
            family: <tag family>    # tag family name: 36h11 [only one family supported]
            size: <tag edge size>   # tag edge size in meter (default: 2.0)
            
            # (optional) list of tags
            max_tags: <maximum tag count>    # maximum number of tags to detect in a single frame (default: 20)
```

The parameters `family` and `size` are required. `family` (string) defines the tag family for the detector and can only be `36h11` at this time. `size` (float) is the tag edge size in meters, assuming square markers.

### Start
As any ROS2 package, check the repository out under `src/` in a ROS2 workspace and invoke `colcon build` appropriately to compile and prepare for use.

The launch file can be used to start a component manager and load the composable node with configuration:
```bash
ros2 launch nvapriltags_ros2 tag_36h11.launch.py
```
You need to run a camera node (e.g. from `v4l2_camera` package) to feed frames into the `nvapriltags_ros2` node for detection. For example, you can run the `v4l2_camera_node` and remap its output topics as follows:
```bash
ros2 run v4l2_camera v4l2_camera_node /camera_info:=/camera/camera_info /image_raw:=/camera/image
```
Tools such as `rqt` or other components can then consume the AprilTag detection messages from the `nvapriltags_ros2` node. 

You will need to calibrate the intrinsics of your camera if you want the node to determine 3D poses for tags instead of just detection and corners as 2D pixel coordinates. See here: https://navigation.ros.org/tutorials/docs/camera_calibration.html
