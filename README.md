# Description
This is a ROS2 node for controlling DepthAI camera.

# Building and running with docker

## Using GitHub Container Repository Image
The following command will pull the image from GHCR and run the container. Modify the drone device id if different.
```
docker run --rm -it --privileged --volume=/dev:/dev --network=host -v depthai_configs:/depthai_configs --env DRONE_DEVICE_ID=${DRONE_DEVICE_ID} ghcr.io/tiiuae/tii-depthai-ctrl:main
```
## Environmental variables that can be set in the docker run command

* `USE_RAW_CAMERA`: If value is set to 1, the node will also publish raw camera images. Default value is 0/unset. The raw camera images are published as `sensor_msgs/Image` messages to the topic `/$DRONE_DEVICE_ID/camera/color/image_raw`

* `USE_USB_THREE`: By default, USB2 is used for the cameras, because the USB3 has interference with the GPS frequencies. If this variable is set to 1, the node will use USB3, and the resolution and FPS can be raised to higher values.

* `USE_NEURAL_NETWORK`: If set true, the depthai camera will use the available neural network in the container (tiny-yolo). It is done on the Movidius core inside the depthai camera, so there are no loads on the host computer. The detected objects are published as `vision_msgs/Detection2DArray` messages to the topic `/$DRONE_DEVICE_ID/camera/detections`

* `USE_MONO_CAMS`: If set true, the depthai camera will publish the left and right monocular images as `sensor_msgs/Image` messages to the topics `/$DRONE_DEVICE_ID/camera/left/image_raw` and `/$DRONE_DEVICE_ID/camera/right/image_raw`

* `USE_AUTO_FOCUS`: If set true, the depthai camera will use the autofocus feature of the camera. If set false, the camera will use the fixed focus mode. Default value is false.

# Building and running the depthai_ctrl in local ROS2 environment

## Setup workspace and build
```
source /opt/ros/humble/setup.bash
mkdir -p ~/depthai_ws/src
cd ~/depthai_ws/src
git clone https://github.com/tiiuae/depthai_ctrl.git --branch main
cd ~/depthai_ws
rosdep install -y -r -q --from-paths src --ignore-src --rosdistro humble
colcon build --symlink-install
source install/setup.bash
```

## Run depthai_ctrl node
Make sure to update the parameter file before you run the node. The parameter file is located in the depthai_ctrl/params/parameters.yaml. Then run with the following command.

The launch command arguments will override the parameter file values. So, you can modify the exact settings before.
```
ros2 launch depthai_ctrl depthai_launch.py use_raw_color_cam:=true use_neural_network:=true use_passthrough_preview:=true use_video_from_color_cam:=true
```

### List of launch arguments:
* params_file: Full path to the ROS2 parameters file to use. Default value points to the parameters.yaml file in the depthai_ctrl/params folder (only if built with `--symlink-install`, otherwise the path is in the install directory of the workspace)
* use_mono_cams: The mono camera of the camera. Default value is false.
* use_raw_color_cam: The raw color camera of the camera. Default value is false.
* use_video_from_color_cam: The video from color camera of the camera. Default value is false.
* use_auto_focus: The auto focus of the camera. Default value is false.
* use_usb_three: The usb three of the camera. Default value is false.
* use_neural_network: The neural network of the camera. Default value is false.
* use_passthrough_preview: The passthrough preview of the camera. Default value is false.



## list node parameters
Execute command in another shell session or a session where ROS2 environment is available. <br>
```
$ source /opt/ros/galactic/setup.bash
$ ros2 param list
```

## change parameters during runtime, such as encoder bitrate and fps
The following command will change the bitrate to 400000 and fps to 10.0. Default values for these parameters are 3000000 and 25.0 respectively.

```
ros2 service call /${DRONE_DEVICE_ID}/camera/change_parameters rcl_interfaces/srv/SetParameters 'parameters: [{name: "video_fps", value: {type: 3, double_value: 10.0}}, {name: "bitrate", value: {type: 2, integer_value: 400000}}]'
```

# Running gstreamer
By default, the depthai_launch.py only launches the camera node. In order to run the gstreamer, you can use the following command.
```
ros2 run depthai_ctrl gstreamer_node --ros-args --remap __ns:=/$DRONE_DEVICE_ID -p address:=$RTSP_SERVER_ADDRESS/$DRONE_DEVICE_ID
```

### **Disclaimer: The H265 stream is not supported by Firefox or Chrome. Only works with IE or Edge browser due to patent problems.** 


## show streamed video with GStreamer
```
$ gst-launch-1.0 udpsrc port=5600 \
! application/x-rtp,media=video,clock-rate=90000,encoding-name=H264,payload=96 \
! rtph264depay ! queue \
! avdec_h264 ! queue \
! videoconvert \
! autovideosink
```

## store streamed video with GStreamer
```
$ gst-launch-1.0 -e udpsrc port=5600 \
! application/x-rtp,media=video,clock-rate=90000,encoding-name=H264,payload=96 \
! rtph264depay ! queue \
! h264parse ! queue \
! mp4mux \
! filesink location=video_output.mp4
```

# TODO list
* Add monitoring information of the OAK-D to the ROS2 node.
* Add manual focus settings to ROS2 node.
* Add URDF based static transform publisher for transforms between camera frames.
* Test and fix timestamping of the color and stereo cameras.(Info below)

# Known issues
## SPDLOG dependency
The version of depthai-core C++ API has been modified to use libspdlog v1.5.0. DepthAI library uses a newer version of libspdlog (v1.8.1) which is not compatible with the one that ROS2 uses (v1.5.0).<br>
This is the reason why compilation is successful but there is a segmentation fault when ROS2 is initialized and tries to use libspdlog which is linked against 1.8.1 version of the library. <br>
The solution is to create a branch of depthai-core which is compatible with v1.5.0 of spdlog.

## Timestamp of DepthAI camera
The ImgFrame of DepthAI callbacks(camera and encoded video output) includes two methods to get the timestamp of the capture time. `getTimestamp()` and `getTimestampDevice()`. 

The `getTimestamp()` is uses a host synced monotonic time source to provide capture timestamp. However, when the host and OAKD boots up at the same time, this monotonic clock demonstrates a behavior such as the timestamps jump back and forth around ~5-10 seconds even though the sequence number correctly increasing. This might be caused by the time sync happens after the pipeline starts. 

On the other hand, `getTimestampDevice()` gives the OAKD device timestamp directly, which does not show the same behavior. At the moment, the video encoder uses this method since it already uses the first arrived message's timestamp to calculate relative time point of the stream. Even though it is not a problem for the video stream, this method cannot be used for color and stereo camera outputs. The clock difference between host and camera will create an issue when these images are used for real-time operations such as SLAM or object tracking. This needs to be investigated further more. 

One possible fix for this is to delay start of the node by some 20-30 seconds. Since the time sync will already be done by that time, the messages can have correct monotonic timestamp. Not tested yet.

## Open-source libraries used
[luxonis/depthai-ros](https://github.com/luxonis/depthai-ros)
[luxonis/depthai-core](https://github.com/luxonis/depthai-core)