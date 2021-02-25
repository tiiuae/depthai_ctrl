# Description
This is a ROS2 node for controlling DepthAI camera (OAK-D model).

# Build instructions
```
$ mkdir build && cd $_
$ cmake .. -DBUILD_SHARED_LIBS=[OFF|ON] -DCMAKE_BUILD_TYPE=[Release|Debug]
$ cmake --build . --parallel
```
Use static libraries.

# Packaging instructions
Generate deb files:
```
$ cd fog_sw/packaging
$ ./package.sh
```
Debian packages are generated into fog_sw/packaging folder

# How to
## start application for development
```
$ cd build
$ source /opt/ros/foxy/setup.bash
$ ./depthai_ctrl
```

## start streaming video for development
Execute command in another shell session. <br>
```
$ source /opt/ros/foxy/setup.bash
$ ros2 topic pub -t 1 /depthai_cam_cmd std_msgs/msg/String "data: '{ \"Command\": \"start\", \"Encoding\": \"H264\" }
```

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
* Refactor code: split code into several files.
* Add method to restart streaming.
* Add options to the stream: H265 encoding, resolution.
* Add monitoring information of the OAK-D to the ROS2 node.
* Add manual focus settings to ROS2 node.

# Known issues
## SPDLOG dependency
The version of depthai-core C++ API has been modified to use libspdlog v1.5.0. DepthAI library uses a newer version of libspdlog (v1.8.1) which is not compatible with the one that ROS2 uses (v1.5.0).<br>
This is the reason why compilation is successful but there is a segmentation fault when ROS2 is initialized and tries to use libspdlog which is linked against 1.8.1 version of the library. <br>
The solution is to create a branch of depthai-core which is compatible with v1.5.0 of spdlog.