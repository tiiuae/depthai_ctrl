#!/bin/bash

if [ "$1" = "gstreamer" ]; then
    exec ros-with-env ros2 run depthai_ctrl gstreamer_node --ros-args --remap __ns:=/$DRONE_DEVICE_ID -p address:=$RTSP_SERVER_ADDRESS/$DRONE_DEVICE_ID
else
    exec ros-with-env ros2 run depthai_ctrl camera_node --ros-args --remap __ns:=/$DRONE_DEVICE_ID -p address:=$RTSP_SERVER_ADDRESS/$DRONE_DEVICE_ID
fi
