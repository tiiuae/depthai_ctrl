#!/bin/bash

source /opt/ros/galactic/setup.bash
ros2 run depthai_ctrl camera_node --ros-args --remap __ns:=/$DRONE_DEVICE_ID -p address:=$RTSP_SERVER_ADDRESS/$DRONE_DEVICE_ID
