#!/bin/bash

source /enclave/fog_env

prev_ifs=$IFS
IFS='@'
read -a strarr <<< "$RTSP_SERVER_ADDRESS"
IFS=$prev_ifs

./publish-diagnostics-loop.sh $DRONE_DEVICE_ID/video-streamer "Video Streamer" 0 operational backend-url ${strarr[1]} &
exec ros-with-env ros2 run depthai_ctrl gstreamer_node --ros-args --remap __ns:=/$DRONE_DEVICE_ID -p address:=$RTSP_SERVER_ADDRESS/$DRONE_DEVICE_ID

pkill -P $$

sleep 5s

./publish-diagnostics-once.sh $DRONE_DEVICE_ID/video-streamer "Video Streamer" 2 crashed backend-url ${strarr[1]}