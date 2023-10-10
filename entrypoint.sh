#!/bin/bash -e

_term() {
    # FILL UP PROCESS SEARCH PATTERN HERE TO FIND PROPER PROCESS FOR SIGINT:
    if [ "${mode}" = "gstreamer" ]; then
        pattern="depthai_ctrl/gstreamer_node"
    else
        pattern="depthai_ctrl/camera_node"
    fi

    pid_value="$(ps -e | grep $pattern | grep -v grep | awk '{ print $1 }')"
    if [ "$pid_value" != "" ]; then
        pid=$pid_value
        echo "Send SIGINT to pid $pid"
    else
        pid=1
        echo "Pattern not found, send SIGINT to pid $pid"
    fi
    kill -s SIGINT $pid
}
# Use SIGTERM or TERM, does not seem to make any difference.
trap _term TERM

if [[ ${DEPTHAI_PARAM_FILE+x} != "" ]]; then
	ROS_FLAGS="params_file:=${DEPTHAI_PARAM_FILE} ${ROS_FLAGS}"
fi

if [ "${USE_RAW_CAMERA}" = "1" ]; then
    ROS_FLAGS="${ROS_FLAGS} use_raw_color_cam:=true"
else
    ROS_FLAGS="${ROS_FLAGS} use_raw_color_cam:=false"
fi

if [ "${USE_NEURAL_NETWORK}" = "1" ]; then
    ROS_FLAGS="${ROS_FLAGS} use_neural_network:=true"
fi

if [ "${USE_USB_THREE}" = "1" ]; then
    ROS_FLAGS="${ROS_FLAGS} use_usb_three:=true"
else
    if [ "${USE_RAW_CAMERA}" = "1" ]; then
        ROS_FLAGS="${ROS_FLAGS} use_video_from_color_cam:=true"
        echo "WARNING: Using raw camera, but not using USB tree. Enabling video from color camera."
    fi
fi
if [ "${USE_MONO_CAMS}" = "1" ]; then
    ROS_FLAGS="${ROS_FLAGS} use_mono_cams:=true"
fi
if [ "${USE_AUTO_FOCUS}" = "1" ]; then
    ROS_FLAGS="${ROS_FLAGS} use_auto_focus:=true"
fi
# This is required for multiarch images. Without this udev start, depthai library fails to find XLink connection to device.
# The reason is that depthai camera switches to onboard processor when initialized, and causing USB device name changed.
# Old version with ubuntu base worked OK for hotplug, but this one does not.
# This could cause problem on the host machine, causing some USB devices to disconnect and require a unplug-replug.
/etc/init.d/udev start
mode="${1}"
if [ "${mode}" = "gstreamer" ]; then
    ros-with-env ros2 run depthai_ctrl gstreamer_node \
        --ros-args \
        --remap __ns:=/$DRONE_DEVICE_ID \
        -p address:=$RTSP_SERVER_ADDRESS/$DRONE_DEVICE_ID &
else
    ros-with-env ros2 launch depthai_ctrl depthai_launch.py ${ROS_FLAGS}&
fi
child=$!

echo "Waiting for pid $child"
# * Calling "wait" will then wait for the job with the specified by $child to finish, or for any signals to be fired.
#   Due to "or for any signals to be fired", "wait" will also handle SIGTERM and it will shutdown before
#   the node ends gracefully.
#   The solution is to add a second "wait" call and remove the trap between the two calls.
# * Do not use -e flag in the first wait call because wait will exit with error after catching SIGTERM.
set +e
wait $child
set -e
trap - TERM
wait $child
RESULT=$?

if [ $RESULT -ne 0 ]; then
    echo "ERROR: DepthAI camera node failed with code $RESULT" >&2
    exit $RESULT
else
    echo "INFO: DepthAI camera node finished successfully, but returning 125 code for docker to restart properly." >&2
    exit 125
fi
