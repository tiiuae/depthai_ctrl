#!/bin/bash -e

_term() {
    # FILL UP PROCESS SEARCH PATTERN HERE TO FIND PROPER PROCESS FOR SIGINT:
    if [ "${mode}" = "gstreamer" ]; then
        pattern="depthai_ctrl/gstreamer_node"
    else
        pattern="depthai_ctrl/camera_node"
    fi

    pid_value="$(ps -ax | grep $pattern | grep -v grep | awk '{ print $1 }')"
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

mode="${1}"
if [ "${mode}" = "gstreamer" ]; then
    ros-with-env ros2 run depthai_ctrl gstreamer_node \
        --ros-args \
        --remap __ns:=/$DRONE_DEVICE_ID \
        -p address:=$RTSP_SERVER_ADDRESS/$DRONE_DEVICE_ID &
else
    ros-with-env ros2 run depthai_ctrl camera_node \
        --ros-args \
        --remap __ns:=/$DRONE_DEVICE_ID \
        -p address:=$RTSP_SERVER_ADDRESS/$DRONE_DEVICE_ID &
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
