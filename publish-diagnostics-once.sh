#!/bin/bash

HARDWARE_ID=$1
NAME=$2
LEVEL=$3
MESSAGE=$4
KEY=$5
VALUE=$6

MSG="{ header: { stamp: { sec: $(date +%s), nanosec: 0 }, frame_id: "" }, status: [ { level: [$3], name: $NAME, message: $MESSAGE, hardware_id: $HARDWARE_ID, values: [ { key: $KEY, value: $VALUE } ] } ] }"
ros2 topic pub --once --keep-alive 2 --qos-profile system_default /diagnostics diagnostic_msgs/msg/DiagnosticArray "$MSG"
