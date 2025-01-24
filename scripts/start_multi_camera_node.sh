#!/usr/bin/env bash

source ~/catkin_ws/devel/setup.bash

set -e

if rosnode list | grep -q multi_camera; then
    echo "The multi_camera node is running. Stopping before restarting."
    bash ./stop_multi_camera_node.sh
fi

FRAME_RATE_ARG=$1
DEFAULT_FRAME_RATE=10
FRAME_RATE=${FRAME_RATE_ARG:-$DEFAULT_FRAME_RATE}

rostopic pub --once /scanner_state std_msgs/Int8 "data: 1"

sleep 5

echo "Camera startup at ${FRAME_RATE} fps"

roslaunch avt_vimba_camera multi_camera_node_no_trigger.launch compression_type:=none acquisition_rate:="${FRAME_RATE}"

echo "Waiting for cameras to start..."

sleep 30

echo "All cameras are ready."
