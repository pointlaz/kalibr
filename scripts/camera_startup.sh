#!/usr/bin/env bash

set -e

if rosnode list | grep -q multi_camera; then
    echo "The multi_camera node is running."
    bash /scripts/stop_multi_camera_node.sh
fi

FRAME_RATE_ARG=$1
DEFAULT_FRAME_RATE=10
FRAME_RATE=${FRAME_RATE_ARG:-$DEFAULT_FRAME_RATE}

echo "Camera startup at ${FRAME_RATE} fps"

screen -dmS camera
screen -XS camera exec roslaunch avt_vimba_camera multi_camera_node_no_trigger.launch compression_type:=none acquisition_rate:="${FRAME_RATE}"

echo "Waiting for cameras to start..."

sleep 30

echo "All cameras are ready."
