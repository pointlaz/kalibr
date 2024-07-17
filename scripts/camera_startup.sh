#!/usr/bin/env bash

FRAME_RATE=$1


screen -XS camera exec rosnode kill /multi_camera 2>/dev/null
screen -XS camera quit 1>/dev/null
screen -dmS camera
screen -XS camera exec roslaunch avt_vimba_camera multi_camera_node_no_trigger.launch compression_type:=none acquisition_rate:=${FRAME_RATE}

echo "Waiting for cameras to start..."

sleep 30

echo "All cameras are ready."
