#!/usr/bin/env bash

CAMERA_INDEX=$1
CALIBRATION_DISTANCE=$2
FRAME_RATE=$3
RECORDING_PATH=$4

screen -XS camera exec rosnode kill /multi_camera
screen -XS camera quit 1>/dev/null
screen -dmS camera
screen -XS camera exec roslaunch avt_vimba_camera multi_camera_node_no_trigger.launch compression_type:=none acquisition_rate:=${FRAME_RATE}

echo "Waiting for cameras to start..."

sleep 25

echo "All cameras ready."

screen -XS record quit 1>/dev/null
screen -dmS record
screen -XS record exec rosbag record -O ${RECORDING_PATH}/calibration_camera_${TODAY}_${CALIBRATION_DISTANCE} /multi_camera/image_raw_${CAMERA_INDEX}


