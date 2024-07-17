#!/usr/bin/env bash

CAMERA_INDEX=$1
CALIBRATION_DISTANCE=$2
RECORDING_PATH=$3

echo "Start recording in 10 seconds..."

sleep 10

screen -dmS record
screen -XS record exec rosbag record -O ${RECORDING_PATH}/${TODAY}${CALIBRATION_DISTANCE}/cam${CAMERA_INDEX} /multi_camera/image_raw_${CAMERA_INDEX}

echo "Recording started."
