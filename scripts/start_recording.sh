#!/usr/bin/env bash

CAMERA_INDEX_ARG=$1
CALIBRATION_DISTANCE_ARG=$2
RECORDING_PATH_ARG=$3

CALIBRATION_DATE=$(date +%Y-%m-%d)

echo "Start recording in 10 seconds..."

sleep 10

screen -dmS record
screen -XS record exec rosbag record -O "${RECORDING_PATH_ARG}/${CALIBRATION_DATE}/${CALIBRATION_DISTANCE_ARG}/${CALIBRATION_DATE}_${CALIBRATION_DISTANCE_ARG}_camera_${CAMERA_INDEX_ARG}" "/multi_camera/image_raw_${CAMERA_INDEX_ARG}"

echo "Recording started."
