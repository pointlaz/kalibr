#!/usr/bin/env bash

set -e

CALIBRATION_DATE=$(date +%Y-%m-%d)

RECORDING_BASE_PATH_ARG=$1
DEFAULT_RECORDING_BASE_PATH="/hdd/Documents/LAZ_Data/Calibrations/Cameras"

if [[ -z "${RECORDING_BASE_PATH_ARG}" ]]; then
	echo "No recording base path has been provided, the default recording base path will be used instead. The default value is: ${DEFAULT_RECORDING_BASE_PATH}"
fi

RECORDING_PATH="${RECORDING_BASE_PATH_ARG:-$DEFAULT_RECORDING_BASE_PATH}/${CALIBRATION_DATE}/bags"

mkdir -p "${RECORDING_PATH}"

BAG_DURATION_ARG=$2
BAG_DURATION_DEFAULT=180

if [[ -z "${RECORDING_BASE_PATH_ARG}" ]]; then
	echo "No bag duration has been provided, the default bag duration will be used instead. Default value is: ${BAG_DURATION_DEFAULT} seconds."
fi

BAG_DURATION="${BAG_DURATION_ARG:-$BAG_DURATION_DEFAULT}"

CAMERA_INDEXES=(
    0
    1
    2
    3
    4
    5
    6
)

screen -dmS record

for CAMERA_INDEX in "${CAMERA_INDEXES[@]}"; do

    BAG_NAME="${CALIBRATION_DATE}_camera_${CAMERA_INDEX}"
    BAG_PATH="${RECORDING_PATH}/${BAG_NAME}"

    echo "Camera $((CAMERA_INDEX + 1)) recording will start in 10 seconds..."

    sleep 10

    screen -XS record exec rosbag record --buffsize 1024 --duration "${BAG_DURATION}" -O "${BAG_PATH}" "/multi_camera/image_raw_${CAMERA_INDEX_ARG}"

    echo "Camera $((CAMERA_INDEX + 1)) recording is started."

    sleep "$((BAG_DURATION + 5))"

    echo "Camera $((CAMERA_INDEX + 1)) recording is finished."

    echo "The ${BAG_NAME} bag has been saved in the ${RECORDING_PATH} directory."

    if [[ "${CAMERA_INDEX}" -lt "${CAMERA_INDEXES[-1]}" ]]; then
        echo "Prepare camera $((CAMERA_INDEX + 2)) now."
    fi

    sleep 10

done

echo "Recording of all cameras is complete."

screen -XS record quit 1>/dev/null
