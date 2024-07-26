#!/usr/bin/env bash

set -e

DEFAULT_RECORDING_BASE_PATH="/hdd/Documents/LAZ_Data/Calibrations/Cameras"
DEFAULT_BAG_DURATION=180

if [[ $# == 0 ]]; then
    RECORDING_BASE_PATH_ARG="${DEFAULT_RECORDING_BASE_PATH}"
    BAG_DURATION_ARG=$DEFAULT_BAG_DURATION
elif [[ $# == 2 ]]; then
    if [[ "$1" == "-p" || "$1" == "--path" ]]; then
        BAG_DURATION_ARG=$DEFAULT_BAG_DURATION
        if [[ -d "$2" ]]; then
            RECORDING_BASE_PATH_ARG="$2"
        else
            echo "The recording path must be an existing directory."
            RECORDING_BASE_PATH_ARG="${DEFAULT_RECORDING_BASE_PATH}"
        fi
    elif [[ "$1" == "-d" || "$1" == "--duration" ]]; then
        RECORDING_BASE_PATH_ARG="${DEFAULT_RECORDING_BASE_PATH}"
        if [[ $2 =~ ^[0-9]+$ ]]; then
            BAG_DURATION_ARG=$2
        else
            echo "The duration must be a number in seconds."
            BAG_DURATION_ARG=$DEFAULT_BAG_DURATION
        fi
    else
        if [[ ("$1" =~ "-p" && "$1" != "-p") || ("$1" =~ "--p" && "$1" != "--path") ]]; then
            echo "Wrong flag for the path."
        elif [[ ("$1" =~ "-d" && "$1" != "-d") || ("$1" =~ "--d" && "$1" != "--duration") ]]; then
            echo "Wrong flag for the duration."
        fi
        echo "Usage: start_recording.sh [(-p | --path) PATH] [(-d | --duration) DURATION]
-p, --path PATH          The path to the recording directory.
-d, --duration DURATION  The duration of the recording  in seconds."
        exit 1
    fi
elif [[ $# == 4 ]]; then
    if [[ ("$1" == "-p" || "$1" == "--path") && ("$3" == "-d" || "$3" == "--duration") ]]; then
        if [[ -d "$2" ]]; then
            RECORDING_BASE_PATH_ARG="$2"
        else
            echo "The recording path must be an existing directory."
            RECORDING_BASE_PATH_ARG="${DEFAULT_RECORDING_BASE_PATH}"
        fi
        if [[ $4 =~ ^[0-9]+$ ]]; then
            BAG_DURATION_ARG=$4
        else
            echo "The duration must be a number in seconds."
            BAG_DURATION_ARG=$DEFAULT_BAG_DURATION
        fi
    elif [[ ("$1" == "-d" || "$1" == "--duration") && ("$3" == "-p" || "$3" == "--path") ]]; then
        if [[ $2 =~ ^[0-9]+$ ]]; then
            BAG_DURATION_ARG=$2
        else
            echo "The duration must be a number in seconds."
            BAG_DURATION_ARG=$DEFAULT_BAG_DURATION
        fi
        if [[ -d "$4" ]]; then
            RECORDING_BASE_PATH_ARG="$4"
        else
            echo "The recording path must be an existing directory."
            RECORDING_BASE_PATH_ARG="${DEFAULT_RECORDING_BASE_PATH}"
        fi
    else
        if [[ (("$1" != "-p" && "$1" != "--path") && ("$3" == "-d" || "$3" == "--duration")) ||
            (("$1" == "-d" || "$1" == "--duration") && ("$3" != "-p" && "$3" != "--path")) ]]; then
            echo "Wrong flag for the path."
        elif [[ (("$1" == "-p" || "$1" == "--path") && ("$3" != "-d" && "$3" != "--duration")) ||
            (("$1" != "-d" && "$1" != "--duration") && ("$3" == "-p" || "$3" == "--path")) ]]; then
            echo "Wrong flag for the duration."
        else
            echo "Wrong flags for the path and the duration."
        fi
        echo "Usage: start_recording.sh [(-p | --path) PATH] [(-d | --duration) DURATION]
-p, --path PATH          The path to the recording directory.
-d, --duration DURATION  The duration of the recording  in seconds."
        exit 1
    fi
else
    echo "Wrong number of arguments. $# arguments were provided instead of 4.
Usage: start_recording.sh [(-p | --path) PATH] [(-d | --duration) DURATION]
-p, --path PATH          The path to the recording directory.
-d, --duration DURATION  The duration of the recording in seconds."
    exit 1
fi

CALIBRATION_DATE=$(date +%Y-%m-%d)
RECORDING_PATH="${RECORDING_BASE_PATH_ARG}/${CALIBRATION_DATE}/Bags"
BAG_DURATION="${BAG_DURATION_ARG}"

mkdir -p "${RECORDING_PATH}"

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

    BAG_NAME="camera_${CAMERA_INDEX}"
    BAG_PATH="${RECORDING_PATH}/${BAG_NAME}"

    echo "Camera $((CAMERA_INDEX + 1)) recording will start in 10 seconds..."

    sleep 10

    screen -XS record exec rosbag record --buffsize 1024 --duration "${BAG_DURATION}" -O "${BAG_PATH}" "/multi_camera/image_raw_${CAMERA_INDEX_ARG}"

    echo "Camera $((CAMERA_INDEX + 1)) recording is started."

    sleep "$((BAG_DURATION + 5))"

    echo "Camera $((CAMERA_INDEX + 1)) recording is finished."

    echo "The file '${BAG_NAME}.bag' has been saved in the '${RECORDING_PATH}' directory."

    if [[ "${CAMERA_INDEX}" -lt "${CAMERA_INDEXES[-1]}" ]]; then
        echo "Prepare the camera $((CAMERA_INDEX + 2)) now."
        sleep 10
    fi

done

echo "All camera recordings are completed."

screen -XS record quit 1>/dev/null
