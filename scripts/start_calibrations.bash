#!/bin/env bash

set -e

if [[ -z "${SCANNER_SN}" ]]; then
	echo "'SCANNER_SN' variable must be set, i.e. A valid path to a directory containing the bags used for calibration."
	exit 1
fi

RAW_DIR_PATH="${SCANNER_SN}/cameras/raw"

if [ ! -d "${RAW_DIR_PATH}" ]; then
	echo "Directory '${RAW_DIR_PATH}' does not exists. Create it before proceeding. Aborting calibration..."
	exit 1
fi

DEFAULT_CAMERA_MODEL=pinhole
if [[ -z "${CAMERA_MODEL}" ]]; then
	echo "'CAMERA_MODEL' variable unset. Using the default value '${DEFAULT_CAMERA_MODEL}' instead."
	CAMERA_MODEL="${DEFAULT_CAMERA_MODEL}"
fi

DEFAULT_DISTORTION_MODEL=radtan
if [[ -z "${DISTORTION_MODEL}" ]]; then
	echo "'DISTORTION_MODEL' variable unset. Using the default value '${DEFAULT_DISTORTION_MODEL}' instead."
	DISTORTION_MODEL="$DEFAULT_DISTORTION_MODEL"
fi

DEFAUL_MUTUAL_INFORMATION_TOLERANCE=0.1
if [[ -z "${MUTUAL_INFORMATION_TOLERANCE}" ]]; then
	echo "'MUTUAL_INFORMATION_TOLERANCE' variable unset. Using the default value '${DEFAUL_MUTUAL_INFORMATION_TOLERANCE}' instead."
	MUTUAL_INFORMATION_TOLERANCE="${DEFAUL_MUTUAL_INFORMATION_TOLERANCE}"
fi

BAG_PATHS=( $( ls "${RAW_DIR_PATH}"/*.bag 2>/dev/null ) )

if (( "${#BAG_PATHS[@]}" == 0 )); then
	echo "Directory '${RAW_DIR_PATH}' is empty. Aborting calibration..."
	exit 1
fi

source "${WORKSPACE}"/devel/setup.bash

for BAG_PATH in "${BAG_PATHS[@]}"; do

	CAMERA_INDEX="${BAG_PATH#*camera-}"
	CAMERA_INDEX="${CAMERA_INDEX%-calibration*}"

	echo ""
	read -n 1 -sp "Calibrating camera '${CAMERA_INDEX}' using the bag: '${BAG_PATH##*/}'.
Press 'ENTER' or 'SPACE' to start calibration or skip by pressing ANY other key." answer
    echo ""

	if [[ "$answer" == "" ]]; then
		rosrun kalibr kalibr_calibrate_cameras \
		--models "${CAMERA_MODEL}-${DISTORTION_MODEL}" \
		--bag "${BAG_PATH}" \
		--topics "/multi_camera/image_raw_${CAMERA_INDEX}" \
		--mi-tol "${MUTUAL_INFORMATION_TOLERANCE}" \
		--target /target.yaml \
		--dont-show-report

		CAMCHAIN_FILE="${BAG_PATH%.bag}-camchain.yaml"
		REPORT_CAM_FILE="${BAG_PATH%.bag}-report-cam.pdf"
		RESULTS_CAM_FILE="${BAG_PATH%.bag}-results-cam.txt"

		if [ -f "$REPORT_CAM_FILE" ]; then
			RESULTS_PATH="${RAW_DIR_PATH}/../results/${CAMERA_MODEL}_${DISTORTION_MODEL}_mi-tol_${MUTUAL_INFORMATION_TOLERANCE}"
			mkdir -p "${RESULTS_PATH}"
			mv "${CAMCHAIN_FILE}"    "${RESULTS_PATH}/"
			mv "${REPORT_CAM_FILE}"  "${RESULTS_PATH}/"
		else
			rm "${CAMCHAIN_FILE}" 2> /dev/null
		fi

		rm "${RESULTS_CAM_FILE}" 2> /dev/null
	fi
done

echo "All camera calibrations are done."
