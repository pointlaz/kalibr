#!/bin/env bash

if [[ -z "${BAGS_BASE_PATH}" ]]; then
	echo "BAGS_BASE_PATH variable must be set, i.e. A valid path to a directory containing the bags used for calibration."
	exit 1
fi

if [ ! -d "${BAGS_BASE_PATH}" ]; then
	echo "Directory '${BAGS_BASE_PATH}' does not exists. Aborting calibration..."
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

CAMERA_INDEXES=(
	0
	1
	2
	3
	4
	5
	6
)

for CAMERA_INDEX in "${CAMERA_INDEXES[@]}"; do

	BAG_NAME="camera_${CAMERA_INDEX}"
	BAG_PATH="${BAGS_BASE_PATH}/${BAG_NAME}"

	if [[ ! -f "${BAG_PATH}.bag" ]]; then
		echo "File '${BAG_PATH}.bag' does not exists. Aborting calibration..."
		exit 1
	fi

	echo "Calibrating camera '${CAMERA_INDEX}' using the bag '${BAG_PATH}.bag' ..."

	rosrun kalibr kalibr_calibrate_cameras \
	--models "${CAMERA_MODEL}-${DISTORTION_MODEL}" \
	--bag "${BAG_PATH}.bag" \
	--topics "/multi_camera/image_raw_${CAMERA_INDEX}" \
	--mi-tol "${MUTUAL_INFORMATION_TOLERANCE}" \
	--target /target.yaml \
	--dont-show-report

	CAMCHAIN_FILE="${BAG_PATH}-camchain.yaml"
	REPORT_CAM_FILE="${BAG_PATH}-report-cam.pdf"
	RESULTS_CAM_FILE="${BAG_PATH}-results-cam.txt"

	if [ -f "$REPORT_CAM_FILE" ]; then
		RESULTS_PATH="${BAGS_BASE_PATH}/../${CAMERA_MODEL}-${DISTORTION_MODEL}_mi-tol_${MUTUAL_INFORMATION_TOLERANCE}"
		mkdir -p "${RESULTS_PATH}"
		mv "${CAMCHAIN_FILE}"    "${RESULTS_PATH}/"
		mv "${REPORT_CAM_FILE}"  "${RESULTS_PATH}/"
	else
		rm "${CAMCHAIN_FILE}" 2>/dev/null
	fi

	rm "${RESULTS_CAM_FILE}" 2>/dev/null
done

echo "All camera calibrations are done."
