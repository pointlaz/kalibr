#!/bin/env bash

CAMERA_INDEXES=({0..6})

for CAMERA_INDEX in "${CAMERA_INDEXES[@]}"; do
	BAG_NAME="cam${CAMERA_INDEX}"
	BAG_FULL_PATH="${BAG_BASE_PATH}/${BAG_NAME}.bag"

	echo "Calibrating camera ${CAMERA_INDEX} using rosbag ${BAG_FULL_PATH}"

	rosrun kalibr kalibr_calibrate_cameras \
	--models "${CAMERA_MODEL}-${DISTORTION_MODEL}" \
	--bag "${BAG_FULL_PATH}" \
	--topics "${CAMERA_BASE_TOPIC}_${CAMERA_INDEX}" \
	--mi-tol "${MUTUAL_INFORMATION_TOLERANCE}" \
	--target /target.yaml \
	--dont-show-report

	CAMCHAIN_FILE="${BAG_BASE_PATH}/${BAG_NAME}-camchain.yaml"
	REPORT_CAM_FILE="${BAG_BASE_PATH}/${BAG_NAME}-report-cam.pdf"
	RESULTS_CAM_FILE="${BAG_BASE_PATH}/${BAG_NAME}-results-cam.txt"

	RESULTS_DIR="${BAG_BASE_PATH}/${CAMERA_MODEL}-${DISTORTION_MODEL}_mi-tol_${MUTUAL_INFORMATION_TOLERANCE}"
	mkdir -p "${RESULTS_DIR}"

	if [ -f "$REPORT_CAM_FILE" ]; then
		mv "${CAMCHAIN_FILE}"    "${RESULTS_DIR}/cam${CAMERA_INDEX}-camchain.yaml"
		mv "${REPORT_CAM_FILE}"  "${RESULTS_DIR}/cam${CAMERA_INDEX}-report-cam.pdf"
		rm "${RESULTS_CAM_FILE}"
	else
		rm "${CAMCHAIN_FILE}"
		rm "${RESULTS_CAM_FILE}"
	fi
done

echo "All camera calibrations are done."
