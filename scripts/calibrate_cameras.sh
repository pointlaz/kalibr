#!/bin/env bash

CAMERA_BASE_TOPIC=/multi_camera/image_raw
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
	BAG_NAME="${CALIBRATION_DATE}_${CALIBRATION_DISTANCE}_camera_${CAMERA_INDEX}"
	BAG_PATH="${BAGS_BASE_PATH}/${BAG_NAME}"

	echo "Calibrating camera ${CAMERA_INDEX} using rosbag ${BAG_PATH}.bag ..."

	rosrun kalibr kalibr_calibrate_cameras \
	--models "${CAMERA_MODEL}-${DISTORTION_MODEL}" \
	--bag "${BAG_PATH}.bag" \
	--topics "${CAMERA_BASE_TOPIC}_${CAMERA_INDEX}" \
	--mi-tol "${MUTUAL_INFORMATION_TOLERANCE}" \
	--target /target.yaml \
	--dont-show-report

	CAMCHAIN_FILE="${BAG_PATH}-camchain.yaml"
	REPORT_CAM_FILE="${BAG_PATH}-report-cam.pdf"
	RESULTS_CAM_FILE="${BAG_PATH}-results-cam.txt"

	if [ -f "$REPORT_CAM_FILE" ]; then
		RESULTS_DIR="${CALIBRATION_BASE_PATH}/${CALIBRATION_DATE}/${CALIBRATION_DISTANCE}/${CAMERA_MODEL}-${DISTORTION_MODEL}_mi-tol_${MUTUAL_INFORMATION_TOLERANCE}"
		mkdir -p "${RESULTS_DIR}"
		mv "${CAMCHAIN_FILE}"    "${RESULTS_DIR}/"
		mv "${REPORT_CAM_FILE}"  "${RESULTS_DIR}/"
	else
		rm "${CAMCHAIN_FILE}" 2>/dev/null
	fi

	rm "${RESULTS_CAM_FILE}" 2>/dev/null
done