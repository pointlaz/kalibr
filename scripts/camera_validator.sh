#!/usr/bin/env bash

CAMERA_INDEX=0

echo "${BAGS_BASE_PATH}/${CAMERA_MODEL}-${DISTORTION_MODEL}_mi-tol_${MUTUAL_INFORMATION_TOLERANCE}/camera_${CAMERA_INDEX}-camchain.yaml"

rosrun kalibr kalibr_camera_validator \
--cam "${BAGS_BASE_PATH}/../${CAMERA_MODEL}-${DISTORTION_MODEL}_mi-tol_${MUTUAL_INFORMATION_TOLERANCE}/camera_${CAMERA_INDEX}-camchain.yaml" \
--target /target.yaml \
