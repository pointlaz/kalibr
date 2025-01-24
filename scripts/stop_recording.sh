#!/usr/bin/env bash

source ~/catkin_ws/devel/setup.bash

set -e

echo "Stopping current recording..."

RECORD_NODE=$(rosnode list | grep record)

rosnode kill "${RECORD_NODE}" 2> /dev/null

echo "Recording stopped."
