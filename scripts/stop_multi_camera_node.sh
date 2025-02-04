#!/usr/bin/env bash

source ~/catkin_ws/devel/setup.bash

set -e

echo "Stopping multi_camera node..."

rosnode kill /multi_camera 2> /dev/null

sleep 2

echo "multi_camera node stopped."

rostopic pub --once /scanner_state std_msgs/Int8 "data: 0" 1>/dev/null
