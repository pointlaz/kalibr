#!/usr/bin/env bash

set -e

echo "Stopping multi_camera node..."

screen -XS camera exec rosnode kill /multi_camera 2>/dev/null

sleep 2

screen -XS camera quit 1>/dev/null

echo "multi_camera node stopped."
