#!/usr/bin/env bash

set -e

echo "Stopping current recording..."

RECORD_NODE=$(rosnode list | grep record)

screen -XS record exec rosnode kill "${RECORD_NODE}" 2>/dev/null
sleep 2
screen -XS record quit 1>/dev/null

echo "Recording stopped."
