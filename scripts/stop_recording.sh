#!/usr/bin/env bash

echo "Stop recording..."

NODE=$(rosnode list | grep record)

screen -XS record exec rosnode kill ${NODE}
sleep 5
screen -XS record quit 1>/dev/null

echo "Recording stopped."
