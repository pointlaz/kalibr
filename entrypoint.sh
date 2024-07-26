#!/bin/env bash

set -e

source /opt/ros/noetic/setup.bash
source ${WORKSPACE}/devel/setup.bash

exec "$@"
