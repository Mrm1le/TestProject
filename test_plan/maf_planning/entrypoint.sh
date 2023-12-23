#!/bin/bash
source /opt/ros/kinetic/setup.bash || exit 1
source "${CONTAINER_AUTO_WS}/install/setup.bash" || exit 1
exec "$@"
