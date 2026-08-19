#!/bin/bash
set -e

source /opt/ros/humble/setup.bash
source /root/turtlebot3_ws/install/setup.bash

if [[ -f /usr/share/gazebo/setup.sh ]]; then
  source /usr/share/gazebo/setup.sh
fi

exec "$@"
