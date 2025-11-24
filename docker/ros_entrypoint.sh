#!/bin/bash
 
set -e

# Ros build
source /opt/ros/noetic/setup.bash

if [ -f /home/catkin_ws/devel/setup.bash ]; then
  source /home/catkin_ws/devel/setup.bash
fi

echo "================Docker Env Ready================"

cd /home/catkin_ws

exec "$@"

