#!/bin/bash

# Source ROS distro environment and local catkin workspace
source "/opt/ros/$ROS_DISTRO/setup.bash" && source "$WS/install/setup.bash"

exec "$@"
