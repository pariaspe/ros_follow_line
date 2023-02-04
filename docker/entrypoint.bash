#!/bin/bash
set -e

# setup ros environment
source "/opt/ros/$ROS_DISTRO/setup.bash" --
source "/usr/share/gazebo/setup.sh"
exec "$@"
