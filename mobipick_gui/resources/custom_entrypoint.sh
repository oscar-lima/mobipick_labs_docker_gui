#!/bin/bash
# set -e

# setup gazebo and ros environment
# source "/usr/share/gazebo/setup.sh" --
source $HOME"/ros_ws/clean_mobipick_labs_ws/devel/setup.bash" --
echo "ROS workspace ros_ws/clean_mobipick_labs_ws is active."

exec "$@"
