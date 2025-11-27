#!/bin/bash
# set -e

# setup gazebo and ros environment
source "/usr/share/gazebo/setup.sh" --
source $HOME"/{{ros1_workspace}}/devel/setup.bash" --
echo "ROS workspace {{ros1_workspace}} is active."

exec "$@"
