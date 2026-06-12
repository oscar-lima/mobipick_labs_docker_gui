#!/usr/bin/env bash

source_if_present() {
    local setup_file="$1"
    if [ -s "$setup_file" ]; then
        # shellcheck disable=SC1090
        source "$setup_file"
        return 0
    fi
    return 1
}

source_if_present /opt/ros/noetic/setup.bash
source_if_present /usr/share/gazebo/setup.sh

if [ "${MOBIPICK_WORKSPACE_ENABLED:-0}" = "1" ]; then
    # shellcheck disable=SC1091
    source /scripts_430ofkjl04fsw/ros_workspace_setup.bash
    mobipick_source_workspace_chain
    if [ "${MOBIPICK_WORKSPACE_BUILT:-0}" = "1" ]; then
        echo "ROS workspace ${MOBIPICK_WORKSPACE_NAME:-unknown} is active."
    else
        echo "ROS workspace ${MOBIPICK_WORKSPACE_NAME:-unknown} is not built; using available underlays and base ROS Noetic." >&2
    fi
else
    for setup_file in \
        "${HOME}/catkin_ws/devel/setup.bash" \
        "/root/catkin_ws/devel/setup.bash" \
        "/home/${MOBIPICK_HOST_USER:-user}/catkin_ws/devel/setup.bash"
    do
        if source_if_present "$setup_file"; then
            echo "Using Docker image workspace: $setup_file"
            break
        fi
    done
fi

if [ "${MOBIPICK_ROS_USE_IP:-1}" = "1" ]; then
    ros_ip="$(hostname -I 2>/dev/null | awk '{print $1}')"
    if [ -n "$ros_ip" ]; then
        export ROS_IP="$ros_ip"
        unset ROS_HOSTNAME
    fi
fi

exec "$@"
