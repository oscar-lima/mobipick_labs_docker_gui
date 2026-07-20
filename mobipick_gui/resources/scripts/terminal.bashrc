# Load standard Bash behavior without the image's stale workspace selector.
if [ -r /etc/bash.bashrc ]; then
    # shellcheck disable=SC1091
    source /etc/bash.bashrc
fi

if command -v dircolors >/dev/null 2>&1; then
    if [ -r "${HOME}/.dircolors" ]; then
        eval "$(dircolors -b "${HOME}/.dircolors")"
    else
        eval "$(dircolors -b)"
    fi
    alias ls='ls --color=auto'
fi

if [ -r "${HOME}/.bash_aliases" ]; then
    # shellcheck disable=SC1090
    source "${HOME}/.bash_aliases"
fi

mobipick_load_image_shell_programs() {
    local scripts_root="${HOME}/scripts"
    local programs_file="${scripts_root}/personal_config/programs_to_load.sh"
    local program
    local -a program_list=()

    [ -r "${scripts_root}/permanent.sh" ] || return
    [ -r "$programs_file" ] || return

    # Import the scripts framework without running its unfiltered main loop.
    # shellcheck disable=SC1090
    source "${scripts_root}/permanent.sh" --source-only

    if [ -r "${scripts_root}/personal_config/personal_config.sh" ]; then
        # shellcheck disable=SC1090
        source "${scripts_root}/personal_config/personal_config.sh"
    fi

    # shellcheck disable=SC1090
    source "$programs_file"
    for program in "${program_list[@]}"; do
        # ROS workspace selection belongs to the GUI and is sourced below.
        [ "$program" = "ros1" ] && continue
        load_program "$program"
    done
}

mobipick_load_image_shell_programs

if [ -s /opt/ros/noetic/setup.bash ]; then
    # shellcheck disable=SC1091
    source /opt/ros/noetic/setup.bash
fi

if [ -s /usr/share/gazebo/setup.sh ]; then
    # shellcheck disable=SC1091
    source /usr/share/gazebo/setup.sh
fi

if [ "${MOBIPICK_WORKSPACE_ENABLED:-0}" = "1" ]; then
    # shellcheck disable=SC1091
    source /scripts_430ofkjl04fsw/ros_workspace_setup.bash
    mobipick_source_workspace_chain
    mobipick_pin_catkin_build_workspace
fi
