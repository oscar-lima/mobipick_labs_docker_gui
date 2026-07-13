mobipick_source_setup() {
    local devel_path="$1"
    local mode="$2"
    local setup_prefix=""
    if [ "$mode" = "local" ]; then
        setup_prefix="local_"
    fi
    local bash_setup="${devel_path}/${setup_prefix}setup.bash"
    local shell_setup="${devel_path}/${setup_prefix}setup.sh"

    if [ -s "$bash_setup" ]; then
        # shellcheck disable=SC1090
        source "$bash_setup"
        return 0
    fi
    if [ -s "$shell_setup" ]; then
        CATKIN_SHELL=bash
        _CATKIN_SETUP_DIR="$devel_path"
        export CATKIN_SHELL
        export _CATKIN_SETUP_DIR
        if [ "$mode" = "local" ]; then
            CATKIN_SETUP_UTIL_ARGS="--extend --local"
            export CATKIN_SETUP_UTIL_ARGS
        fi
        # shellcheck disable=SC1090
        source "$shell_setup"
        unset CATKIN_SHELL
        unset CATKIN_SETUP_UTIL_ARGS
        unset _CATKIN_SETUP_DIR
        return 0
    fi
    return 1
}

mobipick_prepare_workspace_aliases() {
    local compat_root
    local canonical_root="${MOBIPICK_WORKSPACE_MOUNT_TARGET:-}"
    local old_ifs="$IFS"

    MOBIPICK_CREATED_COMPAT_ROOTS=""
    [ -n "$canonical_root" ] || return
    IFS=':'
    for compat_root in ${MOBIPICK_WORKSPACE_COMPAT_ROOTS:-}; do
        [ -n "$compat_root" ] || continue
        [ "$compat_root" != "$canonical_root" ] || continue
        if [ -L "$compat_root" ]; then
            continue
        fi
        if [ -e "$compat_root" ]; then
            echo "Cannot map legacy ROS path $compat_root: path already exists." >&2
            continue
        fi
        if mkdir -p "$(dirname "$compat_root")" 2>/dev/null; then
            if ln -s "$canonical_root" "$compat_root" 2>/dev/null; then
                MOBIPICK_CREATED_COMPAT_ROOTS="${MOBIPICK_CREATED_COMPAT_ROOTS:+${MOBIPICK_CREATED_COMPAT_ROOTS}:}${compat_root}"
            fi
        fi
    done
    IFS="$old_ifs"
}

mobipick_remove_workspace_aliases() {
    local compat_root
    local canonical_root="${MOBIPICK_WORKSPACE_MOUNT_TARGET:-}"
    local old_ifs="$IFS"

    IFS=':'
    for compat_root in ${MOBIPICK_CREATED_COMPAT_ROOTS:-}; do
        [ -L "$compat_root" ] || continue
        if [ "$(readlink "$compat_root" 2>/dev/null)" = "$canonical_root" ]; then
            rm -f "$compat_root"
        fi
    done
    IFS="$old_ifs"
    unset MOBIPICK_CREATED_COMPAT_ROOTS
}

mobipick_normalize_path_list() {
    local variable_name="$1"
    local value
    local path
    local compat_root
    local canonical_root="${MOBIPICK_WORKSPACE_MOUNT_TARGET:-}"
    local old_ifs="$IFS"
    local normalized=""

    eval "value=\${${variable_name}:-}"
    [ -n "$value" ] || return 0

    IFS=':'
    for path in $value; do
        for compat_root in ${MOBIPICK_WORKSPACE_COMPAT_ROOTS:-}; do
            case "$path" in
                "$compat_root")
                    path="$canonical_root"
                    ;;
                "$compat_root"/*)
                    path="${canonical_root}${path#"$compat_root"}"
                    ;;
            esac
        done
        case ":$normalized:" in
            *":$path:"*) ;;
            *) normalized="${normalized:+${normalized}:}${path}" ;;
        esac
    done
    IFS="$old_ifs"
    export "$variable_name=$normalized"
}

mobipick_normalize_workspace_environment() {
    local variable_name
    for variable_name in \
        CMAKE_PREFIX_PATH \
        LD_LIBRARY_PATH \
        PATH \
        PKG_CONFIG_PATH \
        PYTHONPATH \
        ROS_PACKAGE_PATH
    do
        mobipick_normalize_path_list "$variable_name"
    done
}

mobipick_remove_path_prefix() {
    local variable_name="$1"
    local prefix="$2"
    local value
    local path
    local old_ifs="$IFS"
    local filtered=""

    eval "value=\${${variable_name}:-}"
    [ -n "$value" ] || return 0

    IFS=':'
    for path in $value; do
        case "$path" in
            "$prefix"|"$prefix"/*) continue ;;
        esac
        filtered="${filtered:+${filtered}:}${path}"
    done
    IFS="$old_ifs"
    export "$variable_name=$filtered"
}

mobipick_prepend_path() {
    local variable_name="$1"
    local path="$2"
    local value

    [ -d "$path" ] || return 0
    eval "value=\${${variable_name}:-}"
    case ":$value:" in
        *":$path:"*) return 0 ;;
    esac
    export "$variable_name=${path}${value:+:${value}}"
}

mobipick_devel_uses_compat_root() {
    local devel_path="$1"
    local setup_file
    local target
    local compat_root
    local old_ifs="$IFS"

    for setup_file in \
        "$devel_path/setup.bash" \
        "$devel_path/setup.sh" \
        "$devel_path/local_setup.bash" \
        "$devel_path/local_setup.sh"
    do
        [ -L "$setup_file" ] || continue
        target="$(readlink "$setup_file" 2>/dev/null)"
        IFS=':'
        for compat_root in ${MOBIPICK_WORKSPACE_COMPAT_ROOTS:-}; do
            case "$target" in
                "$compat_root"|"$compat_root"/*)
                    IFS="$old_ifs"
                    return 0
                    ;;
            esac
        done
        IFS="$old_ifs"
    done
    return 1
}

mobipick_use_private_devel_paths() {
    local devel_path
    local package_prefix
    local python_path
    local old_ifs="$IFS"

    IFS=':'
    for devel_path in ${MOBIPICK_WORKSPACE_DEVEL_PATHS:-}; do
        IFS="$old_ifs"
        if ! mobipick_devel_uses_compat_root "$devel_path"; then
            IFS=':'
            continue
        fi

        mobipick_remove_path_prefix CMAKE_PREFIX_PATH "$devel_path"
        mobipick_remove_path_prefix LD_LIBRARY_PATH "$devel_path"
        mobipick_remove_path_prefix PATH "$devel_path"
        mobipick_remove_path_prefix PKG_CONFIG_PATH "$devel_path"
        mobipick_remove_path_prefix PYTHONPATH "$devel_path"

        for package_prefix in "$devel_path"/.private/*; do
            [ -d "$package_prefix" ] || continue
            mobipick_prepend_path CMAKE_PREFIX_PATH "$package_prefix"
            mobipick_prepend_path LD_LIBRARY_PATH "$package_prefix/lib"
            mobipick_prepend_path PATH "$package_prefix/bin"
            mobipick_prepend_path \
                PKG_CONFIG_PATH \
                "$package_prefix/lib/pkgconfig"
            for python_path in \
                "$package_prefix"/lib/python*/dist-packages \
                "$package_prefix"/lib/python*/site-packages
            do
                mobipick_prepend_path PYTHONPATH "$python_path"
            done
        done
        IFS=':'
    done
    IFS="$old_ifs"
}

mobipick_source_workspace_chain() {
    local devel_path
    local mode="setup"
    local old_ifs="$IFS"

    mobipick_prepare_workspace_aliases

    IFS=':'
    for devel_path in ${MOBIPICK_WORKSPACE_DEVEL_PATHS:-}; do
        [ -n "$devel_path" ] || continue
        if mobipick_source_setup "$devel_path" "$mode"; then
            mode="local"
        fi
    done
    IFS="$old_ifs"

    mobipick_normalize_workspace_environment
    mobipick_use_private_devel_paths
    # Keep the conventional workspace variable separate from ROS_PACKAGE_PATH,
    # which contains source and package directories.
    export ROS_WORKSPACE="${MOBIPICK_WORKSPACE_PATH%/}"
    if [ -n "${MOBIPICK_ROS_PACKAGE_PATH:-}" ]; then
        export ROS_PACKAGE_PATH="${MOBIPICK_ROS_PACKAGE_PATH}${ROS_PACKAGE_PATH:+:${ROS_PACKAGE_PATH}}"
        mobipick_normalize_path_list ROS_PACKAGE_PATH
    fi
    mobipick_remove_workspace_aliases
}

mobipick_pin_catkin_build_workspace() {
    # catkin_tools normally discovers its workspace by walking up from the
    # current directory to the nearest .catkin_tools directory. A stale marker
    # in src/ therefore breaks package-local `catkin build --this` calls. Pin
    # builds to the workspace selected by the GUI, while preserving an explicit
    # workspace supplied by the user.
    function catkin {
        local argument

        if [ "${1:-}" = "build" ] && [ -n "${MOBIPICK_WORKSPACE_PATH:-}" ]; then
            for argument in "$@"; do
                case "$argument" in
                    -w|--workspace|--workspace=*)
                        command catkin "$@"
                        return
                        ;;
                esac
            done
            command catkin build --workspace "$MOBIPICK_WORKSPACE_PATH" "${@:2}"
            return
        fi

        command catkin "$@"
    }
}
