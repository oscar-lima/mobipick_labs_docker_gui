import subprocess
from pathlib import Path


def test_entrypoint_uses_ros_ip_instead_of_disposable_hostname():
    entrypoint = (
        Path(__file__).parents[2]
        / 'mobipick_gui'
        / 'resources'
        / 'custom_entrypoint.sh'
    ).read_text(encoding='utf-8')

    assert 'socket.getaddrinfo' in entrypoint
    assert 'sock.getsockname()[0]' in entrypoint
    assert 'export ROS_IP="$ros_ip"' in entrypoint
    assert 'unset ROS_HOSTNAME' in entrypoint


def test_entrypoint_exposes_unbuilt_workspace_sources():
    entrypoint = (
        Path(__file__).parents[2]
        / 'mobipick_gui'
        / 'resources'
        / 'custom_entrypoint.sh'
    ).read_text(encoding='utf-8')

    assert 'source /scripts_430ofkjl04fsw/ros_workspace_setup.bash' in entrypoint
    assert 'mobipick_source_workspace_chain' in entrypoint


def test_terminal_rc_restores_ros_shell_functions():
    terminal_rc = (
        Path(__file__).parents[2]
        / 'mobipick_gui'
        / 'resources'
        / 'scripts'
        / 'terminal.bashrc'
    ).read_text(encoding='utf-8')

    bash_rc_index = terminal_rc.index('source /etc/bash.bashrc')
    ros_setup_index = terminal_rc.index('source /opt/ros/noetic/setup.bash')
    workspace_setup_index = terminal_rc.index(
        'source /scripts_430ofkjl04fsw/ros_workspace_setup.bash'
    )

    assert bash_rc_index < ros_setup_index < workspace_setup_index
    assert 'source "${HOME}/.bashrc"' not in terminal_rc
    assert 'mobipick_pin_catkin_build_workspace' in terminal_rc


def test_terminal_rc_loads_image_helpers_but_skips_ros1(tmp_path):
    scripts = tmp_path / 'scripts'
    personal_config = scripts / 'personal_config'
    programs = scripts / 'programs'
    personal_config.mkdir(parents=True)
    for name in ('general', 'git', 'ros1'):
        (programs / name).mkdir(parents=True)
        (programs / name / 'alias.sh').write_text(
            f"alias {name}_loaded='true'\n",
            encoding='utf-8',
        )
    (scripts / 'permanent.sh').write_text(
        '''
load_if_exists() {
    test -f "$1" && source "$1"
}
load_program() {
    load_if_exists "$HOME/scripts/programs/$1/alias.sh"
}
''',
        encoding='utf-8',
    )
    (personal_config / 'programs_to_load.sh').write_text(
        'program_list=(general git ros1)\n',
        encoding='utf-8',
    )
    terminal_rc = (
        Path(__file__).parents[2]
        / 'mobipick_gui'
        / 'resources'
        / 'scripts'
        / 'terminal.bashrc'
    )

    result = subprocess.run(
        [
            'bash',
            '--noprofile',
            '--norc',
            '-c',
            (
                f'HOME={tmp_path!s}; '
                'MOBIPICK_WORKSPACE_ENABLED=0; '
                f'source {terminal_rc!s}; '
                'alias general_loaded; '
                'alias git_loaded; '
                '! alias ros1_loaded'
            ),
        ],
        check=False,
        capture_output=True,
        text=True,
    )

    assert result.returncode == 0, result.stderr
    assert "alias general_loaded='true'" in result.stdout
    assert "alias git_loaded='true'" in result.stdout
    assert 'ros1_loaded' not in result.stdout


def test_workspace_setup_sources_underlays_then_overlay_locals():
    setup = (
        Path(__file__).parents[2]
        / 'mobipick_gui'
        / 'resources'
        / 'scripts'
        / 'ros_workspace_setup.bash'
    ).read_text(encoding='utf-8')

    assert 'local mode="setup"' in setup
    assert 'mode="local"' in setup
    assert 'CATKIN_SHELL=bash' in setup
    assert '_CATKIN_SETUP_DIR="$devel_path"' in setup
    assert 'CATKIN_SETUP_UTIL_ARGS="--extend --local"' in setup
    assert 'mobipick_prepare_workspace_aliases' in setup
    assert 'mobipick_normalize_workspace_environment' in setup
    assert 'mobipick_use_private_devel_paths' in setup


def test_workspace_setup_normalizes_legacy_paths(tmp_path):
    canonical_root = tmp_path / 'home' / 'ros_ws'
    legacy_root = tmp_path / 'home' / 'ros1_ws'
    canonical_root.mkdir(parents=True)
    setup = (
        Path(__file__).parents[2]
        / 'mobipick_gui'
        / 'resources'
        / 'scripts'
        / 'ros_workspace_setup.bash'
    )
    command = f'''
set -e
MOBIPICK_WORKSPACE_MOUNT_TARGET={canonical_root}
MOBIPICK_WORKSPACE_COMPAT_ROOTS={legacy_root}
source {setup}
mobipick_prepare_workspace_aliases
test "$(readlink {legacy_root})" = "{canonical_root}"
ROS_PACKAGE_PATH={legacy_root}/demo/src:{canonical_root}/demo/src:/opt/ros
CMAKE_PREFIX_PATH={legacy_root}/demo/devel:/opt/ros
mobipick_normalize_workspace_environment
test "$ROS_PACKAGE_PATH" = "{canonical_root}/demo/src:/opt/ros"
test "$CMAKE_PREFIX_PATH" = "{canonical_root}/demo/devel:/opt/ros"
mobipick_remove_workspace_aliases
test ! -e {legacy_root}
'''

    result = subprocess.run(
        ['bash', '--noprofile', '--norc', '-c', command],
        check=False,
        capture_output=True,
        text=True,
    )

    assert result.returncode == 0, result.stderr


def test_workspace_setup_replaces_legacy_linked_devel_paths(tmp_path):
    canonical_root = tmp_path / 'home' / 'ros_ws'
    legacy_root = tmp_path / 'home' / 'ros1_ws'
    devel = canonical_root / 'demo_ws' / 'devel'
    package_prefix = devel / '.private' / 'demo_package'
    (package_prefix / 'bin').mkdir(parents=True)
    (package_prefix / 'lib' / 'pkgconfig').mkdir(parents=True)
    python_path = package_prefix / 'lib' / 'python3' / 'dist-packages'
    python_path.mkdir(parents=True)
    (devel / 'setup.bash').symlink_to(
        legacy_root
        / 'demo_ws'
        / 'devel'
        / '.private'
        / 'catkin_tools_prebuild'
        / 'setup.bash'
    )
    setup = (
        Path(__file__).parents[2]
        / 'mobipick_gui'
        / 'resources'
        / 'scripts'
        / 'ros_workspace_setup.bash'
    )
    command = f'''
set -e
MOBIPICK_WORKSPACE_COMPAT_ROOTS={legacy_root}
MOBIPICK_WORKSPACE_DEVEL_PATHS={devel}
CMAKE_PREFIX_PATH={devel}:/opt/ros
LD_LIBRARY_PATH={devel}/lib:/opt/ros/lib
PATH={devel}/bin:/usr/bin
PKG_CONFIG_PATH={devel}/lib/pkgconfig
PYTHONPATH={devel}/lib/python3/dist-packages
source {setup}
mobipick_use_private_devel_paths
test "$CMAKE_PREFIX_PATH" = "{package_prefix}:/opt/ros"
test "$LD_LIBRARY_PATH" = "{package_prefix}/lib:/opt/ros/lib"
test "$PATH" = "{package_prefix}/bin:/usr/bin"
test "$PKG_CONFIG_PATH" = "{package_prefix}/lib/pkgconfig"
test "$PYTHONPATH" = "{python_path}"
'''

    result = subprocess.run(
        ['bash', '--noprofile', '--norc', '-c', command],
        check=False,
        capture_output=True,
        text=True,
    )

    assert result.returncode == 0, result.stderr
