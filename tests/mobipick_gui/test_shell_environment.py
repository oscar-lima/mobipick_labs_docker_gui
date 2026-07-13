import os
import subprocess
from pathlib import Path

os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')

from mobipick_gui.main_window import MainWindow


class _CommandWrapper:
    _sh_quote = staticmethod(MainWindow._sh_quote)


def test_line_buffered_wrapper_runs_compound_shell_commands(tmp_path):
    setup = tmp_path / 'setup.bash'
    setup.write_text('BUFFERING_TEST_VALUE=ready\n', encoding='utf-8')
    inner = f'source {setup!s}; printf "%s" "$BUFFERING_TEST_VALUE"'

    wrapped = MainWindow._wrap_line_buffered(_CommandWrapper(), inner)
    result = subprocess.run(
        ['bash', '--noprofile', '--norc', '-c', wrapped],
        check=False,
        capture_output=True,
        text=True,
    )

    assert result.returncode == 0, result.stderr
    assert result.stdout == 'ready'
    assert 'failed to run command' not in result.stderr


def test_workspace_setup_exports_catkin_workspace_root(tmp_path):
    workspace = tmp_path / 'demo_ws'
    workspace.mkdir()
    setup = (
        Path(__file__).parents[2]
        / 'mobipick_gui'
        / 'resources'
        / 'scripts'
        / 'ros_workspace_setup.bash'
    )

    result = subprocess.run(
        [
            'bash',
            '--noprofile',
            '--norc',
            '-c',
            (
                f'MOBIPICK_WORKSPACE_PATH={workspace!s}; '
                f'source {setup!s}; '
                'MOBIPICK_WORKSPACE_DEVEL_PATHS=; '
                'mobipick_source_workspace_chain; '
                'printf "%s" "$ROS_WORKSPACE"'
            ),
        ],
        check=False,
        capture_output=True,
        text=True,
    )

    assert result.returncode == 0, result.stderr
    assert result.stdout == str(workspace)


def test_terminal_catkin_build_is_pinned_to_selected_workspace(tmp_path):
    workspace = tmp_path / 'demo_ws'
    package = workspace / 'src' / 'nested' / 'demo_package'
    package.mkdir(parents=True)
    bin_dir = tmp_path / 'bin'
    bin_dir.mkdir()
    catkin = bin_dir / 'catkin'
    catkin.write_text(
        '#!/usr/bin/env bash\nprintf \'%s\\n\' "$@"\n',
        encoding='utf-8',
    )
    catkin.chmod(0o755)
    setup = (
        Path(__file__).parents[2]
        / 'mobipick_gui'
        / 'resources'
        / 'scripts'
        / 'ros_workspace_setup.bash'
    )

    result = subprocess.run(
        [
            'bash',
            '--noprofile',
            '--norc',
            '-c',
            (
                f'PATH={bin_dir!s}:$PATH; '
                f'MOBIPICK_WORKSPACE_PATH={workspace!s}; '
                f'source {setup!s}; '
                'mobipick_pin_catkin_build_workspace; '
                f'cd {package!s}; '
                'catkin build --this'
            ),
        ],
        check=False,
        capture_output=True,
        text=True,
    )

    assert result.returncode == 0, result.stderr
    assert result.stdout.splitlines() == [
        'build',
        '--workspace',
        str(workspace),
        '--this',
    ]
