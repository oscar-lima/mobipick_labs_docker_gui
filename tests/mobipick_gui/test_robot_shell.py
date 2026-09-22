"""Remote shells that ssh onto the robot while a remote ROS master is used."""
import os
import shutil
from types import MethodType, SimpleNamespace

import pytest

os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')

from mobipick_gui.main_window import MainWindow
from mobipick_gui.remote_adapter import MainWindowRemoteAdapter
from mobipick_gui.remote_control import RemoteControlError, RemoteShellSession

BASH = shutil.which('bash')
pytestmark_bash = pytest.mark.skipif(BASH is None, reason='bash is required')

DEFAULT_ROS_CFG = {
    'robot_ssh_user': 'robot',
    'robot_ssh_host': '',
    'robot_shell_by_default': True,
    'robot_ssh_options': ['-o', 'BatchMode=yes', '-o', 'ConnectTimeout=10'],
}


def _window(*, remote=True, ros_cfg=None, master='http://mobipick-os-sensor:11311'):
    logs = []
    html = []
    window = SimpleNamespace(
        _exit_in_progress=False,
        _ros_cfg=dict(DEFAULT_ROS_CFG if ros_cfg is None else ros_cfg),
        _remote_master_enabled=lambda: remote,
        _current_master_uri=lambda: master,
        _ensure_tab=lambda key, label, closable=False: SimpleNamespace(
            container_name='stale', exec_id='stale'
        ),
        _append_gui_html=lambda key, text: html.append((key, text)),
        _focus_tab=lambda key: None,
        _log_info=logs.append,
        _ensure_network=lambda log_key='log': logs.append('network'),
        # only the container branch needs these
        _terminal_run_as_root_requested=lambda: False,
        _grant_x=lambda source, log_key='log': logs.append('xhost'),
        _compose_env_args=lambda overrides, container_name='': [],
        _ros_tool_service=lambda: 'mobipick_remote_cmd',
        _prepare_run_env=lambda extra: {'env': {}},
        _project_root='/repo',
    )
    for name in (
        '_robot_ssh_target',
        '_robot_ssh_options',
        '_robot_shell_by_default',
    ):
        setattr(window, name, MethodType(getattr(MainWindow, name), window))
    return window, logs, html


# -- the ssh target ---------------------------------------------------------


def test_robot_target_is_the_master_host_with_the_configured_user():
    window, _, _ = _window()

    assert window._robot_ssh_target() == 'robot@mobipick-os-sensor'


def test_robot_target_is_empty_without_a_remote_master():
    window, _, _ = _window(remote=False)

    assert window._robot_ssh_target() == ''


def test_configured_robot_host_and_user_win_over_the_master_uri():
    window, _, _ = _window(
        ros_cfg={**DEFAULT_ROS_CFG, 'robot_ssh_host': 'other-robot', 'robot_ssh_user': ''}
    )

    assert window._robot_ssh_target() == 'other-robot'


def test_robot_ssh_options_accept_a_string_and_fall_back():
    window, _, _ = _window(ros_cfg={**DEFAULT_ROS_CFG, 'robot_ssh_options': '-o BatchMode=yes'})
    assert window._robot_ssh_options() == ['-o', 'BatchMode=yes']

    window, _, _ = _window(ros_cfg={'robot_ssh_user': 'robot'})
    assert window._robot_ssh_options() == ['-o', 'BatchMode=yes', '-o', 'ConnectTimeout=10']
    assert window._robot_shell_by_default() is True


# -- the shell spec ---------------------------------------------------------


def _spec(window, **kwargs):
    adapter = MainWindowRemoteAdapter(window)
    return adapter.shell_spec(3, 'Remote Shell 3', root=None, **kwargs)


def test_remote_master_mode_opens_the_shell_on_the_robot():
    window, logs, html = _window()

    spec = _spec(window)

    assert spec['argv'] == [
        'ssh', '-o', 'BatchMode=yes', '-o', 'ConnectTimeout=10',
        'robot@mobipick-os-sensor', 'bash', '--noprofile', '--norc',
    ]
    assert spec['runs_on'] == 'robot'
    assert spec['target'] == 'robot@mobipick-os-sensor'
    assert spec['container_name'] is None
    assert spec['signal_prefix'] == [
        'ssh', '-o', 'BatchMode=yes', '-o', 'ConnectTimeout=10',
        'robot@mobipick-os-sensor',
    ]
    assert spec['signal_quote'] is True
    # the robot's own master stays untouched; only an unset one is filled in
    assert 'ROS_MASTER_URI:-http://mobipick-os-sensor:11311' in spec['init_command']
    assert 'network' not in logs          # no Docker network for an ssh shell
    assert html and 'robot@mobipick-os-sensor' in html[0][1]


def test_robot_false_still_opens_a_container_shell():
    window, logs, _ = _window()

    spec = _spec(window, robot=False)

    assert spec['argv'][:3] == ['docker', 'compose', 'run']
    assert spec['runs_on'] == 'container'
    assert spec['container_name'].startswith('mobipick-remote-shell-')
    assert spec.get('signal_prefix') is None     # docker exec, from the name
    assert 'network' in logs and 'xhost' in logs


def test_local_master_keeps_the_container_shell_default():
    window, _, _ = _window(remote=False)

    assert _spec(window)['runs_on'] == 'container'


def test_robot_shell_can_be_switched_off_by_configuration():
    window, _, _ = _window(
        ros_cfg={**DEFAULT_ROS_CFG, 'robot_shell_by_default': False}
    )

    assert _spec(window)['runs_on'] == 'container'
    assert _spec(window, robot=True)['runs_on'] == 'robot'


def test_robot_shell_refused_without_a_remote_master():
    window, _, _ = _window(remote=False)
    adapter = MainWindowRemoteAdapter(window)

    with pytest.raises(RemoteControlError, match='remote ROS master mode is off'):
        adapter.shell_spec(3, 'Remote Shell 3', root=None, robot=True)


# -- signalling a shell on another machine ----------------------------------


@pytestmark_bash
@pytest.mark.skipif(not shutil.which('pkill'), reason='pkill required')
def test_interrupt_sends_one_quoted_word_through_ssh(tmp_path):
    recorder = tmp_path / 'fake_ssh'
    record_file = tmp_path / 'argv.txt'
    recorder.write_text(
        '#!/bin/sh\nfor a in "$@"; do printf "%s\\n" "$a" >> '
        f'{record_file}; done\n'
    )
    recorder.chmod(0o755)
    session = RemoteShellSession(
        1,
        'robot',
        [BASH, '--noprofile', '--norc'],
        signal_prefix=[str(recorder), 'robot@mobipick-os-sensor'],
        signal_quote=True,
        runs_on='robot',
        target='robot@mobipick-os-sensor',
    )
    try:
        init = session.run('echo "__MPRC_PID__ $$"')
        assert session.wait(init, 10.0)
        session.run('sleep 30')
        session.interrupt('INT')
    finally:
        session.close()
    args = record_file.read_text().splitlines()
    # ssh joins its arguments with spaces: every remote command must arrive as
    # one already-quoted word, not as 'sh', '-c', script
    assert args[0::2] == ['robot@mobipick-os-sensor'] * 2
    assert all(arg.startswith('sh -c ') for arg in args[1::2])
    assert 'pkill -INT -P' in args[1]
    # closing the session kills what the shell still ran on the robot
    assert 'pkill -KILL -P' in args[3]
    assert session.describe()['runs_on'] == 'robot'


@pytestmark_bash
def test_container_shell_keeps_its_docker_exec_prefix():
    session = RemoteShellSession(
        1,
        'container',
        [BASH, '--noprofile', '--norc'],
        container_name='mobipick-remote-shell-abc',
    )
    try:
        assert session.signal_prefix == [
            'docker', 'exec', 'mobipick-remote-shell-abc'
        ]
        assert session.signal_quote is False
        assert session.describe()['runs_on'] == 'container'
        assert session.describe()['target'] == 'mobipick-remote-shell-abc'
    finally:
        session.close()
