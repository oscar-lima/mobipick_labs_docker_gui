from types import MethodType, SimpleNamespace

from PyQt5.QtWidgets import QMessageBox

from mobipick_gui.main_window import MainWindow


class _Checkbox:
    def __init__(self, checked):
        self._checked = checked

    def isChecked(self):
        return self._checked


class _Button:
    def __init__(self):
        self.enabled = None

    def setEnabled(self, enabled):
        self.enabled = enabled


def _grace_harness(*, fast):
    harness = SimpleNamespace(
        fast_stop_checkbox=_Checkbox(fast),
        _ros_shutdown_grace_s=20.0,
        _ros_shutdown_grace_exit_s=5.0,
    )
    harness._fast_stop_enabled = MethodType(
        MainWindow._fast_stop_enabled,
        harness,
    )
    harness._shutdown_grace = MethodType(MainWindow._shutdown_grace, harness)
    return harness


def test_shutdown_grace_defaults_to_slow_and_fast_stop_skips_it():
    slow = _grace_harness(fast=False)
    fast = _grace_harness(fast=True)

    assert slow._shutdown_grace() == 20.0
    assert slow._shutdown_grace(exiting=True) == 5.0
    assert fast._shutdown_grace() == 0.0
    assert fast._shutdown_grace(exiting=True) == 0.0


def test_stopping_local_roscore_always_skips_the_grace():
    slow = _grace_harness(fast=False)

    assert slow._shutdown_grace(stopping_local_roscore=True) == 0.0


def test_fast_remote_stop_offers_manual_ros_cleanup():
    calls = []
    tab = SimpleNamespace(key='rviz')
    harness = SimpleNamespace(
        _shutdown_grace=lambda: 0.0,
        _fast_stop_enabled=lambda: True,
        _remote_master_enabled=lambda: True,
        _collect_container_commands=lambda *args, **kwargs: (
            calls.append(('collect', args, kwargs)) or [['stop']]
        ),
        _append_gui_html=lambda *args: calls.append(('html', args)),
        _docker_stop_display=lambda label: f'docker stop {label}',
        _run_command_sequence=lambda commands, **kwargs: (
            calls.append(('run', commands, kwargs['log_key'])),
            kwargs['on_finished'](),
        ),
        _offer_remote_ros_cleanup=lambda **kwargs: calls.append(
            ('offer', kwargs)
        ),
    )
    harness._graceful_stop_container = MethodType(
        MainWindow._graceful_stop_container,
        harness,
    )

    harness._graceful_stop_container(
        'rviz-container',
        tab,
        exec_id='exec-id',
        on_finished=lambda: calls.append(('finished',)),
    )

    collect = next(call for call in calls if call[0] == 'collect')
    assert collect[2]['grace_s'] == 0.0
    assert calls[-2:] == [('finished',), ('offer', {'log_key': 'rviz'})]


def test_slow_remote_stop_keeps_grace_and_does_not_offer_cleanup():
    calls = []
    harness = SimpleNamespace(
        _shutdown_grace=lambda: 20.0,
        _fast_stop_enabled=lambda: False,
        _remote_master_enabled=lambda: True,
        _collect_container_commands=lambda *args, **kwargs: (
            calls.append(kwargs) or [['stop']]
        ),
        _append_gui_html=lambda *args: None,
        _docker_stop_display=lambda label: f'docker stop {label}',
        _run_command_sequence=lambda commands, **kwargs: kwargs[
            'on_finished'
        ](),
        _offer_remote_ros_cleanup=lambda **kwargs: calls.append('offered'),
    )
    harness._graceful_stop_container = MethodType(
        MainWindow._graceful_stop_container,
        harness,
    )

    harness._graceful_stop_container('rviz-container')

    assert calls == [{'exec_id': None, 'log_key': 'log', 'grace_s': 20.0}]


def test_remote_ros_cleanup_runs_in_remote_tool_service(monkeypatch):
    calls = []
    button = _Button()
    harness = SimpleNamespace(
        clean_stale_ros_nodes_button=button,
        _remote_ros_cleanup_pending=True,
        _remote_master_enabled=lambda: True,
        _ensure_network=lambda **kwargs: calls.append(('network', kwargs)),
        _compose_env_args=lambda **kwargs: ['--env', 'ROS_MASTER_URI=robot'],
        _ros_tool_service=lambda: 'mobipick_remote_cmd',
        _wrap_line_buffered=lambda command: command,
        _append_gui_html=lambda *args: calls.append(('html', args)),
        _run_command_sequence=lambda commands, **kwargs: (
            calls.append(('run', commands, kwargs['log_key'])),
            kwargs['on_finished'](),
        ),
    )
    harness._clear_remote_ros_cleanup_offer = MethodType(
        MainWindow._clear_remote_ros_cleanup_offer,
        harness,
    )
    harness._run_remote_ros_cleanup = MethodType(
        MainWindow._run_remote_ros_cleanup,
        harness,
    )
    monkeypatch.setattr(
        QMessageBox,
        'warning',
        lambda *args, **kwargs: QMessageBox.Yes,
    )

    harness._run_remote_ros_cleanup()

    run = next(call for call in calls if call[0] == 'run')
    command = run[1][0]
    assert command[:3] == ['docker', 'compose', 'run']
    assert 'mobipick_remote_cmd' in command
    assert "printf 'y\\n' | rosnode cleanup" in command
    assert harness._remote_ros_cleanup_pending is False
    assert button.enabled is False
