import json
import subprocess
from types import MethodType, SimpleNamespace

from mobipick_gui.main_window import MainWindow


def _harness(*, running=True, remote=False, payload=None):
    calls = []

    def _sp_run(args, **kwargs):
        calls.append((args, kwargs))
        return subprocess.CompletedProcess(
            args,
            0,
            stdout=json.dumps(payload) if payload is not None else '',
        )

    harness = SimpleNamespace(
        _remote_master_enabled=lambda: remote,
        is_roscore_running=lambda: running,
        _roscore_container_name='mobipick-roscore',
        _sp_run=_sp_run,
        _log_info=lambda message: calls.append(('log', message)),
    )
    harness._host_ros_environment = MethodType(
        MainWindow._host_ros_environment,
        harness,
    )
    return harness, calls


def test_host_ros_environment_uses_roscore_bridge_addresses():
    harness, calls = _harness(
        payload={'IPAddress': '172.20.0.2', 'Gateway': '172.20.0.1'}
    )

    environment = harness._host_ros_environment()

    assert environment == {
        'ROS_MASTER_URI': 'http://172.20.0.2:11311',
        'ROS_IP': '172.20.0.1',
    }
    assert calls[0][0][-1] == 'mobipick-roscore'


def test_host_ros_environment_is_empty_without_local_roscore():
    harness, calls = _harness(running=False)

    assert harness._host_ros_environment() == {}
    assert calls == []


def test_host_ros_environment_is_empty_for_remote_master_mode():
    harness, calls = _harness(remote=True)

    assert harness._host_ros_environment() == {}
    assert calls == []


def test_host_ros_environment_rejects_missing_bridge_addresses():
    harness, calls = _harness(payload={'IPAddress': '', 'Gateway': ''})

    assert harness._host_ros_environment() == {}
    assert calls[-1][0] == 'log'
