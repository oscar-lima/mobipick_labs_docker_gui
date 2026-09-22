import json
import socket
import subprocess
from types import MethodType, SimpleNamespace

from mobipick_gui.main_window import MainWindow


def _harness(*, running=True, remote=False, payload=None, master=''):
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
        _current_master_uri=lambda: master,
    )
    for name in ('_host_ros_environment', '_remote_host_ros_environment'):
        setattr(
            harness,
            name,
            MethodType(getattr(MainWindow, name), harness),
        )
    return harness, calls


class _FakeSocket:
    def __init__(self, *args, **kwargs):
        self.connected = None
        self.closed = False

    def connect(self, address):
        self.connected = address

    def getsockname(self):
        return ('192.168.7.5', 45123)

    def close(self):
        self.closed = True


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


def test_host_ros_environment_uses_the_remote_master_and_route_address(
    monkeypatch,
):
    sockets = []

    def _socket(*args, **kwargs):
        sockets.append(_FakeSocket())
        return sockets[-1]

    monkeypatch.setattr(socket, 'socket', _socket)
    harness, calls = _harness(
        remote=True,
        running=False,
        master='http://mobipick-os-sensor:11311',
    )

    assert harness._host_ros_environment() == {
        'ROS_MASTER_URI': 'http://mobipick-os-sensor:11311',
        'ROS_IP': '192.168.7.5',
    }
    assert sockets[0].connected == ('mobipick-os-sensor', 11311)
    assert sockets[0].closed
    assert calls == []


def test_host_ros_environment_is_empty_when_the_remote_master_is_unreachable(
    monkeypatch,
):
    def _socket(*args, **kwargs):
        raise OSError('no route to host')

    monkeypatch.setattr(socket, 'socket', _socket)
    harness, calls = _harness(
        remote=True,
        running=False,
        master='http://mobipick-os-sensor:11311',
    )

    assert harness._host_ros_environment() == {}
    assert calls[-1][0] == 'log'


def test_host_ros_environment_is_empty_without_a_remote_master_uri():
    harness, calls = _harness(remote=True, running=False)

    assert harness._host_ros_environment() == {}
    assert calls == []


def test_host_ros_environment_rejects_missing_bridge_addresses():
    harness, calls = _harness(payload={'IPAddress': '', 'Gateway': ''})

    assert harness._host_ros_environment() == {}
    assert calls[-1][0] == 'log'
