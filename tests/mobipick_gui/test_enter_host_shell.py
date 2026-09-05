import importlib.util
import os
import stat
from pathlib import Path
from types import SimpleNamespace

import pytest


SCRIPT_PATH = (
    Path(__file__).parents[2]
    / 'mobipick_gui'
    / 'resources'
    / 'scripts'
    / 'enter_host_shell.py'
)
SPEC = importlib.util.spec_from_file_location('enter_host_shell', SCRIPT_PATH)
assert SPEC is not None and SPEC.loader is not None
ENTER_HOST_SHELL = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(ENTER_HOST_SHELL)


def test_runtime_validation_rejects_directory_owned_by_another_uid():
    class RuntimePath:
        @staticmethod
        def lstat():
            return SimpleNamespace(st_mode=stat.S_IFDIR | 0o700, st_uid=0)

        @staticmethod
        def is_symlink():
            return False

    assert not ENTER_HOST_SHELL._runtime_directory_is_valid(
        RuntimePath(),
        1001,
    )


def test_prepare_runtime_replaces_invalid_inherited_directory(
    monkeypatch,
    tmp_path,
):
    inherited = tmp_path / 'inherited-runtime'
    inherited.mkdir(mode=0o755)
    monkeypatch.setenv('TMPDIR', str(tmp_path))
    monkeypatch.setenv('XDG_RUNTIME_DIR', str(inherited))
    monkeypatch.delenv('MOBIPICK_WAYLAND_SOCKET', raising=False)
    monkeypatch.delenv('WAYLAND_DISPLAY', raising=False)

    runtime_dir = ENTER_HOST_SHELL._prepare_runtime_directory(
        os.getuid(),
        os.getgid(),
    )

    assert runtime_dir == tmp_path / f'mobipick-runtime-{os.getuid()}'
    assert os.environ['XDG_RUNTIME_DIR'] == str(runtime_dir)
    runtime_stat = runtime_dir.stat()
    assert runtime_stat.st_uid == os.getuid()
    assert stat.S_IMODE(runtime_stat.st_mode) == 0o700


def test_prepare_runtime_relinks_forwarded_wayland_socket(
    monkeypatch,
    tmp_path,
):
    forwarded_socket = tmp_path / 'forwarded-wayland.sock'
    forwarded_socket.touch()
    monkeypatch.setenv('TMPDIR', str(tmp_path))
    monkeypatch.delenv('XDG_RUNTIME_DIR', raising=False)
    monkeypatch.setenv('MOBIPICK_WAYLAND_SOCKET', str(forwarded_socket))
    monkeypatch.setenv('WAYLAND_DISPLAY', 'wayland-1')

    runtime_dir = ENTER_HOST_SHELL._prepare_runtime_directory(
        os.getuid(),
        os.getgid(),
    )

    assert runtime_dir is not None
    assert (runtime_dir / 'wayland-1').readlink() == forwarded_socket


def test_privilege_drop_prepares_runtime_for_target_uid(
    monkeypatch,
    tmp_path,
):
    target_uid = 1001
    target_gid = 1002
    events = []

    monkeypatch.setenv('MOBIPICK_UID', str(target_uid))
    monkeypatch.setenv('MOBIPICK_GID', str(target_gid))
    monkeypatch.setenv('MOBIPICK_HOST_HOME', str(tmp_path / 'home'))
    monkeypatch.setattr(ENTER_HOST_SHELL.os, 'getuid', lambda: 0)
    monkeypatch.setattr(ENTER_HOST_SHELL.os, 'getgid', lambda: 0)
    monkeypatch.setattr(ENTER_HOST_SHELL, '_ensure_group', lambda *_args: None)
    monkeypatch.setattr(
        ENTER_HOST_SHELL,
        '_ensure_user',
        lambda *_args: 'host-user',
    )
    monkeypatch.setattr(
        ENTER_HOST_SHELL,
        '_select_home',
        lambda _hint: (tmp_path / 'home', None),
    )
    monkeypatch.setattr(ENTER_HOST_SHELL, '_ensure_home_ownership', lambda *_args: None)
    monkeypatch.setattr(ENTER_HOST_SHELL, '_ensure_rc_stub', lambda *_args: None)
    monkeypatch.setattr(ENTER_HOST_SHELL, '_ensure_shadow_entry', lambda *_args: None)
    monkeypatch.setattr(ENTER_HOST_SHELL, '_enable_passwordless_sudo', lambda *_args: None)
    monkeypatch.setattr(ENTER_HOST_SHELL, '_relax_permissions', lambda *_args: None)
    monkeypatch.setattr(
        ENTER_HOST_SHELL,
        '_prepare_runtime_directory',
        lambda uid, gid: events.append(('runtime', uid, gid)),
    )
    monkeypatch.setattr(
        ENTER_HOST_SHELL.os,
        'setgroups',
        lambda groups: events.append(('groups', groups)),
    )
    monkeypatch.setattr(
        ENTER_HOST_SHELL.os,
        'setgid',
        lambda gid: events.append(('gid', gid)),
    )
    monkeypatch.setattr(
        ENTER_HOST_SHELL.os,
        'setuid',
        lambda uid: events.append(('uid', uid)),
    )

    def fake_execvp(*_args):
        raise RuntimeError('exec called')

    monkeypatch.setattr(ENTER_HOST_SHELL.os, 'execvp', fake_execvp)

    with pytest.raises(RuntimeError, match='exec called'):
        ENTER_HOST_SHELL.main(['enter_host_shell.py', 'bash'])

    assert events[:4] == [
        ('runtime', target_uid, target_gid),
        ('groups', [target_gid]),
        ('gid', target_gid),
        ('uid', target_uid),
    ]


def test_root_command_keeps_runtime_owned_by_root(monkeypatch):
    events = []
    monkeypatch.setenv('MOBIPICK_UID', '0')
    monkeypatch.setattr(ENTER_HOST_SHELL.os, 'getuid', lambda: 0)
    monkeypatch.setattr(ENTER_HOST_SHELL.os, 'getgid', lambda: 0)
    monkeypatch.setattr(
        ENTER_HOST_SHELL,
        '_prepare_runtime_directory',
        lambda uid, gid: events.append((uid, gid)),
    )

    def fake_execvp(*_args):
        raise RuntimeError('exec called')

    monkeypatch.setattr(ENTER_HOST_SHELL.os, 'execvp', fake_execvp)

    with pytest.raises(RuntimeError, match='exec called'):
        ENTER_HOST_SHELL.main(['enter_host_shell.py', 'bash'])

    assert events == [(0, 0)]
