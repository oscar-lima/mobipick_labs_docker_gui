from __future__ import annotations

import socket
from pathlib import Path

from mobipick_gui.display_runtime import (
    CONTAINER_XAUTHORITY,
    CONTAINER_WAYLAND_SOCKET,
    detect_display_runtime,
)


def _wayland_socket(path: Path) -> socket.socket:
    server = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    server.bind(str(path))
    return server


def test_auto_uses_x11_and_mounts_xauthority(tmp_path):
    x11_dir = tmp_path / 'x11'
    x11_dir.mkdir()
    xauthority = tmp_path / 'Xauthority'
    xauthority.write_text('cookie', encoding='utf-8')

    runtime = detect_display_runtime(
        environ={
            'DISPLAY': ':1',
            'XAUTHORITY': str(xauthority),
        },
        x11_socket_dir=x11_dir,
    )

    assert runtime.backend == 'x11'
    assert runtime.environment == {
        'DISPLAY': ':1',
        'QT_X11_NO_MITSHM': '1',
        'XAUTHORITY': CONTAINER_XAUTHORITY,
        'QT_QPA_PLATFORM': 'xcb',
    }
    assert (str(x11_dir), str(x11_dir), 'ro') in runtime.mounts
    assert (
        str(xauthority.resolve()),
        CONTAINER_XAUTHORITY,
        'ro',
    ) in runtime.mounts
    assert runtime.xauthority_mounted


def test_auto_uses_native_wayland_when_x11_is_unavailable(tmp_path):
    wayland_path = tmp_path / 'wayland-1'
    server = _wayland_socket(wayland_path)
    try:
        runtime = detect_display_runtime(
            environ={
                'XDG_RUNTIME_DIR': str(tmp_path),
                'WAYLAND_DISPLAY': wayland_path.name,
            },
            x11_socket_dir=tmp_path / 'missing-x11',
        )
    finally:
        server.close()

    assert runtime.backend == 'wayland'
    assert runtime.environment == {
        'WAYLAND_DISPLAY': 'wayland-1',
        'MOBIPICK_WAYLAND_SOCKET': CONTAINER_WAYLAND_SOCKET,
        'QT_QPA_PLATFORM': 'wayland',
    }
    assert runtime.mounts == (
        (str(wayland_path), CONTAINER_WAYLAND_SOCKET, 'rw'),
    )
    assert not runtime.x11_available


def test_auto_exposes_xwayland_and_wayland_but_prefers_xcb(tmp_path):
    x11_dir = tmp_path / 'x11'
    x11_dir.mkdir()
    wayland_path = tmp_path / 'wayland-0'
    server = _wayland_socket(wayland_path)
    try:
        runtime = detect_display_runtime(
            environ={
                'DISPLAY': ':0',
                'XDG_RUNTIME_DIR': str(tmp_path),
                'WAYLAND_DISPLAY': wayland_path.name,
            },
            x11_socket_dir=x11_dir,
        )
    finally:
        server.close()

    assert runtime.backend == 'x11'
    assert runtime.environment['QT_QPA_PLATFORM'] == 'xcb'
    assert runtime.environment['DISPLAY'] == ':0'
    assert runtime.environment['WAYLAND_DISPLAY'] == 'wayland-0'
    assert (str(x11_dir), str(x11_dir), 'ro') in runtime.mounts
    assert (
        str(wayland_path),
        CONTAINER_WAYLAND_SOCKET,
        'rw',
    ) in runtime.mounts


def test_forced_wayland_does_not_expose_x11(tmp_path):
    x11_dir = tmp_path / 'x11'
    x11_dir.mkdir()
    wayland_path = tmp_path / 'wayland-0'
    server = _wayland_socket(wayland_path)
    try:
        runtime = detect_display_runtime(
            mode='wayland',
            environ={
                'DISPLAY': ':0',
                'XDG_RUNTIME_DIR': str(tmp_path),
                'WAYLAND_DISPLAY': wayland_path.name,
            },
            x11_socket_dir=x11_dir,
        )
    finally:
        server.close()

    assert runtime.backend == 'wayland'
    assert 'DISPLAY' not in runtime.environment
    assert runtime.environment['QT_QPA_PLATFORM'] == 'wayland'
    assert all(source != str(x11_dir) for source, _, _ in runtime.mounts)


def test_missing_wayland_socket_is_not_mounted(tmp_path):
    runtime = detect_display_runtime(
        mode='wayland',
        environ={
            'XDG_RUNTIME_DIR': str(tmp_path),
            'WAYLAND_DISPLAY': 'wayland-0',
        },
    )

    assert runtime.backend == 'none'
    assert runtime.mounts == ()
    assert any('Wayland socket is not available' in item for item in runtime.warnings)
    assert any('No usable wayland' in item for item in runtime.warnings)


def test_relative_runtime_directory_is_rejected():
    runtime = detect_display_runtime(
        mode='wayland',
        environ={
            'XDG_RUNTIME_DIR': 'run/user/1000',
            'WAYLAND_DISPLAY': 'wayland-0',
        },
    )

    assert runtime.backend == 'none'
    assert any('non-absolute XDG_RUNTIME_DIR' in item for item in runtime.warnings)
