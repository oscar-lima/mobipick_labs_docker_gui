from __future__ import annotations

from pathlib import Path
import socket
from types import SimpleNamespace

from mobipick_gui.display_runtime import (
    CONTAINER_XAUTHORITY,
    CONTAINER_WAYLAND_SOCKET,
    DisplayRuntime,
    container_hostname_environment,
    detect_display_runtime,
    graphics_device_group_environment,
    ogre_glx_environment,
)


def _wayland_socket(path: Path) -> socket.socket:
    server = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    server.bind(str(path))
    return server


def test_graphics_groups_follow_dri_device_ownership(tmp_path):
    dri_dir = tmp_path / 'dri'
    dri_dir.mkdir()
    render_node = dri_dir / 'renderD128'
    card_node = dri_dir / 'card1'
    render_node.touch()
    card_node.touch()

    environment = graphics_device_group_environment(dri_dir)

    assert environment == {
        'MOBIPICK_RENDER_GID': str(render_node.stat().st_gid),
        'MOBIPICK_VIDEO_GID': str(card_node.stat().st_gid),
    }


def test_graphics_groups_fall_back_to_named_host_groups(
    tmp_path,
    monkeypatch,
):
    group_ids = {'render': 992, 'video': 44}
    monkeypatch.setattr(
        'mobipick_gui.display_runtime.grp.getgrnam',
        lambda name: SimpleNamespace(gr_gid=group_ids[name]),
    )

    environment = graphics_device_group_environment(tmp_path / 'missing')

    assert environment == {
        'MOBIPICK_RENDER_GID': '992',
        'MOBIPICK_VIDEO_GID': '44',
    }


def test_container_hostname_follows_host_when_ros_advertises_ips(monkeypatch):
    monkeypatch.setattr(
        'mobipick_gui.display_runtime.socket.gethostname',
        lambda: 'lab-desktop',
    )

    assert container_hostname_environment({'MOBIPICK_ROS_USE_IP': '1'}) == {
        'MOBIPICK_CONTAINER_HOSTNAME': 'lab-desktop'
    }
    assert container_hostname_environment({}) == {
        'MOBIPICK_CONTAINER_HOSTNAME': 'lab-desktop'
    }


def test_container_hostname_is_kept_when_ros_advertises_hostnames(monkeypatch):
    monkeypatch.setattr(
        'mobipick_gui.display_runtime.socket.gethostname',
        lambda: 'lab-desktop',
    )

    assert container_hostname_environment({'MOBIPICK_ROS_USE_IP': '0'}) == {}


def test_container_hostname_is_omitted_without_a_host_name(monkeypatch):
    monkeypatch.setattr(
        'mobipick_gui.display_runtime.socket.gethostname',
        lambda: '',
    )

    assert container_hostname_environment({'MOBIPICK_ROS_USE_IP': '1'}) == {}


def test_ogre_uses_nvidia_glx_through_xwayland(tmp_path):
    runtime = DisplayRuntime(
        backend='wayland',
        environment={'QT_QPA_PLATFORM': 'wayland'},
        mounts=(),
        x11_available=True,
        xauthority_mounted=False,
        warnings=(),
    )
    nvidia_version = tmp_path / 'nvidia-version'
    nvidia_version.touch()

    environment = ogre_glx_environment(
        runtime,
        nvidia_version_file=nvidia_version,
    )

    assert environment == {
        'QT_QPA_PLATFORM': 'xcb',
        '__NV_PRIME_RENDER_OFFLOAD': '1',
        '__GLX_VENDOR_LIBRARY_NAME': 'nvidia',
    }


def test_ogre_keeps_native_backend_without_xwayland():
    runtime = DisplayRuntime(
        backend='wayland',
        environment={'QT_QPA_PLATFORM': 'wayland'},
        mounts=(),
        x11_available=False,
        xauthority_mounted=False,
        warnings=(),
    )

    assert ogre_glx_environment(runtime) == {}


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


def test_auto_exposes_both_transports_but_prefers_native_wayland(tmp_path):
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

    assert runtime.backend == 'wayland'
    assert runtime.environment['QT_QPA_PLATFORM'] == 'wayland'
    assert runtime.environment['DISPLAY'] == ':0'
    assert runtime.environment['WAYLAND_DISPLAY'] == 'wayland-0'
    assert (str(x11_dir), str(x11_dir), 'ro') in runtime.mounts
    assert (
        str(wayland_path),
        CONTAINER_WAYLAND_SOCKET,
        'rw',
    ) in runtime.mounts


def test_auto_uses_x11_for_an_xorg_session_even_with_wayland_socket(tmp_path):
    x11_dir = tmp_path / 'x11'
    x11_dir.mkdir()
    wayland_path = tmp_path / 'wayland-0'
    server = _wayland_socket(wayland_path)
    try:
        runtime = detect_display_runtime(
            environ={
                'XDG_SESSION_TYPE': 'x11',
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
