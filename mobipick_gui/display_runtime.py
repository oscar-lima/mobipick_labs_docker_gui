"""Select host display transports for GUI applications in containers."""
from __future__ import annotations

import grp
import os
import socket
from dataclasses import dataclass
from pathlib import Path
from typing import Mapping

from .window_control import session_type

X11_SOCKET_DIR = Path('/tmp/.X11-unix')
DRI_DEVICE_DIR = Path('/dev/dri')
NVIDIA_VERSION_FILE = Path('/proc/driver/nvidia/version')
CONTAINER_XAUTHORITY = '/tmp/mobipick.Xauthority'
CONTAINER_WAYLAND_SOCKET = '/tmp/mobipick-wayland.sock'
BLOCKED_SESSION_BUS_ADDRESS = 'unix:path=/run/mobipick-no-session-bus'


@dataclass(frozen=True)
class DisplayRuntime:
    """Docker arguments and metadata for the selected display backend."""

    backend: str
    environment: dict[str, str]
    mounts: tuple[tuple[str, str, str], ...]
    x11_available: bool
    xauthority_mounted: bool
    warnings: tuple[str, ...]


def blocked_desktop_notification_environment() -> dict[str, str]:
    """Prevent container applications from reaching desktop notifications.

    Freedesktop notifications are delivered over the desktop session bus.
    Supplying an explicit unreachable address also prevents D-Bus from
    autolaunching a new session bus whose notification windows could appear
    on the forwarded X11 or Wayland display.
    """
    return {'DBUS_SESSION_BUS_ADDRESS': BLOCKED_SESSION_BUS_ADDRESS}


def graphics_device_group_environment(
    dri_device_dir: Path = DRI_DEVICE_DIR,
) -> dict[str, str]:
    """Return host graphics group IDs for Docker Compose interpolation.

    Device ownership is authoritative because group names and numeric IDs can
    differ between the host and container. Named host groups are only used
    when no matching DRI node is present.
    """
    primary_gid = os.getgid()
    return {
        'MOBIPICK_RENDER_GID': str(
            _graphics_group_id(
                dri_device_dir,
                'renderD*',
                'render',
                primary_gid,
            )
        ),
        'MOBIPICK_VIDEO_GID': str(
            _graphics_group_id(
                dri_device_dir,
                'card*',
                'video',
                primary_gid,
            )
        ),
    }


def container_hostname_environment(
    compose_run_env: Mapping[str, str],
) -> dict[str, str]:
    """Return Compose interpolation values that keep tool windows local.

    Mutter treats an X11 or XWayland window whose ``WM_CLIENT_MACHINE``
    differs from the compositor's hostname as remote, and GNOME Shell never
    matches remote windows to desktop entries, so they only ever show the
    generic executable icon.  Qt fills that property from the container's
    hostname, so one-off tool containers run with the host's hostname.

    ROS nodes are unaffected while they advertise container IPs.  When the
    configuration advertises hostnames instead, the disposable Docker
    hostname is kept so peers can still resolve the node URIs.
    """
    use_ip = str(compose_run_env.get('MOBIPICK_ROS_USE_IP', '1')).strip()
    if use_ip != '1':
        return {}
    hostname = socket.gethostname().strip()
    if not hostname:
        return {}
    return {'MOBIPICK_CONTAINER_HOSTNAME': hostname}


def ogre_glx_environment(
    display_runtime: DisplayRuntime,
    *,
    nvidia_version_file: Path = NVIDIA_VERSION_FILE,
) -> dict[str, str]:
    """Use XWayland for Noetic's GLX-only OGRE render windows.

    Qt can create a native Wayland window, but Ubuntu Focal's OGRE 1.9 GL
    renderer expects its parent handle to belong to X11. NVIDIA containers on
    a Wayland desktop also need GLX vendor selection for accelerated XWayland.
    """
    if (
        display_runtime.backend != 'wayland'
        or not display_runtime.x11_available
    ):
        return {}
    environment = {'QT_QPA_PLATFORM': 'xcb'}
    if nvidia_version_file.is_file():
        environment.update(
            {
                '__NV_PRIME_RENDER_OFFLOAD': '1',
                '__GLX_VENDOR_LIBRARY_NAME': 'nvidia',
            }
        )
    return environment


def _graphics_group_id(
    device_dir: Path,
    device_pattern: str,
    group_name: str,
    default_gid: int,
) -> int:
    try:
        for device in sorted(device_dir.glob(device_pattern)):
            return device.stat().st_gid
    except OSError:
        pass
    try:
        return grp.getgrnam(group_name).gr_gid
    except KeyError:
        return default_gid


def detect_display_runtime(
    mode: str = 'auto',
    environ: Mapping[str, str] | None = None,
    *,
    x11_socket_dir: Path = X11_SOCKET_DIR,
) -> DisplayRuntime:
    """Detect X11/XWayland and native Wayland transports on the host.

    In ``auto`` mode both usable transports are exposed and Qt follows the
    host's native desktop session. Wayland is preferred on Wayland sessions;
    X11 is selected on Xorg or as a fallback when Wayland is unavailable.
    """
    host_env = os.environ if environ is None else environ
    requested_mode = str(mode or 'auto').strip().lower()
    warnings: list[str] = []
    if requested_mode not in {'auto', 'x11', 'wayland'}:
        warnings.append(
            f'Unknown display mode {requested_mode!r}; using automatic mode.'
        )
        requested_mode = 'auto'

    display = str(host_env.get('DISPLAY') or '').strip()
    x11_available = bool(display)

    runtime_dir = str(host_env.get('XDG_RUNTIME_DIR') or '').strip()
    wayland_display = str(host_env.get('WAYLAND_DISPLAY') or '').strip()
    wayland_socket: Path | None = None
    if runtime_dir and wayland_display:
        socket_name = Path(wayland_display)
        if not Path(runtime_dir).is_absolute():
            warnings.append('Ignoring non-absolute XDG_RUNTIME_DIR.')
        elif socket_name.name != wayland_display:
            warnings.append('Ignoring unsafe WAYLAND_DISPLAY socket name.')
        else:
            candidate = Path(runtime_dir) / socket_name
            if candidate.is_socket():
                wayland_socket = candidate
            else:
                warnings.append(
                    f'Wayland socket is not available at {candidate}.'
                )

    wayland_available = wayland_socket is not None
    if requested_mode == 'x11':
        backend = 'x11' if x11_available else 'none'
        expose_x11 = x11_available
        expose_wayland = False
    elif requested_mode == 'wayland':
        backend = 'wayland' if wayland_available else 'none'
        expose_x11 = False
        expose_wayland = wayland_available
    else:
        host_session = session_type(host_env)
        if host_session == 'x11' and x11_available:
            backend = 'x11'
        elif wayland_available:
            backend = 'wayland'
        elif x11_available:
            backend = 'x11'
        else:
            backend = 'none'
        expose_x11 = x11_available
        expose_wayland = wayland_available

    if backend == 'none':
        warnings.append(
            f'No usable {requested_mode} display transport was detected.'
        )

    environment: dict[str, str] = {}
    mounts: list[tuple[str, str, str]] = []
    if expose_x11:
        environment['DISPLAY'] = display
        environment['QT_X11_NO_MITSHM'] = '1'
        if x11_socket_dir.is_dir():
            socket_path = str(x11_socket_dir)
            mounts.append((socket_path, socket_path, 'ro'))

    xauthority_mounted = False
    if expose_x11:
        xauthority = _find_xauthority(host_env)
        if xauthority is not None:
            mounts.append((str(xauthority), CONTAINER_XAUTHORITY, 'ro'))
            environment['XAUTHORITY'] = CONTAINER_XAUTHORITY
            xauthority_mounted = True

    if expose_wayland and wayland_socket is not None:
        environment['WAYLAND_DISPLAY'] = wayland_display
        environment['MOBIPICK_WAYLAND_SOCKET'] = CONTAINER_WAYLAND_SOCKET
        mounts.append(
            (
                str(wayland_socket),
                CONTAINER_WAYLAND_SOCKET,
                'rw',
            )
        )

    if backend == 'x11':
        environment['QT_QPA_PLATFORM'] = 'xcb'
    elif backend == 'wayland':
        environment['QT_QPA_PLATFORM'] = 'wayland'

    return DisplayRuntime(
        backend=backend,
        environment=environment,
        mounts=tuple(mounts),
        x11_available=expose_x11,
        xauthority_mounted=xauthority_mounted,
        warnings=tuple(warnings),
    )


def _find_xauthority(environ: Mapping[str, str]) -> Path | None:
    configured = str(environ.get('XAUTHORITY') or '').strip()
    candidates = [Path(configured)] if configured else []
    home = str(environ.get('HOME') or '').strip()
    if home:
        candidates.append(Path(home) / '.Xauthority')
    for candidate in candidates:
        if candidate.is_file():
            return candidate.resolve()
    return None
