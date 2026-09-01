"""Select host display transports for GUI applications in containers."""
from __future__ import annotations

import os
from dataclasses import dataclass
from pathlib import Path
from typing import Mapping


X11_SOCKET_DIR = Path('/tmp/.X11-unix')
CONTAINER_XAUTHORITY = '/tmp/mobipick.Xauthority'


@dataclass(frozen=True)
class DisplayRuntime:
    """Docker arguments and metadata for the selected display backend."""

    backend: str
    environment: dict[str, str]
    mounts: tuple[tuple[str, str, str], ...]
    x11_available: bool
    xauthority_mounted: bool
    warnings: tuple[str, ...]


def detect_display_runtime(
    mode: str = 'auto',
    environ: Mapping[str, str] | None = None,
    *,
    x11_socket_dir: Path = X11_SOCKET_DIR,
) -> DisplayRuntime:
    """Detect X11/XWayland and native Wayland transports on the host.

    In ``auto`` mode both usable transports are exposed, while Qt defaults to
    X11/XWayland for compatibility with ROS Noetic RViz, Gazebo, and OGRE.
    Native Wayland is selected automatically only when X11 is unavailable.
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
        backend = 'x11' if x11_available else (
            'wayland' if wayland_available else 'none'
        )
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
        socket_path = str(wayland_socket)
        environment['XDG_RUNTIME_DIR'] = runtime_dir
        environment['WAYLAND_DISPLAY'] = wayland_display
        mounts.append((socket_path, socket_path, 'rw'))

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
