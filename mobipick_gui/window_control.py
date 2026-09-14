"""Window enumeration and placement backends for layout capture and replay.

``X11WindowBackend`` shells out to ``wmctrl``/``xprop`` and only sees X11 or
XWayland windows. ``GnomeWaylandWindowBackend`` talks over D-Bus to the
GNOME Shell extension shipped in ``resources/gnome-shell-extension`` so that
native Wayland windows can be captured and rearranged as well.
"""
from __future__ import annotations

import ast
import json
import os
import re
import shutil
import subprocess
from dataclasses import dataclass
from pathlib import Path
from typing import Callable, Mapping

GNOME_EXTENSION_UUID = 'winctl@mobipick-labs-docker-gui'
GNOME_EXTENSION_SOURCE = (
    Path(__file__).resolve().parent
    / 'resources'
    / 'gnome-shell-extension'
    / GNOME_EXTENSION_UUID
)
GNOME_EXTENSION_BUS_NAME = 'org.gnome.Shell'
GNOME_EXTENSION_OBJECT_PATH = '/org/gnome/Shell/Extensions/MobipickWinCtl'
GNOME_EXTENSION_INTERFACE = 'org.gnome.Shell.Extensions.MobipickWinCtl'
GNOME_EXTENSION_INSTALL_COMMAND = (
    'mobipick-labs-docker-gui --install-gnome-window-extension'
)

LogFn = Callable[[str], None]


@dataclass
class WindowInfo:
    wid: str
    title: str
    desktop: int | None
    pid: int | None
    x: int
    y: int
    width: int
    height: int
    wm_class: list[str]
    stack_index: int | None = None


def session_type(environ: Mapping[str, str] | None = None) -> str:
    """Return ``wayland``, ``x11``, or ``''`` for the current desktop session."""
    env = os.environ if environ is None else environ
    value = str(env.get('XDG_SESSION_TYPE') or '').strip().lower()
    if value in {'wayland', 'x11'}:
        return value
    qpa_platform = str(env.get('QT_QPA_PLATFORM') or '').strip().lower()
    if qpa_platform.startswith('wayland'):
        return 'wayland'
    if qpa_platform in {'xcb', 'x11'}:
        return 'x11'
    if str(env.get('WAYLAND_DISPLAY') or '').strip():
        return 'wayland'
    if str(env.get('DISPLAY') or '').strip():
        return 'x11'
    return ''


def is_wayland_session(environ: Mapping[str, str] | None = None) -> bool:
    """Return whether the environment identifies a Wayland desktop session."""
    return session_type(environ) == 'wayland'


def is_gnome_session(environ: Mapping[str, str] | None = None) -> bool:
    env = os.environ if environ is None else environ
    desktop = str(env.get('XDG_CURRENT_DESKTOP') or '').lower()
    return 'gnome' in desktop.split(':') or 'gnome' in desktop


def gnome_extension_install_dir(environ: Mapping[str, str] | None = None) -> Path:
    env = os.environ if environ is None else environ
    data_home = str(env.get('XDG_DATA_HOME') or '').strip()
    base = Path(data_home) if data_home else Path.home() / '.local' / 'share'
    return base / 'gnome-shell' / 'extensions' / GNOME_EXTENSION_UUID


def install_gnome_extension(
    *,
    environ: Mapping[str, str] | None = None,
    run: Callable[..., subprocess.CompletedProcess] = subprocess.run,
    log: LogFn | None = None,
) -> Path:
    """Copy the bundled extension into the user's GNOME Shell directory.

    The extension UUID is appended to the ``enabled-extensions`` GSettings key
    so it loads on the next login. GNOME Shell on Wayland only scans for new
    extensions at session start, so the user must log out and back in once.
    """
    say = log or (lambda _msg: None)
    target = gnome_extension_install_dir(environ)
    target.mkdir(parents=True, exist_ok=True)
    for name in ('metadata.json', 'extension.js'):
        shutil.copyfile(GNOME_EXTENSION_SOURCE / name, target / name)
    say(f'Installed GNOME Shell extension to {target}')
    if shutil.which('gsettings'):
        cp = run(
            ['gsettings', 'get', 'org.gnome.shell', 'enabled-extensions'],
            check=False,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
        )
        enabled: list[str] = []
        if cp.returncode == 0:
            enabled = _parse_gvariant_string_list(cp.stdout or '')
        if GNOME_EXTENSION_UUID not in enabled:
            enabled.append(GNOME_EXTENSION_UUID)
            value = '[' + ', '.join(_gvariant_quote(item) for item in enabled) + ']'
            run(
                ['gsettings', 'set', 'org.gnome.shell', 'enabled-extensions', value],
                check=False,
            )
        say(f'Enabled {GNOME_EXTENSION_UUID} in org.gnome.shell enabled-extensions')
    say('Log out and back in so GNOME Shell loads the extension.')
    return target


def _gvariant_quote(value: str) -> str:
    return "'" + value.replace('\\', '\\\\').replace("'", "\\'") + "'"


def _parse_gvariant_string_list(text: str) -> list[str]:
    text = text.strip()
    if text.startswith('@as'):
        text = text[3:].strip()
    try:
        parsed = ast.literal_eval(text) if text else []
    except (ValueError, SyntaxError):
        return []
    if isinstance(parsed, (list, tuple)):
        return [str(item) for item in parsed]
    return []


class X11WindowBackend:
    """Enumerate and place windows with ``wmctrl`` and ``xprop``."""

    name = 'wmctrl'

    def __init__(
        self,
        *,
        wmctrl_bin: str = 'wmctrl',
        xprop_bin: str = 'xprop',
        log_info: LogFn | None = None,
        log_warning: LogFn | None = None,
    ):
        self._wmctrl_bin = wmctrl_bin
        self._xprop_bin = xprop_bin
        self._log_info = log_info or (lambda _msg: None)
        self._log_warning = log_warning or (lambda _msg: None)
        self._wmctrl_available = bool(shutil.which(wmctrl_bin))
        self._xprop_available = bool(shutil.which(xprop_bin))
        # set by select_backend when wmctrl is only a partial fallback
        self.hint: str = ''

    @property
    def available(self) -> bool:
        return self._wmctrl_available

    def missing_tools(self) -> list[str]:
        missing = []
        if not self._wmctrl_available:
            missing.append(self._wmctrl_bin)
        if not self._xprop_available:
            missing.append(self._xprop_bin)
        return missing

    def list_windows(
        self,
        *,
        include_classes: bool = False,
        include_stack: bool = False,
    ) -> list[WindowInfo]:
        try:
            cp = subprocess.run(
                [self._wmctrl_bin, '-lpG'],
                check=False,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
            )
        except FileNotFoundError:
            self._wmctrl_available = False
            return []

        stdout = cp.stdout or ''
        stack_map = self._stacking_map() if include_stack else {}
        windows: list[WindowInfo] = []
        for idx, line in enumerate(stdout.splitlines()):
            parts = line.split(None, 8)
            if len(parts) < 9:
                continue
            wid_raw, desktop_raw, pid_raw, x_raw, y_raw, width_raw, height_raw, _host, title = parts
            try:
                desktop = int(desktop_raw)
            except ValueError:
                desktop = None
            try:
                pid = int(pid_raw)
            except ValueError:
                pid = None
            try:
                x = int(x_raw)
                y = int(y_raw)
                width = int(width_raw)
                height = int(height_raw)
            except ValueError:
                continue
            win = WindowInfo(
                wid=normalize_wid(wid_raw),
                title=title.strip(),
                desktop=desktop,
                pid=pid,
                x=x,
                y=y,
                width=width,
                height=height,
                wm_class=[],
                stack_index=None,
            )
            if include_classes:
                win.wm_class = self._read_wm_class(win.wid)
            if include_stack:
                win.stack_index = stack_map.get(win.wid, idx)
            windows.append(win)
        return windows

    def move_resize(self, wid: str, x: int, y: int, width: int, height: int) -> None:
        # drop maximized flags before resizing/repositioning so wmctrl can move the window
        self._run_wmctrl(['-i', '-r', wid, '-b', 'remove,maximized_vert,maximized_horz'])
        self._run_wmctrl(['-i', '-r', wid, '-e', f'0,{x},{y},{width},{height}'])

    def set_desktop(self, wid: str, desktop: int) -> None:
        self._run_wmctrl(['-i', '-r', wid, '-t', str(desktop)])

    def restack(self, wids: list[str]) -> None:
        for wid in wids:
            self._run_wmctrl(['-i', '-r', wid, '-b', 'remove,below,above'])
        for wid in wids:
            self._run_wmctrl(['-i', '-a', wid])

    def activate(self, wid: str) -> bool:
        self._run_wmctrl(['-i', '-a', wid])
        return self._wmctrl_available

    def clear_attention(self, wid: str) -> bool:
        """Remove the X11 urgency state that produces desktop banners."""
        self._run_wmctrl(
            ['-i', '-r', wid, '-b', 'remove,demands_attention']
        )
        return self._wmctrl_available

    def set_above(self, wid: str, above: bool = True) -> bool:
        action = 'add' if above else 'remove'
        self._run_wmctrl(['-i', '-r', wid, '-b', f'{action},above'])
        return self._wmctrl_available

    def _stacking_map(self) -> dict[str, int]:
        if not self._xprop_available:
            return {}
        try:
            cp = subprocess.run(
                [self._xprop_bin, '-root', '_NET_CLIENT_LIST_STACKING'],
                check=False,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
            )
        except FileNotFoundError:
            self._xprop_available = False
            return {}
        stdout = cp.stdout or ''
        order = [normalize_wid(match) for match in re.findall(r'0x[0-9a-fA-F]+', stdout)]
        return {wid: idx for idx, wid in enumerate(order)}

    def _read_wm_class(self, wid: str) -> list[str]:
        if not self._xprop_available:
            return []
        try:
            cp = subprocess.run(
                [self._xprop_bin, '-id', wid, 'WM_CLASS'],
                check=False,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
            )
        except FileNotFoundError:
            self._xprop_available = False
            return []
        if cp.returncode != 0:
            return []
        stdout = cp.stdout or ''
        if '=' in stdout:
            _, raw_value = stdout.split('=', 1)
        else:
            raw_value = stdout
        parts = []
        for piece in raw_value.split(','):
            cleaned = piece.strip().strip('"')
            if cleaned:
                parts.append(cleaned)
        return parts

    def _run_wmctrl(self, args: list[str]):
        try:
            cmd = [self._wmctrl_bin, *args]
            self._log_info(' '.join(cmd))
            subprocess.run(
                cmd,
                check=False,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
            )
        except FileNotFoundError:
            self._wmctrl_available = False


class GnomeWaylandWindowBackend:
    """Enumerate and place windows through the bundled GNOME Shell extension."""

    name = 'gnome-shell extension'

    def __init__(
        self,
        *,
        gdbus_bin: str = 'gdbus',
        log_info: LogFn | None = None,
        log_warning: LogFn | None = None,
        probe: bool = True,
    ):
        self._gdbus_bin = gdbus_bin
        self._log_info = log_info or (lambda _msg: None)
        self._log_warning = log_warning or (lambda _msg: None)
        self._gdbus_available = bool(shutil.which(gdbus_bin))
        self._extension_available = False
        self.hint: str = ''
        if probe and self._gdbus_available:
            self._extension_available = self._call('Version') is not None

    @property
    def available(self) -> bool:
        return self._gdbus_available and self._extension_available

    def missing_tools(self) -> list[str]:
        missing = []
        if not self._gdbus_available:
            missing.append(self._gdbus_bin)
        if not self._extension_available:
            missing.append(
                f'GNOME Shell extension {GNOME_EXTENSION_UUID} '
                f'(install with: {GNOME_EXTENSION_INSTALL_COMMAND}, then log out and in)'
            )
        return missing

    def list_windows(
        self,
        *,
        include_classes: bool = False,
        include_stack: bool = False,
    ) -> list[WindowInfo]:
        result = self._call('ListWindows')
        if not result:
            return []
        try:
            entries = json.loads(result[0])
        except (ValueError, TypeError, IndexError):
            self._log_warning('Unexpected window list from the GNOME Shell extension.')
            return []
        windows: list[WindowInfo] = []
        for entry in entries if isinstance(entries, list) else []:
            if not isinstance(entry, dict):
                continue
            try:
                win = WindowInfo(
                    wid=str(entry['id']),
                    title=str(entry.get('title') or '').strip(),
                    desktop=_opt_int(entry.get('desktop')),
                    pid=_opt_int(entry.get('pid')),
                    x=int(entry['x']),
                    y=int(entry['y']),
                    width=int(entry['width']),
                    height=int(entry['height']),
                    wm_class=[str(c) for c in entry.get('wm_class') or [] if c]
                    if include_classes
                    else [],
                    stack_index=_opt_int(entry.get('stack_index')) if include_stack else None,
                )
            except (KeyError, TypeError, ValueError):
                continue
            windows.append(win)
        return windows

    def move_resize(self, wid: str, x: int, y: int, width: int, height: int) -> None:
        self._call('MoveResize', str(wid), int(x), int(y), int(width), int(height))

    def set_desktop(self, wid: str, desktop: int) -> None:
        self._call('SetWorkspace', str(wid), int(desktop))

    def restack(self, wids: list[str]) -> None:
        for wid in wids:
            self._call('Activate', str(wid))

    def activate(self, wid: str) -> bool:
        result = self._call('Activate', str(wid))
        return bool(result and result[0])

    def clear_attention(self, wid: str) -> bool:
        """Remove Mutter's attention state from a managed window."""
        result = self._call('ClearAttention', str(wid))
        return bool(result and result[0])

    def set_above(self, wid: str, above: bool = True) -> bool:
        result = self._call('SetAbove', str(wid), bool(above))
        return bool(result and result[0])

    def _call(self, method: str, *args) -> tuple | None:
        cmd = [
            self._gdbus_bin,
            'call',
            '--session',
            '--dest',
            GNOME_EXTENSION_BUS_NAME,
            '--object-path',
            GNOME_EXTENSION_OBJECT_PATH,
            '--method',
            f'{GNOME_EXTENSION_INTERFACE}.{method}',
            *[_gvariant_arg(arg) for arg in args],
        ]
        if method != 'Version':
            self._log_info(' '.join(cmd))
        try:
            cp = subprocess.run(
                cmd,
                check=False,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
                timeout=10,
            )
        except (FileNotFoundError, subprocess.TimeoutExpired):
            self._gdbus_available = bool(shutil.which(self._gdbus_bin))
            return None
        if cp.returncode != 0:
            if method != 'Version':
                self._log_warning(
                    f'GNOME Shell extension call {method} failed: {(cp.stderr or "").strip()}'
                )
            return None
        return parse_gvariant_tuple(cp.stdout or '')


def find_own_window(backend, title: str, pid: int | None = None) -> WindowInfo | None:
    """Locate a window of this process by exact title (used for pinning)."""
    own_pid = os.getpid() if pid is None else pid
    wanted = str(title or '').strip()
    if not wanted:
        return None
    candidates = [
        win
        for win in backend.list_windows()
        if win.title.strip() == wanted and (win.pid is None or win.pid == own_pid)
    ]
    if not candidates:
        return None
    # prefer an exact pid match when several windows share the title
    for win in candidates:
        if win.pid == own_pid:
            return win
    return candidates[0]


def _gvariant_arg(value) -> str:
    if isinstance(value, bool):
        return 'true' if value else 'false'
    if isinstance(value, int):
        return str(value)
    return _gvariant_quote(str(value))


_GVARIANT_STRING_RE = re.compile(r"'(?:\\.|[^'\\])*'|\"(?:\\.|[^\"\\])*\"")


def _pythonize_gvariant_fragment(fragment: str) -> str:
    fragment = re.sub(r'\btrue\b', 'True', fragment)
    fragment = re.sub(r'\bfalse\b', 'False', fragment)
    # strip GVariant type annotations like @as or uint64 prefixes
    fragment = re.sub(r'@[a-z{}()]+\s', '', fragment)
    fragment = re.sub(
        r'\b(?:u?int(?:16|32|64)|byte|double|objectpath|signature)\s', '', fragment
    )
    return fragment


def parse_gvariant_tuple(text: str) -> tuple | None:
    """Parse ``gdbus call`` output such as ``('[...]',)`` or ``(true,)``.

    Substitutions for GVariant literals are applied only outside quoted
    strings so that JSON payloads carrying ``true``/``false`` stay intact.
    """
    text = text.strip()
    if not text:
        return None
    pieces: list[str] = []
    pos = 0
    for match in _GVARIANT_STRING_RE.finditer(text):
        pieces.append(_pythonize_gvariant_fragment(text[pos:match.start()]))
        pieces.append(match.group(0))
        pos = match.end()
    pieces.append(_pythonize_gvariant_fragment(text[pos:]))
    try:
        parsed = ast.literal_eval(''.join(pieces))
    except (ValueError, SyntaxError):
        return None
    if not isinstance(parsed, tuple):
        parsed = (parsed,)
    return parsed


def _opt_int(value) -> int | None:
    try:
        return int(value) if value is not None else None
    except (TypeError, ValueError):
        return None


def normalize_wid(raw: str) -> str:
    value = str(raw).strip().lower()
    if value.startswith('0x'):
        return value
    try:
        return hex(int(value))
    except ValueError:
        return value


def select_backend(
    *,
    wmctrl_bin: str = 'wmctrl',
    xprop_bin: str = 'xprop',
    gdbus_bin: str = 'gdbus',
    environ: Mapping[str, str] | None = None,
    log_info: LogFn | None = None,
    log_warning: LogFn | None = None,
):
    """Pick the window backend that matches the running desktop session.

    On a Wayland session the GNOME Shell extension is preferred because it
    sees native Wayland windows; wmctrl remains the fallback for XWayland
    windows when the extension is not installed. On X11 wmctrl is used.
    """
    x11 = X11WindowBackend(
        wmctrl_bin=wmctrl_bin,
        xprop_bin=xprop_bin,
        log_info=log_info,
        log_warning=log_warning,
    )
    if session_type(environ) != 'wayland':
        return x11
    gnome = GnomeWaylandWindowBackend(
        gdbus_bin=gdbus_bin,
        log_info=log_info,
        log_warning=log_warning,
    )
    if gnome.available:
        return gnome
    if x11.available:
        x11.hint = (
            'Wayland session without the GNOME Shell window extension: '
            'wmctrl only sees XWayland windows. Install the extension with '
            f'"{GNOME_EXTENSION_INSTALL_COMMAND}" and log in again.'
        )
        return x11
    return gnome


__all__ = [
    'GNOME_EXTENSION_INSTALL_COMMAND',
    'GNOME_EXTENSION_UUID',
    'GnomeWaylandWindowBackend',
    'WindowInfo',
    'X11WindowBackend',
    'find_own_window',
    'gnome_extension_install_dir',
    'install_gnome_extension',
    'is_gnome_session',
    'normalize_wid',
    'parse_gvariant_tuple',
    'select_backend',
    'session_type',
]
