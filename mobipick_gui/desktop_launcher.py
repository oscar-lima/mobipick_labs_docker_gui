"""Install the per-user desktop launcher and pin it on GNOME desktops."""
from __future__ import annotations

import ast
import os
import shutil
import subprocess
import sys
from pathlib import Path
from typing import Mapping, Sequence

from .config import PROJECT_ROOT


APPLICATION_DESKTOP_ID = 'mobipick-labs-docker-gui'
APPLICATION_DESKTOP_FILE = f'{APPLICATION_DESKTOP_ID}.desktop'
APPLICATION_ICON = PROJECT_ROOT / 'images' / 'mobipick_icon.png'
GNOME_FAVORITES_SCHEMA = 'org.gnome.shell'
GNOME_FAVORITES_KEY = 'favorite-apps'


def _desktop_exec_argument(value: str) -> str:
    """Quote one argument for a freedesktop desktop entry Exec field."""
    escaped = value.replace('\\', '\\\\')
    for character in ('"', '`', '$'):
        escaped = escaped.replace(character, f'\\{character}')
    return f'"{escaped}"'


def desktop_launch_command(argv0: str | None = None) -> str:
    """Return a launcher command matching the current GUI invocation."""
    raw_launcher = argv0 if argv0 is not None else sys.argv[0]
    launcher = Path(raw_launcher).expanduser()
    if launcher.suffix == '.py':
        arguments = (sys.executable, str(launcher.resolve()))
    else:
        arguments = (str(launcher.resolve()),)
    return ' '.join(_desktop_exec_argument(value) for value in arguments)


def install_user_desktop_entry(
    *,
    environ: Mapping[str, str] | None = None,
    argv0: str | None = None,
) -> Path:
    """Install desktop metadata used to associate the window and its icon."""
    env = os.environ if environ is None else environ
    data_home = str(env.get('XDG_DATA_HOME') or '').strip()
    base = (
        Path(data_home).expanduser()
        if data_home
        else Path.home() / '.local' / 'share'
    )
    target = base / 'applications' / APPLICATION_DESKTOP_FILE
    content = (
        '[Desktop Entry]\n'
        'Type=Application\n'
        'Name=Mobipick Labs Control\n'
        'Comment=Control the Mobipick Labs Docker simulation\n'
        f'Exec={desktop_launch_command(argv0)}\n'
        f'Icon={APPLICATION_ICON.resolve()}\n'
        'Terminal=false\n'
        'Categories=Development;Robotics;\n'
        f'StartupWMClass={APPLICATION_DESKTOP_ID}\n'
    )
    target.parent.mkdir(parents=True, exist_ok=True)
    if not target.exists() or target.read_text(encoding='utf-8') != content:
        target.write_text(content, encoding='utf-8')
    return target


def _parse_gsettings_string_array(value: str) -> list[str]:
    """Parse the string-array representation printed by ``gsettings get``."""
    text = value.strip()
    if text == '@as []':
        return []
    try:
        parsed = ast.literal_eval(text)
    except (SyntaxError, ValueError) as exc:
        raise OSError(f'could not parse GNOME dock favorites: {text}') from exc
    if not isinstance(parsed, list) or not all(
        isinstance(item, str) for item in parsed
    ):
        raise OSError(f'unexpected GNOME dock favorites value: {text}')
    return parsed


def _run_gsettings(arguments: Sequence[str]) -> subprocess.CompletedProcess[str]:
    """Run gsettings and turn command failures into concise setup errors."""
    if shutil.which('gsettings') is None:
        raise OSError('gsettings is not installed; cannot update the GNOME dock')
    result = subprocess.run(
        ['gsettings', *arguments],
        check=False,
        capture_output=True,
        text=True,
    )
    if result.returncode != 0:
        detail = result.stderr.strip() or result.stdout.strip()
        raise OSError(detail or 'gsettings failed')
    return result


def pin_launcher_to_gnome_dock() -> bool:
    """Append the launcher to GNOME favorites; return whether it changed."""
    current = _run_gsettings([
        'get',
        GNOME_FAVORITES_SCHEMA,
        GNOME_FAVORITES_KEY,
    ])
    favorites = _parse_gsettings_string_array(current.stdout)
    if APPLICATION_DESKTOP_FILE in favorites:
        return False
    favorites.append(APPLICATION_DESKTOP_FILE)
    _run_gsettings([
        'set',
        GNOME_FAVORITES_SCHEMA,
        GNOME_FAVORITES_KEY,
        repr(favorites),
    ])
    return True


def install_desktop_launcher(
    *,
    environ: Mapping[str, str] | None = None,
    argv0: str | None = None,
) -> tuple[Path, bool]:
    """Install the desktop entry and pin it to the GNOME/Ubuntu dock."""
    desktop_file = install_user_desktop_entry(
        environ=environ,
        argv0=argv0,
    )
    return desktop_file, pin_launcher_to_gnome_dock()


__all__ = [
    'APPLICATION_DESKTOP_FILE',
    'APPLICATION_DESKTOP_ID',
    'APPLICATION_ICON',
    'desktop_launch_command',
    'install_desktop_launcher',
    'install_user_desktop_entry',
    'pin_launcher_to_gnome_dock',
]
