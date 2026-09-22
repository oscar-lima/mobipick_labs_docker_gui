"""Install the per-user desktop launcher and pin it on GNOME desktops.

The module also installs hidden desktop entries for the ROS tools that the
GUI runs inside containers.  GNOME Shell 45 and newer no longer read the
``_NET_WM_ICON`` a window sets itself; the dock, Alt-Tab switcher, and
overview only show an icon when the window's X11 ``WM_CLASS`` or Wayland
``app_id`` matches an installed desktop entry.  Containers cannot install
entries on the host, so the GUI provides them for RViz, RQt, and Gazebo.
The matching additionally requires the window not to look remote; see
``display_runtime.container_hostname_environment``.
"""
from __future__ import annotations

import ast
import os
import re
import shutil
import subprocess
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Mapping, Sequence

from .config import PROJECT_ROOT


APPLICATION_DESKTOP_ID = 'mobipick-labs-docker-gui'
APPLICATION_DESKTOP_FILE = f'{APPLICATION_DESKTOP_ID}.desktop'
APPLICATION_ICON = PROJECT_ROOT / 'images' / 'mobipick_icon.png'
GNOME_FAVORITES_SCHEMA = 'org.gnome.shell'
GNOME_FAVORITES_KEY = 'favorite-apps'

RVIZ_DESKTOP_ID = 'mobipick-rviz'
RQT_DESKTOP_ID = 'mobipick-rqt'
GAZEBO_DESKTOP_ID = 'mobipick-gazebo'


@dataclass(frozen=True)
class ToolDesktopEntry:
    """Hidden desktop entry that lends its icon to container tool windows."""

    desktop_id: str
    name: str
    comment: str
    icon: Path
    startup_wm_class: str


# ``startup_wm_class`` is the identity a tool reports on its own: Qt on X11
# and XWayland uses the executable name as WM_CLASS, and Qt 5.12 on native
# Wayland derives the app_id from the interpreter, so every rqt plugin
# shows up as ``python3`` there.  GUI launches additionally set
# ``RESOURCE_NAME`` (see ``tool_window_environment``) so that X11 windows
# match the entry by its desktop-file ID regardless of the script name.
# GNOME resolves a ``StartupWMClass`` hit on either WM_CLASS part before it
# falls back to the desktop-file ID, so a launch that sets the RQt
# ``RESOURCE_NAME`` still shows Gazebo and RViz windows with their own icons
# and only lends the RQt identity to windows nobody else claims.
TOOL_DESKTOP_ENTRIES: tuple[ToolDesktopEntry, ...] = (
    ToolDesktopEntry(
        desktop_id=RVIZ_DESKTOP_ID,
        name='RViz',
        comment='ROS visualization launched from Mobipick Labs Control',
        icon=PROJECT_ROOT / 'images' / 'rviz_icon.png',
        startup_wm_class='rviz',
    ),
    ToolDesktopEntry(
        desktop_id=RQT_DESKTOP_ID,
        name='RQt',
        comment='ROS Qt tools launched from Mobipick Labs Control',
        icon=PROJECT_ROOT / 'images' / 'rqt_icon.png',
        startup_wm_class='python3',
    ),
    ToolDesktopEntry(
        desktop_id=GAZEBO_DESKTOP_ID,
        name='Gazebo',
        comment='Gazebo simulation launched from Mobipick Labs Control',
        icon=PROJECT_ROOT / 'images' / 'gazebo_icon.svg',
        # gzclient names its QApplication "gazebo"; both WM_CLASS parts use it.
        startup_wm_class='gazebo',
    ),
)


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
    if launcher.name == '__main__.py':
        # ``python -m mobipick_gui`` leaves the package's ``__main__.py`` in
        # argv[0].  Running that file as a script fails with "attempted
        # relative import with no known parent package", so record the shim
        # beside the package, or the module itself, instead.
        package = launcher.resolve().parent
        shim = package.parent / 'gui.py'
        if shim.is_file():
            arguments = (sys.executable, str(shim))
        else:
            arguments = (sys.executable, '-m', package.name)
    elif launcher.suffix == '.py':
        arguments = (sys.executable, str(launcher.resolve()))
    else:
        arguments = (str(launcher.resolve()),)
    return ' '.join(_desktop_exec_argument(value) for value in arguments)


def _applications_dir(environ: Mapping[str, str] | None = None) -> Path:
    """Return the per-user directory holding desktop entries."""
    env = os.environ if environ is None else environ
    data_home = str(env.get('XDG_DATA_HOME') or '').strip()
    base = (
        Path(data_home).expanduser()
        if data_home
        else Path.home() / '.local' / 'share'
    )
    return base / 'applications'


def _write_desktop_entry(target: Path, content: str) -> Path:
    """Write ``content`` to ``target`` unless it is already identical."""
    target.parent.mkdir(parents=True, exist_ok=True)
    if not target.exists() or target.read_text(encoding='utf-8') != content:
        target.write_text(content, encoding='utf-8')
    return target


def install_user_desktop_entry(
    *,
    environ: Mapping[str, str] | None = None,
    argv0: str | None = None,
) -> Path:
    """Install desktop metadata used to associate the window and its icon."""
    target = _applications_dir(environ) / APPLICATION_DESKTOP_FILE
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
    return _write_desktop_entry(target, content)


def install_tool_desktop_entries(
    *,
    environ: Mapping[str, str] | None = None,
) -> list[Path]:
    """Install hidden desktop entries that give container tool windows icons.

    The entries are metadata only: ``NoDisplay`` keeps them out of the
    application grid and ``Exec`` is inert because the tools can only run
    from the GUI's containers.  ``SingleMainWindow`` stops GNOME from
    offering a "New Window" action for them.
    """
    applications = _applications_dir(environ)
    installed = []
    for entry in TOOL_DESKTOP_ENTRIES:
        content = (
            '[Desktop Entry]\n'
            'Type=Application\n'
            f'Name={entry.name}\n'
            f'Comment={entry.comment}\n'
            'Exec=true\n'
            f'Icon={entry.icon.resolve()}\n'
            'Terminal=false\n'
            'NoDisplay=true\n'
            'SingleMainWindow=true\n'
            f'StartupWMClass={entry.startup_wm_class}\n'
        )
        target = applications / f'{entry.desktop_id}.desktop'
        installed.append(_write_desktop_entry(target, content))
    return installed


_RQT_COMMAND_PATTERN = re.compile(r'(?<![\w-])rqt(?=[_\W]|$)')


def desktop_entry_for_command(command: str) -> str | None:
    """Return the desktop entry ID whose icon a container command should use.

    Commands that run an rqt tool (``rqt``, ``rqt_gui --standalone ...``,
    ``rosrun rqt_graph rqt_graph``, launch files from ``rqt_*`` packages)
    create windows named after the plugin script, which no static
    ``StartupWMClass`` can cover, so they borrow the RQt identity.  RViz
    windows always report ``rviz`` themselves and need no override.
    """
    if _RQT_COMMAND_PATTERN.search(command or ''):
        return RQT_DESKTOP_ID
    return None


def tool_window_environment(desktop_id: str) -> dict[str, str]:
    """Return container environment tying Qt X11 windows to a desktop entry.

    Qt's XCB platform uses ``RESOURCE_NAME`` as the WM_CLASS instance name,
    which GNOME resolves to ``<instance>.desktop``.  Native Wayland windows
    ignore it and are matched through the entry's ``StartupWMClass``.
    """
    return {'RESOURCE_NAME': desktop_id}


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
    install_tool_desktop_entries(environ=environ)
    return desktop_file, pin_launcher_to_gnome_dock()


__all__ = [
    'APPLICATION_DESKTOP_FILE',
    'APPLICATION_DESKTOP_ID',
    'APPLICATION_ICON',
    'GAZEBO_DESKTOP_ID',
    'RQT_DESKTOP_ID',
    'RVIZ_DESKTOP_ID',
    'TOOL_DESKTOP_ENTRIES',
    'ToolDesktopEntry',
    'desktop_entry_for_command',
    'desktop_launch_command',
    'install_desktop_launcher',
    'install_tool_desktop_entries',
    'install_user_desktop_entry',
    'pin_launcher_to_gnome_dock',
    'tool_window_environment',
]
