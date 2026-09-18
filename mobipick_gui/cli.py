"""Command-line entry points for the Mobipick Labs GUI."""
from __future__ import annotations

import argparse
import os
import signal
import sys
from typing import Sequence

from PyQt5.QtCore import QCoreApplication, qInstallMessageHandler
from PyQt5.QtGui import QIcon
from PyQt5.QtWidgets import QApplication

from . import MainWindow, trigger_sigint
from .desktop_launcher import (
    APPLICATION_DESKTOP_ID as _APPLICATION_DESKTOP_ID,
    APPLICATION_ICON as _APPLICATION_ICON,
    install_desktop_launcher,
    install_tool_desktop_entries as _install_tool_desktop_entries,
    install_user_desktop_entry as _install_user_desktop_entry,
)
from .remote_client import refresh_installed_skill
from .window_control import install_gnome_extension, session_type


_QT_SOCKET_NOTIFIER_THREAD_WARNING = (
    'QSocketNotifier: Can only be used with threads started with QThread'
)
_QT_WAYLAND_ACTIVATION_WARNING = (
    'Wayland does not support QWindow::requestActivate()'
)


def _create_application(
    arguments: list[str],
    desktop_session: str | None = None,
) -> QApplication:
    """Create the application while hiding known benign Qt warnings.

    Some Qt 5 Wayland installations emit the socket-notifier thread warning
    from ``QApplication`` construction itself, even for a minimal application
    with no worker threads.  Limit that filter to construction.  On Wayland,
    keep filtering Qt's unsupported activation warning because Qt widgets can
    request activation internally when they are shown.  Preserve all other Qt
    messages so genuine application warnings stay visible.
    """
    previous_handler = None
    current_session = desktop_session or session_type()
    wayland_session = current_session == 'wayland'
    QCoreApplication.setApplicationName(_APPLICATION_DESKTOP_ID)

    def startup_message_handler(message_type, context, message):
        if message == _QT_SOCKET_NOTIFIER_THREAD_WARNING:
            return
        if wayland_session and message == _QT_WAYLAND_ACTIVATION_WARNING:
            return
        if previous_handler is not None:
            previous_handler(message_type, context, message)
        else:
            print(message, file=sys.stderr, flush=True)

    previous_handler = qInstallMessageHandler(startup_message_handler)
    application = None
    try:
        application = QApplication(arguments)
        if current_session == 'wayland':
            application.setDesktopFileName(_APPLICATION_DESKTOP_ID)
        application.setWindowIcon(QIcon(str(_APPLICATION_ICON)))
        platform_name = getattr(application, 'platformName', None)
        if callable(platform_name):
            wayland_session = str(platform_name()).lower().startswith('wayland')
        return application
    finally:
        if application is None or not wayland_session:
            qInstallMessageHandler(previous_handler)


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description='Mobipick Labs Control GUI')
    parser.add_argument(
        '-v',
        '--v',
        '--verbose',
        dest='verbosity',
        nargs='?',
        const=3,
        default=1,
        type=int,
        choices=[1, 2, 3],
        help='Verbosity level (1=min, 3=max). If no value provided defaults to 3.',
    )
    parser.add_argument(
        '--install-desktop-launcher',
        action='store_true',
        help=(
            'Install the per-user application launcher, add it to the '
            'GNOME/Ubuntu dock, then exit.'
        ),
    )
    parser.add_argument(
        '--install-gnome-window-extension',
        dest='install_gnome_window_extension',
        action='store_true',
        help=(
            'Install and enable the bundled GNOME Shell extension used for '
            'window layout capture and replay on Wayland sessions, then exit. '
            'Log out and back in afterwards.'
        ),
    )
    remote = parser.add_argument_group(
        'remote control',
        'Expose a JSON HTTP API so other machines (or automation agents) can '
        'press buttons, wait for launch events, read logs, and run commands '
        'in container shells. Environment overrides: '
        'MOBIPICK_GUI_REMOTE_CONTROL=1, MOBIPICK_GUI_REMOTE_HOST, '
        'MOBIPICK_GUI_REMOTE_PORT, MOBIPICK_GUI_REMOTE_TOKEN.',
    )
    remote.add_argument(
        '--remote-control',
        dest='remote_control',
        action='store_true',
        default=None,
        help='Start the remote control API on launch.',
    )
    remote.add_argument(
        '--no-remote-control',
        dest='remote_control',
        action='store_false',
        help='Do not start the remote control API even if the config enables it.',
    )
    remote.add_argument(
        '--remote-host',
        dest='remote_host',
        default=None,
        help='Bind address for the remote control API (default from config: 0.0.0.0).',
    )
    remote.add_argument(
        '--remote-port',
        dest='remote_port',
        type=int,
        default=None,
        help='TCP port for the remote control API (default from config: 8765).',
    )
    remote.add_argument(
        '--remote-token',
        dest='remote_token',
        default=None,
        help='Shared secret required in "Authorization: Bearer <token>" headers.',
    )
    return parser


def _env_flag(name: str) -> bool | None:
    raw = os.environ.get(name)
    if raw is None or raw.strip() == '':
        return None
    return raw.strip().lower() not in {'0', 'false', 'no', 'off'}


def remote_control_overrides(parsed_args: argparse.Namespace) -> dict:
    """Combine environment and CLI remote-control settings (CLI wins).

    Remote control is opt-in for every launch.  A stale user configuration
    must not expose the command API when neither the command-line flag nor
    the environment variable enables it.
    """
    overrides: dict = {}
    env_enabled = _env_flag('MOBIPICK_GUI_REMOTE_CONTROL')
    if env_enabled is not None:
        overrides['enabled'] = env_enabled
        overrides['_enabled_source'] = 'MOBIPICK_GUI_REMOTE_CONTROL'
    env_host = os.environ.get('MOBIPICK_GUI_REMOTE_HOST')
    if env_host:
        overrides['host'] = env_host
    env_port = os.environ.get('MOBIPICK_GUI_REMOTE_PORT')
    if env_port:
        try:
            overrides['port'] = int(env_port)
        except ValueError:
            pass
    env_token = os.environ.get('MOBIPICK_GUI_REMOTE_TOKEN')
    if env_token is not None and env_token != '':
        overrides['token'] = env_token
    if parsed_args.remote_control is not None:
        overrides['enabled'] = bool(parsed_args.remote_control)
        overrides['_enabled_source'] = (
            '--remote-control'
            if parsed_args.remote_control
            else '--no-remote-control'
        )
    if parsed_args.remote_host:
        overrides['host'] = parsed_args.remote_host
    if parsed_args.remote_port is not None:
        overrides['port'] = int(parsed_args.remote_port)
    if parsed_args.remote_token is not None:
        overrides['token'] = parsed_args.remote_token
    if (
        'enabled' not in overrides
        and any(key in overrides for key in ('host', 'port', 'token'))
        and parsed_args.remote_control is None
    ):
        # Giving a host/port/token on the command line implies enabling.
        if any(
            getattr(parsed_args, name) is not None
            for name in ('remote_host', 'remote_port', 'remote_token')
        ):
            overrides['enabled'] = True
            overrides['_enabled_source'] = 'remote-control CLI option'
    if 'enabled' not in overrides:
        overrides['enabled'] = False
    return overrides


def main(argv: Sequence[str] | None = None) -> int:
    """Run the Qt application."""

    desktop_session = session_type()

    if argv is None:
        argv = sys.argv[1:]

    parser = _build_parser()
    parsed_args, qt_args = parser.parse_known_args(list(argv))
    verbosity = parsed_args.verbosity or 1

    if parsed_args.install_desktop_launcher:
        try:
            desktop_file, pinned = install_desktop_launcher()
        except OSError as exc:
            print(
                f'Failed to install the desktop launcher: {exc}',
                file=sys.stderr,
            )
            return 1
        print(f'Installed desktop launcher: {desktop_file}')
        if pinned:
            print('Added Mobipick Labs Control to the GNOME/Ubuntu dock.')
        else:
            print('Mobipick Labs Control is already pinned to the dock.')
        return 0

    if parsed_args.install_gnome_window_extension:
        try:
            install_gnome_extension(log=print)
        except OSError as exc:
            print(f'Failed to install the GNOME Shell extension: {exc}', file=sys.stderr)
            return 1
        return 0

    if desktop_session in {'x11', 'wayland'}:
        try:
            _install_user_desktop_entry()
            _install_tool_desktop_entries()
        except OSError as exc:
            print(
                f'Failed to install desktop application metadata: {exc}',
                file=sys.stderr,
            )
    refreshed_skill = refresh_installed_skill()
    if refreshed_skill is not None and verbosity >= 1:
        print(f'updated installed remote-control skill: {refreshed_skill}')

    app = _create_application(
        [sys.argv[0]] + qt_args,
        desktop_session=desktop_session,
    )
    window = MainWindow(
        verbosity=verbosity,
        remote_control=remote_control_overrides(parsed_args),
    )
    window.setWindowIcon(app.windowIcon())
    window.show_with_restored_state()

    def _handle_sigint(_sig, _frame):
        trigger_sigint()

    signal.signal(signal.SIGINT, _handle_sigint)

    return app.exec_()


__all__ = ['main', 'remote_control_overrides']
