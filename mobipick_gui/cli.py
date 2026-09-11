"""Command-line entry points for the Mobipick Labs GUI."""
from __future__ import annotations

import argparse
import os
import signal
import sys
from typing import Sequence

from PyQt5.QtWidgets import QApplication

from . import MainWindow, trigger_sigint
from .window_control import install_gnome_extension


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

    if argv is None:
        argv = sys.argv[1:]

    parser = _build_parser()
    parsed_args, qt_args = parser.parse_known_args(list(argv))
    verbosity = parsed_args.verbosity or 1

    if parsed_args.install_gnome_window_extension:
        try:
            install_gnome_extension(log=print)
        except OSError as exc:
            print(f'Failed to install the GNOME Shell extension: {exc}', file=sys.stderr)
            return 1
        return 0

    app = QApplication([sys.argv[0]] + qt_args)
    window = MainWindow(
        verbosity=verbosity,
        remote_control=remote_control_overrides(parsed_args),
    )
    window.show()

    def _handle_sigint(_sig, _frame):
        trigger_sigint()

    signal.signal(signal.SIGINT, _handle_sigint)

    return app.exec_()


__all__ = ['main', 'remote_control_overrides']
