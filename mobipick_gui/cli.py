"""Command-line entry points for the Mobipick Labs GUI."""
from __future__ import annotations

import argparse
import signal
import sys
from typing import Sequence

from PyQt5.QtWidgets import QApplication

from . import MainWindow, trigger_sigint


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
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    """Run the Qt application."""

    if argv is None:
        argv = sys.argv[1:]

    parser = _build_parser()
    parsed_args, qt_args = parser.parse_known_args(list(argv))
    verbosity = parsed_args.verbosity or 1

    app = QApplication([sys.argv[0]] + qt_args)
    window = MainWindow(verbosity=verbosity)
    window.show()

    def _handle_sigint(_sig, _frame):
        trigger_sigint()

    signal.signal(signal.SIGINT, _handle_sigint)

    return app.exec_()


__all__ = ['main']
