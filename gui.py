"""Entry point for the Mobipick Labs Control GUI."""
from __future__ import annotations

import argparse
import signal
import sys

from PyQt5.QtWidgets import QApplication

from mobipick_gui import MainWindow, trigger_sigint


def main() -> int:
    parser = argparse.ArgumentParser(description='Mobipick Labs Control GUI')
    parser.add_argument(
        '-v', '--v', '--verbose',
        dest='verbosity',
        nargs='?',
        const=3,
        default=1,
        type=int,
        choices=[1, 2, 3],
        help='Verbosity level (1=min, 3=max). If no value provided defaults to 3.'
    )

    parsed_args, qt_args = parser.parse_known_args()
    verbosity = parsed_args.verbosity or 1

    app = QApplication([sys.argv[0]] + qt_args)
    window = MainWindow(verbosity=verbosity)
    window.show()

    def _handle_sigint(_sig, _frame):
        trigger_sigint()

    signal.signal(signal.SIGINT, _handle_sigint)

    return app.exec_()


if __name__ == '__main__':
    sys.exit(main())
