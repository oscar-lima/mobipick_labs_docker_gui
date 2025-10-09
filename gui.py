"""Entry point for the Mobipick Labs Control GUI."""
from __future__ import annotations

import argparse
import signal
import sys

from PyQt5.QtCore import Qt
from PyQt5.QtWidgets import QApplication

from mobipick_gui import (
    MainWindow,
    WebBridge,
    WebController,
    WebUiServer,
    find_free_port,
    trigger_sigint,
)


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
    parser.add_argument(
        '--web',
        action='store_true',
        help='Serve the GUI over HTTP instead of showing the native Qt window.',
    )
    parser.add_argument(
        '--web-port',
        type=int,
        default=0,
        help='Port for the web UI (0 selects an available port automatically).',
    )
    parser.add_argument(
        '--web-host',
        type=str,
        default='127.0.0.1',
        help='Host interface for the web UI.',
    )

    parsed_args, qt_args = parser.parse_known_args()
    verbosity = parsed_args.verbosity or 1

    app = QApplication([sys.argv[0]] + qt_args)
    bridge: WebBridge | None = None
    server: WebUiServer | None = None

    if parsed_args.web:
        bridge = WebBridge()
        window = MainWindow(verbosity=verbosity, web_bridge=bridge)
        window.setAttribute(Qt.WA_DontShowOnScreen, True)
        controller = WebController(window, bridge)
        host = parsed_args.web_host or '127.0.0.1'
        port = parsed_args.web_port if parsed_args.web_port and parsed_args.web_port > 0 else find_free_port()
        server = WebUiServer(bridge, controller, host=host, port=port)
        server.start()
        address = server.address
        if address:
            host, port = address
            print(f'Web UI available at http://{host}:{port}', flush=True)
        else:
            print('Failed to start web server', file=sys.stderr)
    else:
        window = MainWindow(verbosity=verbosity)
        window.show()

    def _handle_sigint(_sig, _frame):
        trigger_sigint()

    signal.signal(signal.SIGINT, _handle_sigint)
    try:
        return app.exec_()
    finally:
        if bridge is not None:
            bridge.shutdown()
        if server is not None:
            server.stop()


if __name__ == '__main__':
    sys.exit(main())
