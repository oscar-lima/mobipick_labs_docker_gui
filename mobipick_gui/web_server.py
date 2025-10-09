"""Minimal HTTP server exposing the GUI through a browser-friendly interface."""
from __future__ import annotations

import json
import socket
import threading
import time
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, HTTPServer
from typing import Any, Callable
from urllib.parse import parse_qs, urlparse

from .web_assets import INDEX_HTML
from .web_bridge import WebBridge
from .web_controller import WebController


class _ThreadingHTTPServer(HTTPServer):
    """HTTP server that tracks connections and exposes bridge/controller."""

    daemon_threads = True

    def __init__(self, server_address, RequestHandlerClass, *, bridge: WebBridge, controller: WebController):
        super().__init__(server_address, RequestHandlerClass)
        self.bridge = bridge
        self.controller = controller


class _RequestHandler(BaseHTTPRequestHandler):
    protocol_version = 'HTTP/1.1'

    def log_message(self, format: str, *args: Any) -> None:  # pragma: no cover - reduce noise
        return

    def _write_response(self, status: HTTPStatus, body: bytes, content_type: str = 'application/json') -> None:
        self.send_response(status)
        self.send_header('Content-Type', content_type)
        self.send_header('Content-Length', str(len(body)))
        self.send_header('Cache-Control', 'no-store')
        self.end_headers()
        if body:
            self.wfile.write(body)

    def _json(self, obj: Any, status: HTTPStatus = HTTPStatus.OK) -> None:
        body = json.dumps(obj).encode('utf-8')
        self._write_response(status, body, 'application/json')

    def _bad_request(self, message: str) -> None:
        self._json({'error': message}, HTTPStatus.BAD_REQUEST)

    @property
    def bridge(self) -> WebBridge:
        return getattr(self.server, 'bridge')  # type: ignore[no-any-return]

    @property
    def controller(self) -> WebController:
        return getattr(self.server, 'controller')  # type: ignore[no-any-return]

    def do_GET(self) -> None:  # noqa: N802 - required signature
        parsed = urlparse(self.path)
        if parsed.path in {'/', '/index.html'}:
            self._write_response(HTTPStatus.OK, INDEX_HTML.encode('utf-8'), 'text/html; charset=utf-8')
            return
        if parsed.path == '/api/snapshot':
            snapshot = self.bridge.snapshot()
            self._json(snapshot)
            return
        if parsed.path == '/api/events':
            query = parse_qs(parsed.query)
            since = int(query.get('since', ['0'])[0] or 0)
            events = self.bridge.events_since(since)
            self._json(events)
            return
        if parsed.path == '/api/health':
            self._json({'ok': True, 'ts': time.time()})
            return
        self._write_response(HTTPStatus.NOT_FOUND, b'Not Found', 'text/plain; charset=utf-8')

    def do_POST(self) -> None:  # noqa: N802 - required signature
        parsed = urlparse(self.path)
        if parsed.path != '/api/action':
            self._write_response(HTTPStatus.NOT_FOUND, b'Not Found', 'text/plain; charset=utf-8')
            return
        try:
            length = int(self.headers.get('Content-Length', '0'))
        except ValueError:
            self._bad_request('Invalid Content-Length header')
            return
        raw = self.rfile.read(length) if length > 0 else b''
        try:
            data = json.loads(raw.decode('utf-8') or '{}')
        except json.JSONDecodeError:
            self._bad_request('Invalid JSON payload')
            return
        action = data.get('action')
        payload = data.get('payload') or {}
        if not isinstance(payload, dict):
            self._bad_request('Payload must be an object')
            return
        if not isinstance(action, str) or not action:
            self._bad_request('Action is required')
            return
        try:
            result = self.controller.handle(action, payload)
        except Exception as exc:  # noqa: BLE001 - propagate to caller
            self._json({'error': str(exc)}, HTTPStatus.BAD_REQUEST)
            return
        self._json(result)


class WebUiServer:
    """Wraps the HTTP server in a background thread."""

    def __init__(self, bridge: WebBridge, controller: WebController, host: str = '127.0.0.1', port: int = 0):
        self._bridge = bridge
        self._controller = controller
        self._host = host
        self._port = port
        self._server: _ThreadingHTTPServer | None = None
        self._thread: threading.Thread | None = None
        self._ready = threading.Event()

    @property
    def address(self) -> tuple[str, int] | None:
        if self._server is None:
            return None
        host, port = self._server.server_address
        if isinstance(host, str):
            return host, int(port)
        return '127.0.0.1', int(port)

    def start(self) -> None:
        if self._server is not None:
            return

        def _serve():
            with _ThreadingHTTPServer((self._host, self._port), _RequestHandler, bridge=self._bridge, controller=self._controller) as server:
                self._server = server
                self._ready.set()
                try:
                    server.serve_forever()
                finally:
                    self._server = None

        self._thread = threading.Thread(target=_serve, name='web-ui-server', daemon=True)
        self._thread.start()
        self._ready.wait(timeout=5)

    def stop(self) -> None:
        server = self._server
        if server is None:
            return
        server.shutdown()
        server.server_close()
        self._server = None
        if self._thread and self._thread.is_alive():
            self._thread.join(timeout=2)
        self._thread = None


def find_free_port() -> int:
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.bind(('127.0.0.1', 0))
        return int(s.getsockname()[1])


__all__ = ["WebUiServer", "find_free_port"]

