"""Network remote-control API for the Mobipick Labs GUI.

The server exposes a small JSON-over-HTTP API so another machine (or an
automation agent such as Claude Code) can press toolbar buttons, wait for
launch events, read log tabs, and run commands in persistent ROS 1 shell
sessions inside the Mobipick containers.

Design notes:

- The HTTP server runs in a daemon thread. Anything that touches Qt objects
  is marshalled onto the GUI thread through :class:`GuiInvoker`.
- Events (button state changes, finished processes, auto-launch readiness,
  applied window layouts) are recorded in an :class:`EventBus` with
  monotonically increasing sequence numbers so clients can wait without
  missing anything.
- Shell sessions are plain ``subprocess.Popen`` pipes owned by the server.
  Their output is mirrored into a GUI tab and buffered with line sequence
  numbers so clients can decide how much of it to stream back.
"""
from __future__ import annotations

import base64
import codecs
import json
import os
import re
import secrets
import shlex
import subprocess
import threading
import time
import uuid
from collections import deque
from dataclasses import dataclass
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Any, Callable, Deque, Iterable
from urllib.parse import parse_qs, unquote, urlsplit

from PyQt5.QtCore import QObject, Qt, QThread, pyqtSignal

from .ansi import CSI_SEQ_RE, OSC_SEQ_RE

REMOTE_CONTROL_DEFAULTS: dict[str, Any] = {
    'enabled': False,
    'host': '0.0.0.0',
    'port': 8765,
    'token': '',
    'gui_timeout_s': 10.0,
    'shell_max_lines': 20000,
    'shell_start_timeout_s': 180.0,
    'default_exec_timeout_s': 60.0,
    'max_output_lines': 400,
}

END_MARKER_PREFIX = '__MPRC_END_'
PID_MARKER = '__MPRC_PID__'
_END_MARKER_RE = re.compile(r'^__MPRC_END_([0-9a-f]+)__ (-?\d+)\s*$')
_PID_MARKER_RE = re.compile(r'^__MPRC_PID__ (\d+)\s*$')
_LINE_SPLIT_RE = re.compile(r'\r?\n')


class RemoteControlError(Exception):
    """Base error carrying an HTTP status code."""

    status = HTTPStatus.BAD_REQUEST

    def __init__(self, message: str, *, status: HTTPStatus | int | None = None, **extra):
        super().__init__(message)
        self.message = message
        if status is not None:
            self.status = status
        self.extra = extra


class NotFound(RemoteControlError):
    status = HTTPStatus.NOT_FOUND


class Conflict(RemoteControlError):
    status = HTTPStatus.CONFLICT


class GuiTimeout(RemoteControlError):
    status = HTTPStatus.GATEWAY_TIMEOUT


# ---------------------------------------------------------------------------
# Event bus
# ---------------------------------------------------------------------------


class EventBus:
    """Thread-safe append-only event log with blocking waits."""

    def __init__(self, max_events: int = 5000):
        self._events: Deque[dict] = deque(maxlen=max_events)
        self._cond = threading.Condition()
        self._seq = 0

    @property
    def last_seq(self) -> int:
        with self._cond:
            return self._seq

    def emit(self, name: str, /, **data: Any) -> dict:
        with self._cond:
            self._seq += 1
            event = {
                'seq': self._seq,
                'time': time.time(),
                'name': str(name),
                'data': _jsonable(data),
            }
            self._events.append(event)
            self._cond.notify_all()
        return event

    def since(
        self,
        seq: int,
        names: Iterable[str] | None = None,
        *,
        limit: int | None = None,
    ) -> list[dict]:
        wanted = {str(n) for n in names} if names else None
        with self._cond:
            found = [
                event
                for event in self._events
                if event['seq'] > seq and (not wanted or event['name'] in wanted)
            ]
        if limit is not None and limit >= 0:
            found = found[-limit:] if limit else []
        return found

    def wait(
        self,
        names: Iterable[str] | None,
        since: int,
        timeout: float,
    ) -> dict | None:
        """Block until an event newer than ``since`` matches ``names``."""
        wanted = {str(n) for n in names} if names else None
        deadline = time.monotonic() + max(0.0, float(timeout))
        with self._cond:
            while True:
                for event in self._events:
                    if event['seq'] <= since:
                        continue
                    if wanted and event['name'] not in wanted:
                        continue
                    return event
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    return None
                self._cond.wait(remaining)


# ---------------------------------------------------------------------------
# GUI thread marshalling
# ---------------------------------------------------------------------------


class _Job:
    __slots__ = ('fn', 'result', 'error', 'done')

    def __init__(self, fn: Callable[[], Any]):
        self.fn = fn
        self.result: Any = None
        self.error: BaseException | None = None
        self.done = threading.Event()


class GuiInvoker(QObject):
    """Run callables on the Qt GUI thread from any other thread."""

    _job_signal = pyqtSignal(object)

    def __init__(self, parent: QObject | None = None):
        super().__init__(parent)
        self._job_signal.connect(self._run_job, Qt.QueuedConnection)

    def invoke(self, fn: Callable[[], Any], *, timeout: float = 10.0) -> Any:
        if QThread.currentThread() is self.thread():
            return fn()
        job = _Job(fn)
        self._job_signal.emit(job)
        if not job.done.wait(max(0.0, float(timeout))):
            raise GuiTimeout(
                f'the GUI thread did not respond within {timeout:g}s '
                '(a modal dialog or a blocking operation may be active)'
            )
        if job.error is not None:
            raise job.error
        return job.result

    def post(self, fn: Callable[[], Any]) -> None:
        """Queue ``fn`` on the GUI thread without waiting for it."""
        self._job_signal.emit(_Job(fn))

    def _run_job(self, job: _Job) -> None:
        try:
            job.result = job.fn()
        except BaseException as exc:  # noqa: BLE001 - forwarded to caller
            job.error = exc
        finally:
            job.done.set()


class DirectInvoker:
    """Invoker used in tests and headless contexts: runs callables inline."""

    def invoke(self, fn: Callable[[], Any], *, timeout: float = 10.0) -> Any:
        return fn()

    def post(self, fn: Callable[[], Any]) -> None:
        fn()


# ---------------------------------------------------------------------------
# Shell sessions
# ---------------------------------------------------------------------------


def clean_output_line(line: str) -> str:
    """Strip terminal escapes and collapse carriage-return overwrites."""
    if '\x1b' in line:
        line = OSC_SEQ_RE.sub('', line)
        line = CSI_SEQ_RE.sub('', line)
        line = line.replace('\x1b7', '').replace('\x1b8', '')
    line = line.replace('\x07', '')
    if '\r' in line:
        line = line.split('\r')[-1]
    if '\b' in line:
        chars: list[str] = []
        for ch in line:
            if ch == '\b':
                if chars:
                    chars.pop()
            else:
                chars.append(ch)
        line = ''.join(chars)
    return line


def build_exec_line(command: str, token: str) -> str:
    """Return the single shell line that runs ``command`` and prints a marker.

    The command is base64-encoded so arbitrary quoting, newlines, and shell
    syntax survive transport through the session's stdin pipe. Standard input
    of the command is redirected from ``/dev/null`` so commands that read the
    terminal fail fast instead of consuming later API requests.
    """
    encoded = base64.b64encode(command.encode('utf-8')).decode('ascii')
    return (
        f'eval "$(printf %s {shlex.quote(encoded)} | base64 -d)" </dev/null; '
        '__mprc_rc=$?; '
        f"printf '\\n{END_MARKER_PREFIX}%s__ %d\\n' {shlex.quote(token)} \"$__mprc_rc\"\n"
    )


@dataclass
class ShellCommand:
    id: int
    command: str
    token: str
    started: float
    first_seq: int
    finished: float | None = None
    exit_code: int | None = None
    last_seq: int | None = None
    timed_out: bool = False
    interrupted: bool = False
    session_closed: bool = False

    @property
    def running(self) -> bool:
        return self.finished is None

    def describe(self) -> dict:
        return {
            'id': self.id,
            'command': self.command,
            'running': self.running,
            'exit_code': self.exit_code,
            'started': self.started,
            'finished': self.finished,
            'duration_s': (
                round((self.finished or time.time()) - self.started, 3)
            ),
            'first_seq': self.first_seq,
            'last_seq': self.last_seq,
            'interrupted': self.interrupted,
            'session_closed': self.session_closed,
        }


class RemoteShellSession:
    """A persistent shell whose stdin/stdout are driven by the API."""

    def __init__(
        self,
        session_id: int,
        name: str,
        argv: list[str],
        *,
        env: dict[str, str] | None = None,
        cwd: str | None = None,
        container_name: str | None = None,
        tab_key: str | None = None,
        stream_default: bool = True,
        max_lines: int = 20000,
        on_output: Callable[['RemoteShellSession', list[str]], None] | None = None,
        on_exit: Callable[['RemoteShellSession'], None] | None = None,
    ):
        self.id = session_id
        self.name = name
        self.argv = list(argv)
        self.container_name = container_name
        self.tab_key = tab_key
        self.stream_default = bool(stream_default)
        self.created = time.time()
        self.shell_pid: int | None = None
        self._on_output = on_output
        self._on_exit = on_exit
        self._cond = threading.Condition()
        self._lines: Deque[tuple[int, str]] = deque(maxlen=max(100, int(max_lines)))
        self._line_seq = 0
        self._partial = ''
        self._commands: dict[int, ShellCommand] = {}
        self._command_counter = 0
        self._current: ShellCommand | None = None
        self._closed = False
        self._exit_code: int | None = None
        self._decoder = codecs.getincrementaldecoder('utf-8')(errors='replace')
        self.proc = subprocess.Popen(
            self.argv,
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            env=env,
            cwd=cwd,
            bufsize=0,
            start_new_session=True,
        )
        self._reader = threading.Thread(
            target=self._read_loop,
            name=f'mobipick-remote-shell-{session_id}',
            daemon=True,
        )
        self._reader.start()

    # -- state -------------------------------------------------------------

    @property
    def closed(self) -> bool:
        return self._closed

    @property
    def busy(self) -> bool:
        with self._cond:
            return self._current is not None

    @property
    def last_seq(self) -> int:
        with self._cond:
            return self._line_seq

    def describe(self) -> dict:
        with self._cond:
            current = self._current.describe() if self._current else None
            line_count = self._line_seq
        return {
            'id': self.id,
            'name': self.name,
            'container': self.container_name,
            'tab': self.tab_key,
            'shell_pid': self.shell_pid,
            'closed': self._closed,
            'exit_code': self._exit_code,
            'busy': current is not None,
            'current_command': current,
            'stream_default': self.stream_default,
            'line_count': line_count,
            'created': self.created,
        }

    def command(self, command_id: int) -> ShellCommand:
        with self._cond:
            try:
                return self._commands[int(command_id)]
            except (KeyError, ValueError):
                raise NotFound(f'unknown command id {command_id}') from None

    # -- reader ------------------------------------------------------------

    def _read_loop(self) -> None:
        stdout = self.proc.stdout
        assert stdout is not None
        try:
            while True:
                try:
                    chunk = os.read(stdout.fileno(), 65536)
                except OSError:
                    chunk = b''
                if not chunk:
                    break
                self._ingest(self._decoder.decode(chunk))
        finally:
            self._ingest(self._decoder.decode(b'', final=True), final=True)
            code = self.proc.wait()
            self._finish_session(code)

    def _ingest(self, text: str, *, final: bool = False) -> None:
        if not text and not final:
            return
        data = self._partial + text
        parts = _LINE_SPLIT_RE.split(data)
        if final:
            self._partial = ''
            complete = list(parts)
            if complete and complete[-1] == '':
                complete.pop()
        else:
            self._partial = parts.pop() if parts else ''
            complete = parts
        if not complete:
            return
        visible: list[str] = []
        finished: list[ShellCommand] = []
        with self._cond:
            for raw in complete:
                line = clean_output_line(raw)
                pid_match = _PID_MARKER_RE.match(line)
                if pid_match:
                    self.shell_pid = int(pid_match.group(1))
                    continue
                end_match = _END_MARKER_RE.match(line)
                if end_match:
                    token, code = end_match.group(1), int(end_match.group(2))
                    current = self._current
                    if current is not None and current.token == token:
                        # The marker is preceded by an extra newline so it
                        # always starts on its own line; drop that blank.
                        if (
                            self._lines
                            and self._lines[-1][0] > current.first_seq
                            and self._lines[-1][1] == ''
                        ):
                            self._lines.pop()
                            self._line_seq -= 1
                        current.exit_code = code
                        current.finished = time.time()
                        current.last_seq = self._line_seq
                        self._current = None
                        finished.append(current)
                    continue
                self._line_seq += 1
                self._lines.append((self._line_seq, line))
                visible.append(raw)
            self._cond.notify_all()
        if visible and self._on_output:
            try:
                self._on_output(self, visible)
            except Exception:
                pass

    def _finish_session(self, code: int) -> None:
        with self._cond:
            self._closed = True
            self._exit_code = code
            current = self._current
            if current is not None:
                current.finished = time.time()
                current.exit_code = code
                current.last_seq = self._line_seq
                current.session_closed = True
                self._current = None
            self._cond.notify_all()
        if self._on_exit:
            try:
                self._on_exit(self)
            except Exception:
                pass

    # -- commands ----------------------------------------------------------

    def run(self, command: str) -> ShellCommand:
        """Start ``command`` and return its record without waiting."""
        with self._cond:
            if self._closed:
                raise Conflict('shell session is closed')
            if self._current is not None:
                raise Conflict(
                    'shell session is busy running command '
                    f'{self._current.id}: {self._current.command!r}; '
                    'wait for it, poll its output, or interrupt it',
                    command=self._current.describe(),
                )
            self._command_counter += 1
            token = uuid.uuid4().hex[:12]
            record = ShellCommand(
                id=self._command_counter,
                command=command,
                token=token,
                started=time.time(),
                first_seq=self._line_seq,
            )
            self._commands[record.id] = record
            self._current = record
            if len(self._commands) > 200:
                oldest = sorted(self._commands)[: len(self._commands) - 200]
                for key in oldest:
                    self._commands.pop(key, None)
        line = build_exec_line(command, token)
        try:
            assert self.proc.stdin is not None
            self.proc.stdin.write(line.encode('utf-8'))
            self.proc.stdin.flush()
        except (OSError, ValueError) as exc:
            with self._cond:
                record.finished = time.time()
                record.exit_code = -1
                record.last_seq = self._line_seq
                record.session_closed = True
                self._current = None
                self._cond.notify_all()
            raise Conflict(f'failed to write to shell session: {exc}') from exc
        return record

    def wait(self, record: ShellCommand, timeout: float) -> bool:
        """Wait for ``record`` to finish; return False on timeout."""
        deadline = time.monotonic() + max(0.0, float(timeout))
        with self._cond:
            while record.running:
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    record.timed_out = True
                    return False
                self._cond.wait(remaining)
        return True

    def wait_for_lines(self, since_seq: int, timeout: float) -> bool:
        """Block until a line newer than ``since_seq`` exists or the session idles out."""
        deadline = time.monotonic() + max(0.0, float(timeout))
        with self._cond:
            while self._line_seq <= since_seq:
                if self._closed:
                    return False
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    return False
                self._cond.wait(remaining)
            return True

    def lines(
        self,
        *,
        since: int = 0,
        until: int | None = None,
        tail: int | None = None,
        grep: str | None = None,
        max_lines: int | None = None,
        include_partial: bool = False,
    ) -> tuple[list[dict], int, bool]:
        """Return ``(lines, total_matching, truncated)`` for the requested range."""
        pattern = re.compile(grep) if grep else None
        with self._cond:
            selected = [
                {'seq': seq, 'text': text}
                for seq, text in self._lines
                if seq > since and (until is None or seq <= until)
            ]
            partial = self._partial if include_partial else ''
        if pattern is not None:
            selected = [entry for entry in selected if pattern.search(entry['text'])]
        total = len(selected)
        truncated = False
        if tail is not None and tail >= 0 and len(selected) > tail:
            selected = selected[-tail:] if tail else []
            truncated = True
        if max_lines is not None and max_lines >= 0 and len(selected) > max_lines:
            selected = selected[-max_lines:] if max_lines else []
            truncated = True
        if partial:
            selected.append({'seq': None, 'text': clean_output_line(partial), 'partial': True})
        return selected, total, truncated

    def interrupt(self, sig: str = 'INT') -> dict:
        signal_name = str(sig or 'INT').upper().replace('SIG', '')
        if signal_name not in {'INT', 'TERM', 'KILL', 'HUP'}:
            raise RemoteControlError(f'unsupported signal {sig!r}')
        if self._closed:
            raise Conflict('shell session is closed')
        if not self.shell_pid:
            raise Conflict('shell PID is unknown; the session is not ready yet')
        with self._cond:
            current = self._current
            if current is not None:
                current.interrupted = True
        prefix = ['docker', 'exec', self.container_name] if self.container_name else []
        script = (
            f'if command -v pkill >/dev/null 2>&1; then pkill -{signal_name} -P {self.shell_pid}; '
            f'else for p in $(pgrep -P {self.shell_pid}); do kill -{signal_name} "$p"; done; fi'
        )
        cp = subprocess.run(
            [*prefix, 'sh', '-c', script],
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            check=False,
            timeout=15,
        )
        return {
            'signal': signal_name,
            'returncode': cp.returncode,
            'output': (cp.stdout or '').strip(),
        }

    def close(self, timeout: float = 5.0) -> None:
        if not self._closed:
            try:
                assert self.proc.stdin is not None
                self.proc.stdin.write(b'exit\n')
                self.proc.stdin.flush()
                self.proc.stdin.close()
            except (OSError, ValueError, AssertionError):
                pass
            try:
                self.proc.wait(timeout=timeout)
            except subprocess.TimeoutExpired:
                try:
                    self.proc.terminate()
                    self.proc.wait(timeout=2)
                except Exception:
                    try:
                        self.proc.kill()
                    except Exception:
                        pass
        if self.container_name:
            for docker_args in (['stop', '-t', '2'], ['rm', '-f']):
                try:
                    subprocess.run(
                        ['docker', *docker_args, self.container_name],
                        stdout=subprocess.DEVNULL,
                        stderr=subprocess.DEVNULL,
                        check=False,
                        timeout=30,
                    )
                except Exception:
                    pass


# ---------------------------------------------------------------------------
# Server
# ---------------------------------------------------------------------------


# A client must re-declare presence at least this often; when it lapses the
# GUI stops everything that client started (see MainWindow._sync_remote_clients).
PRESENCE_DEFAULT_TTL_S = 600.0
PRESENCE_MAX_TTL_S = 1800.0

API_INDEX = [
    ('GET', '/', 'This endpoint list.'),
    ('GET', '/status', 'GUI summary: workspace, image, running state, buttons, tabs, shells.'),
    ('GET', '/buttons', 'Toolbar buttons with their color state and text.'),
    ('POST', '/buttons/{key}/click', 'Press a button. Body: {"wait_for": [events], "timeout": s}.'),
    ('POST', '/buttons/{key}/start', 'Press only when the button is not running.'),
    ('POST', '/buttons/{key}/stop', 'Press only when the button is running.'),
    ('GET', '/presence', 'Clients that declared they are using the GUI (lights the window icon).'),
    ('POST', '/presence', 'Declare that you are using the GUI. Body: {"name": "claude", "ttl_s": 600, "note": ""}. Repeat before ttl_s (max 1800) runs out; when it lapses the GUI stops what you started.'),
    ('DELETE', '/presence', 'Declare that you are done; stops what you started unless {"keep": true}. Body or query: {"name": "claude"}.'),
    ('GET', '/events?since=N&names=a,b&follow=1&timeout=s', 'List or stream (NDJSON) events.'),
    ('POST', '/wait', 'Block until an event. Body: {"events": [names], "since": N, "timeout": s}.'),
    ('POST', '/reload', 'Re-read gui_settings.yaml and the workspace button profile without restarting the GUI.'),
    ('POST', '/tabs/{key}/stop', 'Stop the process behind a log tab: a button process, a customN command, or a remote shell.'),
    ('GET', '/tabs', 'Log tabs and whether their process runs.'),
    ('GET', '/tabs/{key}?tail=N&grep=RE', 'Plain text of a log tab.'),
    ('GET', '/dialogs', 'Active modal dialog, if any.'),
    ('POST', '/dialogs/dismiss', 'Close the active modal dialog. Body: {"button": "text|accept|reject"}.'),
    ('POST', '/command', 'Run text through the GUI custom command box. Body: {"command": "..."}.'),
    ('GET', '/shell', 'Open remote shell sessions.'),
    ('POST', '/shell', 'Open a shell in a ROS container. Body: {"name": "", "stream": true, "root": null}.'),
    ('GET', '/shell/{id}', 'Session details and its current command.'),
    ('POST', '/shell/{id}/exec', 'Run a command. Body: {"command", "stream", "tail", "grep", "timeout", "wait"}.'),
    ('GET', '/shell/{id}/output?since=N&tail=N&grep=RE&command=ID&follow=1&timeout=s', 'Fetch or stream buffered output.'),
    ('POST', '/shell/{id}/interrupt', 'Send SIGINT (or {"signal": "TERM"}) to the running command.'),
    ('POST', '/shell/{id}/settings', 'Change session defaults. Body: {"stream": bool}.'),
    ('DELETE', '/shell/{id}', 'Close the session and its container.'),
    ('POST', '/quit', 'Close the GUI with its normal cleanup.'),
]


class RemoteControlServer:
    """HTTP server that drives a GUI adapter from other threads."""

    def __init__(
        self,
        adapter: 'GuiAdapter',
        *,
        host: str = '0.0.0.0',
        port: int = 8765,
        token: str = '',
        invoker: Any = None,
        gui_timeout: float = 10.0,
        shell_max_lines: int = 20000,
        shell_start_timeout: float = 180.0,
        default_exec_timeout: float = 60.0,
        max_output_lines: int = 400,
    ):
        self.adapter = adapter
        self.host = host
        self.port = int(port)
        self.token = str(token or '')
        self.invoker = invoker or DirectInvoker()
        self.gui_timeout = max(0.5, float(gui_timeout))
        self.shell_max_lines = int(shell_max_lines)
        self.shell_start_timeout = float(shell_start_timeout)
        self.default_exec_timeout = float(default_exec_timeout)
        self.max_output_lines = int(max_output_lines)
        self.events = EventBus()
        self._sessions: dict[int, RemoteShellSession] = {}
        self._session_counter = 0
        self._sessions_lock = threading.Lock()
        self._httpd: ThreadingHTTPServer | None = None
        self._thread: threading.Thread | None = None
        self._active_requests = 0
        self._active_requests_lock = threading.Lock()
        self._clients: dict[str, dict] = {}
        self._clients_lock = threading.Lock()
        # (kind, key) of processes each client started and has not stopped yet
        self._owned: dict[str, list[dict]] = {}
        self._leave_reasons: dict[str, str] = {}
        self.started = time.time()

    # -- lifecycle ---------------------------------------------------------

    @property
    def running(self) -> bool:
        return self._httpd is not None

    @property
    def active_requests(self) -> int:
        """Number of HTTP requests currently being served (including streams)."""
        with self._active_requests_lock:
            return self._active_requests

    def _request_started(self) -> None:
        with self._active_requests_lock:
            self._active_requests += 1

    def _request_finished(self) -> None:
        with self._active_requests_lock:
            self._active_requests = max(0, self._active_requests - 1)

    # -- presence ----------------------------------------------------------
    #
    # A client that plans to send several requests announces itself first so
    # the GUI can show "in use" (window icon glow) for the whole session
    # instead of flickering per request.  Entries expire after ``ttl_s`` in
    # case the client dies without saying goodbye.

    def declare_presence(self, name: str, ttl_s: float | None = None, note: str = '') -> dict:
        name = str(name or '').strip()
        if not name:
            raise RemoteControlError('presence needs a client name')
        ttl = float(ttl_s) if ttl_s is not None else PRESENCE_DEFAULT_TTL_S
        ttl = max(1.0, min(ttl, PRESENCE_MAX_TTL_S))
        now = time.time()
        with self._clients_lock:
            self._expire_clients_locked(now)
            entry = self._clients.get(name)
            fresh = entry is None
            if fresh:
                entry = {'name': name, 'since': now}
                self._clients[name] = entry
            entry['expires'] = now + ttl
            entry['ttl_s'] = ttl
            entry['note'] = str(note or '')
            snapshot = dict(entry)
        if fresh:
            self.events.emit('client_connected', name=name, note=snapshot['note'])
        return snapshot

    def withdraw_presence(self, name: str, *, keep: bool = False) -> bool:
        """Forget ``name``; with ``keep`` its processes are left running."""
        name = str(name or '').strip()
        with self._clients_lock:
            removed = self._clients.pop(name, None) is not None
            if keep:
                self._owned.pop(name, None)
            if removed:
                self._leave_reasons[name] = 'done (processes kept)' if keep else 'done'
        if removed:
            self.events.emit('client_disconnected', name=name, keep=bool(keep))
        return removed

    # -- ownership: what each client started -------------------------------

    def _current_client(self) -> str | None:
        with self._clients_lock:
            self._expire_clients_locked(time.time())
            if not self._clients:
                return None
            return min(self._clients.values(), key=lambda entry: entry['since'])['name']

    def _record_owned(self, kind: str, key: Any) -> None:
        name = self._current_client()
        if name is None:
            return
        with self._clients_lock:
            entries = self._owned.setdefault(name, [])
            if not any(e['kind'] == kind and e['key'] == key for e in entries):
                entries.append({'kind': kind, 'key': key, 'started': time.time()})

    def _forget_owned(self, kind: str, key: Any) -> None:
        with self._clients_lock:
            for entries in self._owned.values():
                entries[:] = [e for e in entries if not (e['kind'] == kind and e['key'] == key)]

    def owned_by(self, name: str) -> list[dict]:
        with self._clients_lock:
            return [dict(e) for e in self._owned.get(name, [])]

    def leave_reason(self, name: str) -> str:
        with self._clients_lock:
            return self._leave_reasons.pop(name, 'done')

    def take_owned(self, name: str) -> list[dict]:
        """Return and forget what ``name`` started; used for cleanup after it left."""
        with self._clients_lock:
            return self._owned.pop(name, [])

    def clients(self) -> list[dict]:
        """Present clients, expired entries pruned; ``[]`` when nobody is using the GUI."""
        now = time.time()
        with self._clients_lock:
            expired = self._expire_clients_locked(now)
            entries = [dict(entry) for entry in self._clients.values()]
        for name in expired:
            self.events.emit('client_disconnected', name=name, expired=True)
        for entry in entries:
            entry['expires_in_s'] = round(max(0.0, entry['expires'] - now), 1)
        return sorted(entries, key=lambda entry: entry['since'])

    @property
    def in_use(self) -> bool:
        return bool(self.clients())

    def _expire_clients_locked(self, now: float) -> list[str]:
        expired = [name for name, entry in self._clients.items() if entry['expires'] <= now]
        for name in expired:
            del self._clients[name]
            self._leave_reasons[name] = 'presence expired'
        return expired

    @property
    def address(self) -> tuple[str, int]:
        if self._httpd is not None:
            host, port = self._httpd.server_address[:2]
            return str(host), int(port)
        return self.host, self.port

    def start(self) -> tuple[str, int]:
        if self._httpd is not None:
            return self.address
        server = self

        class Handler(_RequestHandler):
            remote = server

        httpd = ThreadingHTTPServer((self.host, self.port), Handler)
        httpd.daemon_threads = True
        self._httpd = httpd
        self._thread = threading.Thread(
            target=httpd.serve_forever,
            kwargs={'poll_interval': 0.25},
            name='mobipick-remote-control',
            daemon=True,
        )
        self._thread.start()
        self.events.emit('server_started', host=self.address[0], port=self.address[1])
        return self.address

    def stop(self) -> None:
        self.close_all_sessions()
        httpd = self._httpd
        self._httpd = None
        if httpd is not None:
            try:
                httpd.shutdown()
                httpd.server_close()
            except Exception:
                pass
        if self._thread is not None:
            self._thread.join(timeout=2)
            self._thread = None

    def emit(self, name: str, /, **data: Any) -> dict:
        return self.events.emit(name, **data)

    # -- sessions ----------------------------------------------------------

    def sessions(self) -> list[RemoteShellSession]:
        with self._sessions_lock:
            return [self._sessions[key] for key in sorted(self._sessions)]

    def session(self, session_id: Any) -> RemoteShellSession:
        try:
            key = int(session_id)
        except (TypeError, ValueError):
            raise NotFound(f'invalid shell session id {session_id!r}') from None
        with self._sessions_lock:
            session = self._sessions.get(key)
        if session is None:
            raise NotFound(f'unknown shell session {key}')
        return session

    def session_for_tab(self, tab_key: str) -> RemoteShellSession | None:
        with self._sessions_lock:
            for session in self._sessions.values():
                if session.tab_key == tab_key:
                    return session
        return None

    def open_session(
        self,
        *,
        name: str = '',
        stream: bool | None = None,
        root: bool | None = None,
        timeout: float | None = None,
    ) -> dict:
        with self._sessions_lock:
            self._session_counter += 1
            session_id = self._session_counter
        label = str(name or '').strip() or f'Remote Shell {session_id}'
        spec = self._invoke(lambda: self.adapter.shell_spec(session_id, label, root=root))
        session = RemoteShellSession(
            session_id,
            label,
            spec['argv'],
            env=spec.get('env'),
            cwd=spec.get('cwd'),
            container_name=spec.get('container_name'),
            tab_key=spec.get('tab_key'),
            stream_default=True if stream is None else bool(stream),
            max_lines=self.shell_max_lines,
            on_output=self._on_session_output,
            on_exit=self._on_session_exit,
        )
        with self._sessions_lock:
            self._sessions[session_id] = session
        self.events.emit('shell_opened', id=session_id, name=label, container=spec.get('container_name'))
        self._record_owned('shell', session_id)
        init_command = str(spec.get('init_command') or '').strip()
        ready_timeout = self.shell_start_timeout if timeout is None else float(timeout)
        init = session.run(f'{init_command}; echo "{PID_MARKER} $$"' if init_command else f'echo "{PID_MARKER} $$"')
        ready = session.wait(init, ready_timeout)
        init_lines, _, _ = session.lines(since=init.first_seq, until=init.last_seq, tail=self.max_output_lines)
        result = {
            'session': session.describe(),
            'ready': ready and not session.closed and init.exit_code == 0,
            'startup_output': [entry['text'] for entry in init_lines],
        }
        if not result['ready']:
            result['error'] = (
                'shell session exited during startup'
                if session.closed
                else 'shell session did not become ready in time'
                if not ready
                else f'startup command failed with exit code {init.exit_code}'
            )
        return result

    def stop_tab(self, key: str) -> dict:
        """Stop whatever runs behind log tab ``key`` (button, custom command, shell)."""
        session = self.session_for_tab(key)
        if session is not None:
            result = self.close_session(session.id)
            return {'tab': key, 'stopped': True, 'kind': 'shell', 'session': result.get('session')}
        result = dict(self._invoke(lambda: self.adapter.stop_tab(key)))
        if result.get('stopped'):
            self._forget_owned('tab', key)
            self._forget_owned('button', key)
        return result

    def close_session(self, session_id: Any) -> dict:
        session = self.session(session_id)
        self._forget_owned('shell', session.id)
        with self._sessions_lock:
            self._sessions.pop(session.id, None)
        session.close()
        self.events.emit('shell_closed', id=session.id, name=session.name)
        return {'session': session.describe()}

    def close_all_sessions(self) -> None:
        with self._sessions_lock:
            sessions = list(self._sessions.values())
            self._sessions.clear()
        for session in sessions:
            try:
                session.close()
            except Exception:
                pass

    def _on_session_output(self, session: RemoteShellSession, lines: list[str]) -> None:
        text = ''.join(line + '\n' for line in lines)
        self.invoker.post(lambda: self.adapter.mirror_shell_output(session, text))

    def _on_session_exit(self, session: RemoteShellSession) -> None:
        self.events.emit('shell_exited', id=session.id, name=session.name, exit_code=session._exit_code)
        self.invoker.post(lambda: self.adapter.shell_session_exited(session))

    # -- helpers -----------------------------------------------------------

    def _invoke(self, fn: Callable[[], Any], timeout: float | None = None) -> Any:
        return self.invoker.invoke(fn, timeout=self.gui_timeout if timeout is None else timeout)

    def _active_dialog(self) -> dict | None:
        try:
            return self.invoker.invoke(self.adapter.active_dialog, timeout=2.0)
        except Exception:
            return None

    def _envelope(self, payload: dict | None = None) -> dict:
        data = {'ok': True}
        if payload:
            data.update(payload)
        data['seq'] = self.events.last_seq
        return data

    # -- request dispatch ------------------------------------------------

    def handle(self, method: str, path: str, query: dict[str, str], body: dict) -> tuple[int, dict]:
        """Dispatch a non-streaming request; returns ``(status, payload)``."""
        parts = [unquote(part) for part in path.strip('/').split('/') if part]
        try:
            payload = self._dispatch(method, parts, query, body)
        except GuiTimeout as exc:
            data = {'ok': False, 'error': exc.message, 'dialog': self._active_dialog()}
            data.update(exc.extra)
            return int(exc.status), data
        except RemoteControlError as exc:
            data = {'ok': False, 'error': exc.message}
            data.update(_jsonable(exc.extra))
            return int(exc.status), data
        except Exception as exc:  # noqa: BLE001 - reported to the client
            return int(HTTPStatus.INTERNAL_SERVER_ERROR), {
                'ok': False,
                'error': f'{type(exc).__name__}: {exc}',
            }
        return int(HTTPStatus.OK), self._envelope(payload)

    def _dispatch(self, method: str, parts: list[str], query: dict[str, str], body: dict) -> dict:
        if not parts:
            if method != 'GET':
                raise RemoteControlError('method not allowed', status=HTTPStatus.METHOD_NOT_ALLOWED)
            return {
                'name': 'mobipick-labs-docker-gui remote control',
                'endpoints': [
                    {'method': m, 'path': p, 'description': d} for m, p, d in API_INDEX
                ],
                'events': [
                    'server_started', 'button_state', 'process_finished',
                    'auto_launch_started', 'auto_launch_ready', 'auto_launch_complete',
                    'auto_launch_stopped', 'window_layout_applied', 'shell_opened',
                    'shell_exited', 'shell_closed', 'client_connected',
                    'client_disconnected', 'config_reloaded', 'gui_closing',
                ],
            }
        head = parts[0]
        if head == 'status' and method == 'GET':
            status = self._invoke(self.adapter.status)
            status['shells'] = [session.describe() for session in self.sessions()]
            status['clients'] = self.clients()
            status['in_use'] = bool(status['clients'])
            status['server'] = {
                'host': self.address[0],
                'port': self.address[1],
                'uptime_s': round(time.time() - self.started, 1),
            }
            return status
        if head == 'buttons':
            return self._dispatch_buttons(method, parts[1:], body)
        if head == 'presence' and len(parts) == 1:
            if method == 'GET':
                return {'clients': self.clients()}
            if method == 'POST':
                entry = self.declare_presence(
                    body.get('name') or query.get('name') or '',
                    _float_param(body.get('ttl_s'), None),
                    str(body.get('note') or ''),
                )
                entry['expires_in_s'] = round(entry['expires'] - time.time(), 1)
                return {'client': entry, 'clients': self.clients()}
            if method == 'DELETE':
                name = str(body.get('name') or query.get('name') or '')
                keep = _bool_param(body.get('keep', query.get('keep')), False)
                return {'removed': self.withdraw_presence(name, keep=keep), 'clients': self.clients()}
            raise RemoteControlError('method not allowed', status=HTTPStatus.METHOD_NOT_ALLOWED)
        if head == 'events' and method == 'GET':
            since = _int_param(query.get('since'), 0)
            names = _list_param(query.get('names'))
            limit = _int_param(query.get('limit'), 200)
            events = self.events.since(since, names, limit=limit)
            return {'events': events}
        if head == 'wait' and method == 'POST':
            names = _list_param(body.get('events') or body.get('event'))
            since = _int_param(body.get('since'), None)
            timeout = _float_param(body.get('timeout'), 120.0)
            return self._wait_for_event(names, since, timeout)
        if head == 'tabs':
            if method == 'POST' and len(parts) == 3 and parts[2] == 'stop':
                return self.stop_tab(parts[1])
            if method != 'GET':
                raise RemoteControlError('method not allowed', status=HTTPStatus.METHOD_NOT_ALLOWED)
            if len(parts) == 1:
                return {'tabs': self._invoke(self.adapter.tabs)}
            key = parts[1]
            tail = _int_param(query.get('tail'), 100)
            grep = query.get('grep') or None
            text = self._invoke(lambda: self.adapter.tab_text(key))
            lines = text.splitlines()
            total = len(lines)
            if grep:
                pattern = re.compile(grep)
                lines = [line for line in lines if pattern.search(line)]
            matched = len(lines)
            truncated = False
            if tail is not None and tail >= 0 and len(lines) > tail:
                lines = lines[-tail:] if tail else []
                truncated = True
            return {
                'tab': key,
                'total_lines': total,
                'matched_lines': matched,
                'truncated': truncated,
                'lines': lines,
            }
        if head == 'dialogs':
            if method == 'GET' and len(parts) == 1:
                return {'dialog': self._invoke(self.adapter.active_dialog)}
            if method == 'POST' and len(parts) == 2 and parts[1] == 'dismiss':
                button = str(body.get('button') or 'reject')
                return self._invoke(lambda: self.adapter.dismiss_dialog(button))
        if head == 'reload' and method == 'POST':
            return self._invoke(self.adapter.reload_configuration)
        if head == 'command' and method == 'POST':
            command = str(body.get('command') or '').strip()
            if not command:
                raise RemoteControlError('"command" is required')
            result = dict(self._invoke(lambda: self.adapter.run_gui_command(command)))
            if result.get('accepted') and result.get('tab'):
                self._record_owned('tab', result['tab'])
            return result
        if head == 'shell':
            return self._dispatch_shell(method, parts[1:], query, body)
        if head == 'quit' and method == 'POST':
            self.events.emit('gui_closing')
            self.invoker.post(self.adapter.quit)
            return {'quitting': True}
        raise NotFound(f'no endpoint for {method} /{"/".join(parts)}')

    def _dispatch_buttons(self, method: str, parts: list[str], body: dict) -> dict:
        if not parts:
            if method != 'GET':
                raise RemoteControlError('method not allowed', status=HTTPStatus.METHOD_NOT_ALLOWED)
            return {'buttons': self._invoke(self.adapter.buttons)}
        key = parts[0]
        action = parts[1] if len(parts) > 1 else 'click'
        if method != 'POST' or action not in {'click', 'start', 'stop'}:
            raise NotFound(f'unknown button action {action!r}')
        names = _list_param(body.get('wait_for'))
        timeout = _float_param(body.get('timeout'), 120.0)
        since = self.events.last_seq
        result = self._invoke(lambda: self.adapter.press_button(key, action))
        result = dict(result)
        if result.get('accepted'):
            # a click toggles: it starts the process unless it was running
            if action == 'stop' or (action == 'click' and result.get('was_running')):
                self._forget_owned('button', key)
            else:
                self._record_owned('button', key)
        if names and result.get('accepted'):
            result['wait'] = self._wait_for_event(names, since, timeout)
        return result

    def _wait_for_event(self, names: list[str], since: int | None, timeout: float) -> dict:
        start_seq = self.events.last_seq if since is None else since
        event = self.events.wait(names, start_seq, timeout)
        if event is None:
            return {
                'timed_out': True,
                'events': names,
                'since': start_seq,
                'timeout': timeout,
                'event': None,
            }
        return {'timed_out': False, 'since': start_seq, 'event': event}

    def _dispatch_shell(self, method: str, parts: list[str], query: dict[str, str], body: dict) -> dict:
        if not parts:
            if method == 'GET':
                return {'shells': [session.describe() for session in self.sessions()]}
            if method == 'POST':
                return self.open_session(
                    name=str(body.get('name') or ''),
                    stream=_bool_param(body.get('stream'), None),
                    root=_bool_param(body.get('root'), None),
                    timeout=_float_param(body.get('timeout'), None),
                )
            raise RemoteControlError('method not allowed', status=HTTPStatus.METHOD_NOT_ALLOWED)
        session = self.session(parts[0])
        action = parts[1] if len(parts) > 1 else ''
        if method == 'GET' and not action:
            return {'session': session.describe()}
        if (method == 'DELETE' and not action) or (method == 'POST' and action == 'close'):
            return self.close_session(session.id)
        if method == 'POST' and action == 'exec':
            return self._exec(session, body)
        if method == 'GET' and action == 'output':
            return self._output(session, query)
        if method == 'POST' and action == 'interrupt':
            return session.interrupt(str(body.get('signal') or 'INT'))
        if method == 'POST' and action == 'settings':
            stream = _bool_param(body.get('stream'), None)
            if stream is not None:
                session.stream_default = stream
            return {'session': session.describe()}
        raise NotFound(f'unknown shell action {method} {action!r}')

    def _exec(self, session: RemoteShellSession, body: dict) -> dict:
        command = body.get('command')
        if not isinstance(command, str) or not command.strip():
            raise RemoteControlError('"command" is required')
        stream = _bool_param(body.get('stream'), session.stream_default)
        wait = _bool_param(body.get('wait'), True)
        timeout = _float_param(body.get('timeout'), self.default_exec_timeout)
        tail = _int_param(body.get('tail'), None)
        grep = body.get('grep') or None
        max_lines = _int_param(body.get('max_lines'), self.max_output_lines)
        record = session.run(command)
        if not wait:
            return {'command': record.describe(), 'session_id': session.id, 'waited': False}
        finished = session.wait(record, timeout)
        result: dict[str, Any] = {
            'command': record.describe(),
            'session_id': session.id,
            'waited': True,
            'timed_out': not finished,
        }
        lines, total, truncated = session.lines(
            since=record.first_seq,
            until=record.last_seq,
            tail=tail,
            grep=grep,
            max_lines=max_lines if stream else 0,
            include_partial=stream and not finished,
        )
        result['line_count'] = total
        if stream:
            result['output'] = [entry['text'] for entry in lines]
            result['truncated'] = truncated
        else:
            result['output_hint'] = (
                f'GET /shell/{session.id}/output?command={record.id}&tail=50 '
                'returns the buffered output'
            )
        if not finished:
            result['hint'] = (
                'command still running; poll /output, wait again, or POST /interrupt'
            )
        return result

    def _output(self, session: RemoteShellSession, query: dict[str, str]) -> dict:
        since = _int_param(query.get('since'), 0)
        until: int | None = None
        command_id = _int_param(query.get('command'), None)
        record = None
        if command_id is not None:
            record = session.command(command_id)
            since = max(since, record.first_seq)
            until = record.last_seq
        tail = _int_param(query.get('tail'), None)
        grep = query.get('grep') or None
        max_lines = _int_param(query.get('max_lines'), self.max_output_lines)
        lines, total, truncated = session.lines(
            since=since,
            until=until,
            tail=tail,
            grep=grep,
            max_lines=max_lines,
            include_partial=_bool_param(query.get('partial'), False),
        )
        return {
            'session_id': session.id,
            'command': record.describe() if record else None,
            'since': since,
            'last_seq': session.last_seq,
            'matched_lines': total,
            'truncated': truncated,
            'lines': lines,
            'busy': session.busy,
            'closed': session.closed,
        }

    # -- streaming ---------------------------------------------------------

    def stream_events(self, query: dict[str, str], write: Callable[[dict], bool]) -> None:
        since = _int_param(query.get('since'), self.events.last_seq)
        names = _list_param(query.get('names'))
        timeout = _float_param(query.get('timeout'), 300.0)
        deadline = time.monotonic() + timeout
        for event in self.events.since(since, names):
            since = event['seq']
            if not write(event):
                return
        while True:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                write({'name': 'stream_timeout', 'seq': self.events.last_seq})
                return
            event = self.events.wait(names, since, min(remaining, 1.0))
            if event is None:
                continue
            since = event['seq']
            if not write(event):
                return

    def stream_shell_output(
        self,
        session: RemoteShellSession,
        query: dict[str, str],
        write: Callable[[dict], bool],
    ) -> None:
        since = _int_param(query.get('since'), None)
        grep = query.get('grep') or None
        command_id = _int_param(query.get('command'), None)
        timeout = _float_param(query.get('timeout'), 300.0)
        record = session.command(command_id) if command_id is not None else None
        if since is None:
            since = record.first_seq if record else session.last_seq
        deadline = time.monotonic() + timeout
        while True:
            lines, _, _ = session.lines(since=since, grep=grep, max_lines=None)
            for entry in lines:
                since = entry['seq']
                if not write(entry):
                    return
            with session._cond:
                current = session._current
            finished = record is not None and not record.running
            idle = record is None and current is None
            if finished or (idle and since >= session.last_seq) or session.closed:
                if session.lines(since=since, grep=grep, max_lines=None)[0]:
                    continue
                write({
                    'done': True,
                    'closed': session.closed,
                    'busy': session.busy,
                    'command': record.describe() if record else None,
                    'last_seq': session.last_seq,
                })
                return
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                write({'done': False, 'timed_out': True, 'last_seq': session.last_seq})
                return
            session.wait_for_lines(since, min(remaining, 1.0))


# ---------------------------------------------------------------------------
# HTTP plumbing
# ---------------------------------------------------------------------------


class _RequestHandler(BaseHTTPRequestHandler):
    remote: RemoteControlServer
    protocol_version = 'HTTP/1.1'
    server_version = 'MobipickRemoteControl/1.0'

    def log_message(self, format: str, *args: Any) -> None:  # noqa: A002 - Qt API naming
        return

    def _authorized(self) -> bool:
        token = self.remote.token
        if not token:
            return True
        header = self.headers.get('Authorization', '')
        if header.lower().startswith('bearer ') and secrets.compare_digest(header[7:].strip(), token):
            return True
        alt = self.headers.get('X-Mobipick-Token', '')
        if alt and secrets.compare_digest(alt.strip(), token):
            return True
        query = parse_qs(urlsplit(self.path).query)
        query_token = (query.get('token') or [''])[0]
        return bool(query_token) and secrets.compare_digest(query_token, token)

    def _send_json(self, status: int, payload: dict) -> None:
        data = json.dumps(payload, ensure_ascii=False).encode('utf-8')
        self.send_response(status)
        self.send_header('Content-Type', 'application/json; charset=utf-8')
        self.send_header('Content-Length', str(len(data)))
        self.send_header('Cache-Control', 'no-store')
        self.end_headers()
        self.wfile.write(data)

    def _read_body(self) -> dict:
        length = int(self.headers.get('Content-Length') or 0)
        if length <= 0:
            return {}
        raw = self.rfile.read(length)
        if not raw.strip():
            return {}
        try:
            body = json.loads(raw.decode('utf-8'))
        except (UnicodeDecodeError, json.JSONDecodeError) as exc:
            raise RemoteControlError(f'invalid JSON body: {exc}') from exc
        if not isinstance(body, dict):
            raise RemoteControlError('JSON body must be an object')
        return body

    def _handle(self, method: str) -> None:
        self.remote._request_started()
        try:
            self._handle_request(method)
        finally:
            self.remote._request_finished()

    def _handle_request(self, method: str) -> None:
        if not self._authorized():
            self._send_json(int(HTTPStatus.UNAUTHORIZED), {'ok': False, 'error': 'missing or invalid token'})
            return
        split = urlsplit(self.path)
        query = {key: values[-1] for key, values in parse_qs(split.query, keep_blank_values=True).items()}
        query.pop('token', None)
        try:
            body = self._read_body() if method in {'POST', 'PUT', 'PATCH', 'DELETE'} else {}
        except RemoteControlError as exc:
            self._send_json(int(exc.status), {'ok': False, 'error': exc.message})
            return
        if method == 'GET' and _bool_param(query.get('follow'), False):
            if self._stream(split.path, query):
                return
        status, payload = self.remote.handle(method, split.path, query, body)
        self._send_json(status, payload)

    def _stream(self, path: str, query: dict[str, str]) -> bool:
        parts = [unquote(part) for part in path.strip('/').split('/') if part]
        if parts == ['events']:
            producer = lambda write: self.remote.stream_events(query, write)  # noqa: E731
        elif len(parts) == 3 and parts[0] == 'shell' and parts[2] == 'output':
            try:
                session = self.remote.session(parts[1])
            except RemoteControlError as exc:
                self._send_json(int(exc.status), {'ok': False, 'error': exc.message})
                return True
            producer = lambda write: self.remote.stream_shell_output(session, query, write)  # noqa: E731
        else:
            return False
        self.send_response(int(HTTPStatus.OK))
        self.send_header('Content-Type', 'application/x-ndjson; charset=utf-8')
        self.send_header('Transfer-Encoding', 'chunked')
        self.send_header('Cache-Control', 'no-store')
        self.send_header('X-Accel-Buffering', 'no')
        self.end_headers()

        def write(item: dict) -> bool:
            data = (json.dumps(item, ensure_ascii=False) + '\n').encode('utf-8')
            try:
                self.wfile.write(f'{len(data):x}\r\n'.encode('ascii') + data + b'\r\n')
                self.wfile.flush()
                return True
            except (BrokenPipeError, ConnectionResetError, OSError):
                return False

        try:
            producer(write)
        finally:
            try:
                self.wfile.write(b'0\r\n\r\n')
                self.wfile.flush()
            except OSError:
                pass
            self.close_connection = True
        return True

    def do_GET(self) -> None:  # noqa: N802 - http.server API
        self._handle('GET')

    def do_POST(self) -> None:  # noqa: N802 - http.server API
        self._handle('POST')

    def do_DELETE(self) -> None:  # noqa: N802 - http.server API
        self._handle('DELETE')

    def do_PATCH(self) -> None:  # noqa: N802 - http.server API
        self._handle('PATCH')


# ---------------------------------------------------------------------------
# Adapter protocol (implemented by MainWindow's remote adapter)
# ---------------------------------------------------------------------------


class GuiAdapter:
    """Interface the server expects; every method runs on the GUI thread."""

    def status(self) -> dict:
        raise NotImplementedError

    def buttons(self) -> list[dict]:
        raise NotImplementedError

    def press_button(self, key: str, action: str) -> dict:
        raise NotImplementedError

    def tabs(self) -> list[dict]:
        raise NotImplementedError

    def tab_text(self, key: str) -> str:
        raise NotImplementedError

    def stop_tab(self, key: str) -> dict:
        raise NotImplementedError

    def reload_configuration(self) -> dict:
        raise NotImplementedError

    def stop_owned(self, name: str, entries: list[dict]) -> list[str]:
        raise NotImplementedError

    def active_dialog(self) -> dict | None:
        return None

    def dismiss_dialog(self, button: str) -> dict:
        raise NotImplementedError

    def run_gui_command(self, command: str) -> dict:
        raise NotImplementedError

    def shell_spec(self, session_id: int, label: str, *, root: bool | None) -> dict:
        """Return ``{'argv', 'env', 'cwd', 'container_name', 'tab_key', 'init_command'}``."""
        raise NotImplementedError

    def mirror_shell_output(self, session: RemoteShellSession, text: str) -> None:
        pass

    def shell_session_exited(self, session: RemoteShellSession) -> None:
        pass

    def quit(self) -> None:
        raise NotImplementedError


# ---------------------------------------------------------------------------
# Small parsing helpers
# ---------------------------------------------------------------------------


def _jsonable(value: Any) -> Any:
    if isinstance(value, dict):
        return {str(k): _jsonable(v) for k, v in value.items()}
    if isinstance(value, (list, tuple, set)):
        return [_jsonable(v) for v in value]
    if isinstance(value, (str, int, float, bool)) or value is None:
        return value
    return str(value)


def _int_param(value: Any, default: int | None) -> int | None:
    if value is None or value == '':
        return default
    try:
        return int(value)
    except (TypeError, ValueError):
        raise RemoteControlError(f'expected an integer, got {value!r}') from None


def _float_param(value: Any, default: float | None) -> float | None:
    if value is None or value == '':
        return default
    try:
        return float(value)
    except (TypeError, ValueError):
        raise RemoteControlError(f'expected a number, got {value!r}') from None


def _bool_param(value: Any, default: bool | None) -> bool | None:
    if value is None or value == '':
        return default
    if isinstance(value, bool):
        return value
    text = str(value).strip().lower()
    if text in {'1', 'true', 'yes', 'on'}:
        return True
    if text in {'0', 'false', 'no', 'off'}:
        return False
    raise RemoteControlError(f'expected a boolean, got {value!r}')


def _list_param(value: Any) -> list[str]:
    if value is None:
        return []
    if isinstance(value, str):
        return [item.strip() for item in value.split(',') if item.strip()]
    if isinstance(value, (list, tuple, set)):
        return [str(item).strip() for item in value if str(item).strip()]
    raise RemoteControlError(f'expected a list or comma-separated string, got {value!r}')


def remote_control_settings(config: dict | None, overrides: dict | None = None) -> dict:
    """Merge defaults, the config section, and CLI/env overrides."""
    merged = dict(REMOTE_CONTROL_DEFAULTS)
    for source in (config or {}, overrides or {}):
        for key, value in source.items():
            if value is None:
                continue
            merged[str(key)] = value
    merged['enabled'] = bool(_bool_param(merged.get('enabled'), False))
    merged['host'] = str(merged.get('host') or '0.0.0.0')
    port = merged.get('port')
    try:
        merged['port'] = 8765 if port is None or port == '' else int(port)
    except (TypeError, ValueError):
        merged['port'] = 8765
    merged['token'] = str(merged.get('token') or '')
    return merged


__all__ = [
    'API_INDEX',
    'DirectInvoker',
    'EventBus',
    'GuiAdapter',
    'GuiInvoker',
    'GuiTimeout',
    'REMOTE_CONTROL_DEFAULTS',
    'RemoteControlError',
    'RemoteControlServer',
    'RemoteShellSession',
    'build_exec_line',
    'clean_output_line',
    'remote_control_settings',
]
