"""Persistent plain-text copies of the GUI log and every process tab.

The log tabs live only in memory, so a force quit used to lose everything.
:class:`LogFileManager` writes the GUI's own log to
``<data dir>/logs/gui-<session>.log`` and every process start of a tab to
``<data dir>/logs/<session>/<tabkey>-<HHMMSS>.log``, where ``<session>`` is
the GUI start time.  Lines are written as they arrive and flushed at most one
second later, so the files are complete up to the last second even after a
crash.  The newest ``keep_sessions`` sessions are kept; older ones are pruned
when the GUI starts.
"""
from __future__ import annotations

import html
import re
import shutil
import time
from datetime import datetime
from pathlib import Path
from typing import Callable

from .ansi import CSI_SEQ_RE, OSC_SEQ_RE

SESSION_RE = re.compile(r'^\d{8}-\d{6}$')
GUI_LOG_RE = re.compile(r'^gui-(\d{8}-\d{6})\.log$')
_TAG_RE = re.compile(r'<[^>]+>')
_BR_RE = re.compile(r'<br\s*/?>', re.IGNORECASE)
_SLUG_RE = re.compile(r'[^a-zA-Z0-9_.-]+')


def html_to_plain(text: str) -> str:
    """Return the readable text of a log HTML fragment."""
    text = _BR_RE.sub('\n', text)
    text = _TAG_RE.sub('', text)
    return html.unescape(text).replace('\xa0', ' ')


def ansi_to_plain(text: str) -> str:
    """Strip ANSI escape sequences from process output."""
    if '\x1b' not in text:
        return text
    text = OSC_SEQ_RE.sub('', text)
    return CSI_SEQ_RE.sub('', text)


class LogFileWriter:
    """Append-only text file flushed line by line or at most every second."""

    def __init__(
        self,
        path: Path,
        *,
        flush_interval_s: float = 1.0,
        clock: Callable[[], float] = time.monotonic,
        on_error: Callable[[str], None] | None = None,
    ) -> None:
        self.path = Path(path)
        self._flush_interval_s = max(0.0, float(flush_interval_s))
        self._clock = clock
        self._on_error = on_error
        self._handle = None
        self._last_flush = self._clock()
        self._dirty = False
        self.failed = False
        try:
            self.path.parent.mkdir(parents=True, exist_ok=True)
            self._handle = open(self.path, 'a', encoding='utf-8', errors='replace')
        except OSError as exc:
            self._fail(f'cannot open log file {self.path}: {exc}')

    @property
    def is_open(self) -> bool:
        return self._handle is not None

    def write(self, text: str) -> None:
        """Append ``text`` (a newline is added when it lacks one)."""
        if self._handle is None or not text:
            return
        if not text.endswith('\n'):
            text += '\n'
        try:
            self._handle.write(text)
            self._dirty = True
            now = self._clock()
            if now - self._last_flush >= self._flush_interval_s:
                self._flush_now(now)
        except OSError as exc:
            self._fail(f'cannot write log file {self.path}: {exc}')

    def flush(self) -> None:
        if self._handle is None or not self._dirty:
            return
        try:
            self._flush_now(self._clock())
        except OSError as exc:
            self._fail(f'cannot flush log file {self.path}: {exc}')

    def close(self) -> None:
        if self._handle is None:
            return
        try:
            self.flush()
            self._handle.close()
        except OSError:
            pass
        self._handle = None

    def _flush_now(self, now: float) -> None:
        self._handle.flush()
        self._dirty = False
        self._last_flush = now

    def _fail(self, message: str) -> None:
        self.failed = True
        handle, self._handle = self._handle, None
        if handle is not None:
            try:
                handle.close()
            except OSError:
                pass
        if self._on_error:
            self._on_error(message)


class LogFileManager:
    """Own the log files of one GUI session."""

    def __init__(
        self,
        root: Path,
        *,
        keep_sessions: int = 20,
        session: str | None = None,
        flush_interval_s: float = 1.0,
        on_error: Callable[[str], None] | None = None,
    ) -> None:
        self.root = Path(root)
        self.keep_sessions = max(1, int(keep_sessions))
        self.session = session or datetime.now().strftime('%Y%m%d-%H%M%S')
        self.session_dir = self.root / self.session
        self.gui_log_path = self.root / f'gui-{self.session}.log'
        self._flush_interval_s = flush_interval_s
        self._on_error = on_error
        self._writers: list[LogFileWriter] = []
        self._gui_writer: LogFileWriter | None = None

    # -- files -------------------------------------------------------------

    def gui_writer(self) -> LogFileWriter:
        """Return the (single) writer of this session's GUI log."""
        if self._gui_writer is None:
            self._gui_writer = self._open(self.gui_log_path)
        return self._gui_writer

    def open_tab_log(self, key: str) -> LogFileWriter:
        """Open a new log file for a process start of tab ``key``."""
        slug = _SLUG_RE.sub('_', key.strip()).strip('_') or 'tab'
        stamp = datetime.now().strftime('%H%M%S')
        path = self.session_dir / f'{slug}-{stamp}.log'
        suffix = 1
        while path.exists():  # same tab started twice within a second
            path = self.session_dir / f'{slug}-{stamp}-{suffix}.log'
            suffix += 1
        return self._open(path)

    def flush_all(self) -> None:
        for writer in self._writers:
            writer.flush()

    def close_all(self) -> None:
        for writer in self._writers:
            writer.close()
        self._writers.clear()
        self._gui_writer = None

    def close(self, writer: LogFileWriter | None) -> None:
        if writer is None:
            return
        writer.close()
        if writer in self._writers:
            self._writers.remove(writer)

    def _open(self, path: Path) -> LogFileWriter:
        writer = LogFileWriter(
            path,
            flush_interval_s=self._flush_interval_s,
            on_error=self._on_error,
        )
        self._writers.append(writer)
        return writer

    # -- pruning -----------------------------------------------------------

    def sessions(self) -> list[str]:
        """Return the session ids present under the root, oldest first."""
        found: set[str] = set()
        try:
            entries = list(self.root.iterdir())
        except OSError:
            return []
        for entry in entries:
            if entry.is_dir() and SESSION_RE.match(entry.name):
                found.add(entry.name)
                continue
            match = GUI_LOG_RE.match(entry.name)
            if match and entry.is_file():
                found.add(match.group(1))
        return sorted(found)

    def prune(self) -> list[str]:
        """Delete the oldest sessions beyond ``keep_sessions`` (this one counts)."""
        sessions = [s for s in self.sessions() if s != self.session]
        excess = len(sessions) + 1 - self.keep_sessions
        removed: list[str] = []
        for session in sessions[:max(0, excess)]:
            ok = True
            session_dir = self.root / session
            if session_dir.is_dir():
                try:
                    shutil.rmtree(session_dir)
                except OSError:
                    ok = False
            gui_log = self.root / f'gui-{session}.log'
            try:
                gui_log.unlink(missing_ok=True)
            except OSError:
                ok = False
            if ok:
                removed.append(session)
        return removed


__all__ = [
    'LogFileManager',
    'LogFileWriter',
    'ansi_to_plain',
    'html_to_plain',
]
