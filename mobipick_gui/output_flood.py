"""Flood protection for process output streams.

A child process that prints the same line in a tight loop can produce tens
of thousands of lines per second.  Rendering every one of them into a text
widget blocks the Qt event loop, so every tab passes its lines through an
:class:`OutputFloodGuard` first.  The guard is pure Python (no Qt) and does
two things:

* a line repeated more than ``min_repeats`` times in a row is shown
  ``min_repeats`` times and the further copies are hidden behind one
  ``(repeated N more times)`` notice (an interim notice at most every
  ``repeat_notice_interval_s`` while the repeat goes on), and
* distinct lines are rate limited: once more than ``max_lines_per_second``
  lines arrived within one second the rest of that second is dropped and a
  notice says how many lines were dropped.

Both notices are returned as events so the caller renders them in the widget
and writes them to the disk log instead of the raw flood.
"""
from __future__ import annotations

import time
from typing import Callable, Iterable

LINE = 'line'
NOTICE = 'notice'

Event = tuple[str, str]


class OutputFloodGuard:
    """Collapse repeated lines and rate limit a line stream."""

    def __init__(
        self,
        *,
        max_lines_per_second: int = 1000,
        min_repeats: int = 3,
        repeat_notice_interval_s: float = 10.0,
        clock: Callable[[], float] = time.monotonic,
    ) -> None:
        self.max_lines_per_second = max(1, int(max_lines_per_second))
        self.min_repeats = max(1, int(min_repeats))
        self.repeat_notice_interval_s = max(0.0, float(repeat_notice_interval_s))
        self._clock = clock
        self.reset()

    def reset(self) -> None:
        """Forget all state, e.g. when a tab starts a new process."""
        self._last_line: str | None = None
        self._repeat_count = 0      # copies of the last line after its first occurrence
        self._hidden = 0            # of those, the ones not shown
        self._repeat_reported = 0   # hidden copies already covered by a notice
        self._repeat_notice_at = 0.0
        self._window_start = self._clock()
        self._window_count = 0
        self._dropped = 0
        self.total_collapsed = 0
        self.total_dropped = 0

    @property
    def has_pending(self) -> bool:
        """True when a repeat or drop summary is still unreported."""
        return self._hidden > self._repeat_reported or self._dropped > 0

    def feed(self, line: str) -> list[Event]:
        """Feed one line (newline kept) and return the events to render."""
        return self.feed_many((line,))

    def feed_many(self, lines: Iterable[str]) -> list[Event]:
        """Feed several lines at once and return the events to render."""
        events: list[Event] = []
        now = self._clock()
        for line in lines:
            if line == self._last_line:
                self._repeat_count += 1
                if self._repeat_count < self.min_repeats:
                    # a few copies are more readable than a notice: show them
                    self._roll_window(events, now)
                    if self._window_count >= self.max_lines_per_second:
                        self._dropped += 1
                        self.total_dropped += 1
                        continue
                    self._window_count += 1
                    events.append((LINE, line))
                    continue
                self._hidden += 1
                self.total_collapsed += 1
                if now - self._repeat_notice_at >= self.repeat_notice_interval_s:
                    self._emit_repeat_notice(events, now, final=False)
                continue
            self._emit_repeat_notice(events, now, final=True)
            self._roll_window(events, now)
            if self._window_count >= self.max_lines_per_second:
                self._dropped += 1
                self.total_dropped += 1
                continue
            self._window_count += 1
            self._last_line = line
            self._repeat_count = 0
            self._hidden = 0
            self._repeat_reported = 0
            self._repeat_notice_at = now
            events.append((LINE, line))
        return events

    def flush(self, *, final: bool = False) -> list[Event]:
        """Report pending summaries.

        ``final`` closes the stream (process finished): every pending repeat
        is reported and the last line is forgotten.  Otherwise a repeat
        summary is reported only ``repeat_notice_interval_s`` after the last
        one and a drop summary only after a full second, so a periodic timer
        can call this without producing a notice per tick.
        """
        events: list[Event] = []
        now = self._clock()
        if final or now - self._repeat_notice_at >= self.repeat_notice_interval_s:
            self._emit_repeat_notice(events, now, final=final)
        if final or now - self._window_start >= 1.0:
            self._roll_window(events, now, force=True)
        if final:
            self._last_line = None
            self._repeat_count = 0
            self._hidden = 0
            self._repeat_reported = 0
        return events

    # -- helpers -----------------------------------------------------------

    def _emit_repeat_notice(
        self, events: list[Event], now: float, *, final: bool
    ) -> None:
        if self._hidden <= self._repeat_reported:
            return
        count = self._hidden
        suffix = '' if final else ' so far'
        events.append(
            (NOTICE, f'... (previous line repeated {count} more times{suffix})')
        )
        self._repeat_reported = count
        self._repeat_notice_at = now

    def _roll_window(
        self, events: list[Event], now: float, *, force: bool = False
    ) -> None:
        if not force and now - self._window_start < 1.0:
            return
        if self._dropped:
            events.append(
                (
                    NOTICE,
                    f'... dropped {self._dropped} lines in the last second; '
                    'process output is being rate limited '
                    f'({self.max_lines_per_second} lines/s shown)',
                )
            )
        self._dropped = 0
        self._window_count = 0
        self._window_start = now


__all__ = ['Event', 'LINE', 'NOTICE', 'OutputFloodGuard']
