"""Flood protection for process output streams.

A child process that prints lines in a tight loop can produce tens of
thousands of lines per second.  Rendering every one of them into a text
widget blocks the Qt event loop, so every tab passes its lines through an
:class:`OutputFloodGuard` first.  The guard is pure Python (no Qt) and rate
limits the stream with a token bucket: ``burst_lines`` lines may arrive at any
speed (a roslaunch parameter dump or a stack trace is a burst, not a flood)
and the bucket refills at ``max_lines_per_second``; only a flood that keeps
exceeding that rate after the burst allowance is used up gets its surplus
dropped, with one notice per second saying how many lines were dropped.
Within the allowance every line passes through unchanged, also identical
ones: ROS nodes print the same text for separate events, and hiding that
would hamper debugging.

The notice is returned as an event so the caller renders it in the widget
and writes it to the disk log instead of the raw flood.
"""
from __future__ import annotations

import time
from typing import Callable, Iterable

LINE = 'line'
NOTICE = 'notice'

Event = tuple[str, str]


class OutputFloodGuard:
    """Rate limit a line stream (token bucket)."""

    def __init__(
        self,
        *,
        max_lines_per_second: int = 1000,
        burst_lines: int = 20000,
        clock: Callable[[], float] = time.monotonic,
    ) -> None:
        self.max_lines_per_second = max(1, int(max_lines_per_second))
        self.burst_lines = max(self.max_lines_per_second, int(burst_lines))
        self._clock = clock
        self.reset()

    def reset(self) -> None:
        """Forget all state, e.g. when a tab starts a new process."""
        now = self._clock()
        self._tokens = float(self.burst_lines)
        self._refilled_at = now
        self._window_start = now
        self._dropped = 0
        self.total_dropped = 0

    @property
    def has_pending(self) -> bool:
        """True when a drop summary is still unreported."""
        return self._dropped > 0

    def feed(self, line: str) -> list[Event]:
        """Feed one line (newline kept) and return the events to render."""
        return self.feed_many((line,))

    def feed_many(self, lines: Iterable[str]) -> list[Event]:
        """Feed several lines at once and return the events to render."""
        events: list[Event] = []
        now = self._clock()
        self._refill(now)
        self._roll_window(events, now)
        for line in lines:
            if self._tokens < 1.0:
                self._dropped += 1
                self.total_dropped += 1
                continue
            self._tokens -= 1.0
            events.append((LINE, line))
        return events

    def flush(self, *, final: bool = False) -> list[Event]:
        """Report a pending drop summary.

        ``final`` closes the stream (process finished): a pending summary is
        reported at once.  Otherwise only a summary older than one second is
        reported, so a periodic timer can call this without producing a
        notice per tick.
        """
        events: list[Event] = []
        now = self._clock()
        if final or now - self._window_start >= 1.0:
            self._roll_window(events, now, force=True)
        return events

    # -- helpers -----------------------------------------------------------

    def _refill(self, now: float) -> None:
        elapsed = max(0.0, now - self._refilled_at)
        self._refilled_at = now
        self._tokens = min(
            float(self.burst_lines),
            self._tokens + elapsed * self.max_lines_per_second,
        )

    def _roll_window(
        self, events: list[Event], now: float, *, force: bool = False
    ) -> None:
        if not force and now - self._window_start < 1.0:
            return
        if self._dropped:
            events.append(
                (
                    NOTICE,
                    f'... dropped {self._dropped} lines in the last second: '
                    f'this process printed more than {self.burst_lines} lines '
                    f'faster than {self.max_lines_per_second} lines/s, the '
                    'surplus is not shown',
                )
            )
        self._dropped = 0
        self._window_start = now


__all__ = ['Event', 'LINE', 'NOTICE', 'OutputFloodGuard']
