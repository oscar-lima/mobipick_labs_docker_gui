"""Utilities for bridging GUI state into a lightweight web UI."""
from __future__ import annotations

import threading
import time
from dataclasses import dataclass, field
from typing import Any, Callable, Iterable, List, Optional

from PyQt5.QtCore import QObject, pyqtSignal


@dataclass(slots=True)
class _TabState:
    """State for a tab mirrored into the web interface."""

    key: str
    label: str
    closable: bool
    logs: List[dict[str, Any]] = field(default_factory=list)


class NullWebBridge:
    """Fallback bridge that performs no work when web mode is disabled."""

    def attach_window(self, *_: Any, **__: Any) -> None:  # pragma: no cover - trivial
        return

    def ensure_tab(self, *_: Any, **__: Any) -> None:  # pragma: no cover - trivial
        return

    def remove_tab(self, *_: Any, **__: Any) -> None:  # pragma: no cover - trivial
        return

    def append_log(self, *_: Any, **__: Any) -> None:  # pragma: no cover - trivial
        return

    def update_toggle(self, *_: Any, **__: Any) -> None:  # pragma: no cover - trivial
        return

    def update_combobox(self, *_: Any, **__: Any) -> None:  # pragma: no cover - trivial
        return

    def record_status(self, *_: Any, **__: Any) -> None:  # pragma: no cover - trivial
        return

    def snapshot(self) -> dict[str, Any]:  # pragma: no cover - trivial
        return {
            "toggles": {},
            "tabs": {},
            "events": [],
            "combobox": {},
        }

    def events_since(self, *_: Any, **__: Any) -> list[dict[str, Any]]:  # pragma: no cover - trivial
        return []

    def invoke(self, callback: Callable[[], Any]) -> None:  # pragma: no cover - trivial
        callback()

    def shutdown(self) -> None:  # pragma: no cover - trivial
        return


class WebBridge(QObject):
    """Shares GUI state between the Qt widgets and a lightweight web client."""

    invoke_signal = pyqtSignal(object)

    def __init__(self, *, max_events: int = 1500, max_logs: int = 400):
        super().__init__()
        self._lock = threading.RLock()
        self._toggles: dict[str, dict[str, Any]] = {}
        self._tabs: dict[str, _TabState] = {}
        self._events: list[dict[str, Any]] = []
        self._max_events = max(10, int(max_events))
        self._max_logs = max(50, int(max_logs))
        self._next_event_id = 1
        self._combobox: dict[str, dict[str, Any]] = {}
        self._status: dict[str, Any] = {}
        self._qt_thread_id = threading.get_ident()
        self.invoke_signal.connect(self._dispatch_invocation)

    # ------------------------------------------------------------------
    # Invocation helpers
    # ------------------------------------------------------------------
    def attach_window(self, window: QObject) -> None:
        """Ensure the bridge lives on the same thread as the Qt window."""

        if window.thread() is not None and window.thread() is not self.thread():
            self.moveToThread(window.thread())

    def _dispatch_invocation(self, callback: Callable[[], Any]) -> None:
        try:
            callback()
        except Exception:
            # We intentionally swallow exceptions here; they will surface
            # through the GUI logging path instead.
            pass

    def invoke(self, callback: Callable[[], Any]) -> None:
        """Schedule *callback* to run on the Qt GUI thread."""

        if threading.get_ident() == self._qt_thread_id:
            callback()
            return
        self.invoke_signal.emit(callback)

    # ------------------------------------------------------------------
    # Event and snapshot bookkeeping
    # ------------------------------------------------------------------
    def _publish_event(self, event_type: str, payload: dict[str, Any]) -> None:
        with self._lock:
            event = {
                "id": self._next_event_id,
                "type": event_type,
                "payload": payload,
                "ts": time.time(),
            }
            self._next_event_id += 1
            self._events.append(event)
            if len(self._events) > self._max_events:
                self._events = self._events[-self._max_events :]

    def snapshot(self) -> dict[str, Any]:
        """Return a snapshot of the mirrored state."""

        with self._lock:
            return {
                "toggles": dict(self._toggles),
                "tabs": {k: {
                    "key": v.key,
                    "label": v.label,
                    "closable": v.closable,
                    "logs": list(v.logs),
                } for k, v in self._tabs.items()},
                "events": list(self._events[-50:]),
                "combobox": dict(self._combobox),
                "status": dict(self._status),
            }

    def events_since(self, event_id: int) -> list[dict[str, Any]]:
        with self._lock:
            if not self._events:
                return []
            if event_id <= 0:
                return list(self._events)
            return [evt for evt in self._events if evt["id"] > event_id]

    # ------------------------------------------------------------------
    # GUI -> web updates
    # ------------------------------------------------------------------
    def ensure_tab(self, key: str, label: str, closable: bool) -> None:
        with self._lock:
            if key in self._tabs:
                tab_state = self._tabs[key]
                changed = tab_state.label != label or tab_state.closable != closable
                tab_state.label = label
                tab_state.closable = closable
            else:
                tab_state = _TabState(key=key, label=label, closable=closable)
                self._tabs[key] = tab_state
                changed = True
        if changed:
            self._publish_event("tab", {"key": key, "label": label, "closable": closable})

    def remove_tab(self, key: str) -> None:
        removed = False
        with self._lock:
            if key in self._tabs:
                del self._tabs[key]
                removed = True
        if removed:
            self._publish_event("tab_removed", {"key": key})

    def append_log(self, key: str, html_text: str, *, origin: str) -> None:
        log_item = {
            "html": html_text,
            "origin": origin,
            "ts": time.time(),
        }
        with self._lock:
            tab = self._tabs.setdefault(key, _TabState(key=key, label=key, closable=True))
            tab.logs.append(log_item)
            if len(tab.logs) > self._max_logs:
                tab.logs = tab.logs[-self._max_logs :]
        self._publish_event("log", {"key": key, **log_item})

    def update_toggle(
        self,
        key: str,
        *,
        state: str,
        text: str,
        enabled: bool,
        colors: dict[str, str],
    ) -> None:
        with self._lock:
            self._toggles[key] = {
                "state": state,
                "text": text,
                "enabled": enabled,
                "colors": dict(colors),
            }
        self._publish_event("toggle", {"key": key, **self._toggles[key]})

    def update_combobox(
        self,
        name: str,
        *,
        options: Iterable[str],
        current: Optional[str],
    ) -> None:
        with self._lock:
            self._combobox[name] = {
                "options": list(options),
                "current": current,
            }
        self._publish_event(
            "combobox",
            {"name": name, "options": list(options), "current": current},
        )

    def record_status(self, key: str, value: Any) -> None:
        with self._lock:
            self._status[key] = value
        self._publish_event("status", {"key": key, "value": value})

    # ------------------------------------------------------------------
    # Shutdown helpers
    # ------------------------------------------------------------------
    def shutdown(self) -> None:
        with self._lock:
            self._events.append({
                "id": self._next_event_id,
                "type": "shutdown",
                "payload": {},
                "ts": time.time(),
            })
            self._next_event_id += 1


__all__ = ["WebBridge", "NullWebBridge"]

