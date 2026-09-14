from __future__ import annotations

from datetime import datetime
from pathlib import Path
from typing import Callable, Iterable, Mapping
import time

import yaml

from .window_control import (
    WindowInfo,
    normalize_wid,
    select_backend,
)


class WindowLayoutManager:
    """Capture and re-apply window positions.

    The actual window enumeration and placement is delegated to a backend
    from :mod:`mobipick_gui.window_control`: ``wmctrl``/``xprop`` on X11 and
    the bundled GNOME Shell extension on Wayland sessions.
    """

    def __init__(
        self,
        state_file: str | Path,
        *,
        wmctrl_bin: str = 'wmctrl',
        xprop_bin: str = 'xprop',
        log_info: Callable[[str], None] | None = None,
        log_warning: Callable[[str], None] | None = None,
        log_debug: Callable[[str], None] | None = None,
        apply_delay_ms: int = 0,
        on_applied: Callable[[int], None] | None = None,
        backend=None,
        environ: Mapping[str, str] | None = None,
    ):
        self.state_file = Path(state_file)
        self._on_applied = on_applied
        self._log_info = log_info or (lambda _msg: None)
        self._log_warning = log_warning or (lambda _msg: None)
        self._log_debug = log_debug or (lambda _msg: None)
        self._apply_delay_ms = max(0, int(apply_delay_ms or 0))
        self._backend = backend or select_backend(
            wmctrl_bin=wmctrl_bin,
            xprop_bin=xprop_bin,
            environ=environ,
            log_info=self._log_info,
            log_warning=self._log_warning,
        )
        self._layout: dict = {}
        self._applied_ids: set[str] = set()
        self._auto_apply_done = False
        self._warned_missing = False
        self._last_capture_ids: set[str] = set()
        self._baseline_ids: set[str] = set()
        self._baseline_signatures: set[tuple[str, tuple[str, ...]]] = set()
        self._attention_suppression_active = False
        self._attention_ignored_ids: set[str] = set()
        self._attention_cleared_ids: set[str] = set()
        self._start_ts = time.monotonic()

    def record_baseline(self, *, exclude_titles: Iterable[str] | None = None):
        if not self._backend.available:
            self._auto_apply_done = True
            return
        titles = {t.strip() for t in (exclude_titles or []) if t}
        windows = self._enumerate_windows(include_classes=True, include_stack=False)
        for win in windows:
            if win.title in titles:
                continue
            self._baseline_ids.add(win.wid)
            sig = self._signature(win)
            if sig:
                self._baseline_signatures.add(sig)
        if self._baseline_ids:
            self._log_debug(f'Window layout baseline captured ({len(self._baseline_ids)} window(s)).')

    def load_layout(self) -> dict:
        data: dict = {}
        try:
            if self.state_file.is_file():
                with open(self.state_file, 'r', encoding='utf-8') as handle:
                    data = yaml.safe_load(handle) or {}
        except Exception as exc:
            self._log_warning(f'Failed to read window layout from {self.state_file}: {exc}')
            data = {}
        self._layout = data if isinstance(data, dict) else {}
        self._applied_ids.clear()
        windows = self._layout.get('windows') if isinstance(self._layout, dict) else []
        self._auto_apply_done = not bool(windows)
        return self._layout

    def reset_auto_apply(self):
        """Allow a saved layout to be applied again (e.g., after relaunching windows)."""
        windows = self._layout.get('windows') if isinstance(self._layout, dict) else []
        self._applied_ids.clear()
        self._auto_apply_done = not bool(windows)
        self._start_ts = time.monotonic()

    def set_apply_delay_ms(self, delay_ms: int) -> None:
        """Update the wait time used before auto-applying saved layouts."""
        self._apply_delay_ms = max(0, int(delay_ms or 0))

    def begin_attention_suppression(self) -> bool:
        """Treat windows appearing from now on as GUI-launched windows."""
        if not self._backend.available:
            return False
        windows = self._enumerate_windows()
        self._attention_ignored_ids = {win.wid for win in windows}
        self._attention_cleared_ids.clear()
        self._attention_suppression_active = True
        return True

    def suppress_new_window_attention(self) -> int:
        """Clear attention from newly discovered GUI-launched windows once."""
        if not self._attention_suppression_active:
            return 0
        cleared = 0
        for win in self._enumerate_windows():
            if win.wid in self._attention_ignored_ids:
                continue
            if win.wid in self._attention_cleared_ids:
                continue
            self._attention_cleared_ids.add(win.wid)
            if self._backend.clear_attention(win.wid):
                cleared += 1
        return cleared

    def end_attention_suppression(self) -> None:
        """Stop classifying newly appearing windows as GUI-launched."""
        self._attention_suppression_active = False
        self._attention_ignored_ids.clear()
        self._attention_cleared_ids.clear()

    @property
    def backend_name(self) -> str:
        """Human readable name of the window backend in use."""
        return getattr(self._backend, 'name', 'unknown')

    @property
    def backend_available(self) -> bool:
        return bool(self._backend.available)

    @property
    def backend(self):
        """The window backend (see :mod:`mobipick_gui.window_control`)."""
        return self._backend

    def has_saved_layout(self) -> bool:
        """Return whether a saved layout contains windows to rearrange."""
        windows = (
            self._layout.get('windows')
            if isinstance(self._layout, dict)
            else []
        )
        return bool(windows)

    def capture_layout(self, exclude_titles: Iterable[str] | None = None) -> dict | None:
        if not self._backend.available:
            self._warn_missing_tools()
            return None
        self._last_capture_ids = set()
        windows = self._enumerate_windows(include_classes=True, include_stack=True)
        if not windows:
            hint = getattr(self._backend, 'hint', '')
            self._log_warning(
                'No windows found to capture.' + (f' {hint}' if hint else '')
            )
            return None
        exclude = {title.strip() for title in (exclude_titles or []) if title}
        entries = []
        for win in windows:
            if win.title in exclude:
                continue
            if win.wid in self._baseline_ids:
                continue
            sig = self._signature(win)
            if sig and sig in self._baseline_signatures:
                continue
            entries.append(self._serialize_window(win))
            self._last_capture_ids.add(win.wid)
        if not entries:
            self._log_warning('Window capture skipped because no entries remained after filtering.')
            return None
        return {
            'captured_at': datetime.now().isoformat(timespec='seconds'),
            'windows': entries,
        }

    def capture_and_save(self, exclude_titles: Iterable[str] | None = None) -> bool:
        layout = self.capture_layout(exclude_titles=exclude_titles)
        if not layout:
            return False
        try:
            self.state_file.parent.mkdir(parents=True, exist_ok=True)
            with open(self.state_file, 'w', encoding='utf-8') as handle:
                yaml.safe_dump(layout, handle, sort_keys=False)
            self._layout = layout
            self._applied_ids = set(self._last_capture_ids)
            self._auto_apply_done = False
            self._log_info(f'Saved window layout to {self.state_file}')
            return True
        except Exception as exc:
            self._log_warning(f'Failed to write window layout to {self.state_file}: {exc}')
            return False

    def maybe_apply_saved_layout(self):
        if self._auto_apply_done:
            return
        windows_cfg = self._layout.get('windows') if isinstance(self._layout, dict) else []
        if not windows_cfg:
            self._auto_apply_done = True
            return
        if not self._backend.available:
            self._warn_missing_tools()
            return
        if self._apply_delay_ms:
            elapsed = int((time.monotonic() - self._start_ts) * 1000)
            if elapsed < self._apply_delay_ms:
                return

        active_windows = self._enumerate_windows(include_classes=True, include_stack=False)
        if not active_windows:
            return

        matches: list[tuple[dict, WindowInfo]] = []
        for entry in windows_cfg:
            if not isinstance(entry, dict):
                continue
            match = self._match_entry(entry, active_windows)
            if match is None:
                continue
            if match.wid in self._applied_ids:
                continue
            if match.wid in self._baseline_ids:
                continue
            sig = self._signature(match)
            if sig and sig in self._baseline_signatures:
                continue
            matches.append((entry, match))

        if not matches:
            return

        self._log_info(f'Applying saved window layout to {len(matches)} window(s).')
        for entry, win in matches:
            self._apply_entry(entry, win)
        self._apply_stack(matches)

        for _, win in matches:
            self._applied_ids.add(win.wid)
        if len(self._applied_ids) >= len(windows_cfg):
            self._auto_apply_done = True
        if self._on_applied is not None:
            try:
                self._on_applied(len(matches))
            except Exception as exc:
                self._log_debug(f'window layout applied callback failed: {exc}')

    def _warn_missing_tools(self):
        if self._warned_missing:
            return
        missing = self._backend.missing_tools()
        if missing:
            self._log_warning(
                f"Missing tools: {', '.join(missing)}. Window layout support is disabled."
            )
        self._warned_missing = True
        if not self._backend.available:
            self._auto_apply_done = True

    def _enumerate_windows(
        self,
        *,
        include_classes: bool = False,
        include_stack: bool = False,
    ) -> list[WindowInfo]:
        windows = self._backend.list_windows(
            include_classes=include_classes,
            include_stack=include_stack,
        )
        if not windows and not self._backend.available:
            self._warn_missing_tools()
        return windows

    def _serialize_window(self, window: WindowInfo) -> dict:
        data = {
            'title': window.title,
            'wm_class': window.wm_class,
            'desktop': window.desktop,
            'pid': window.pid,
            'geometry': {
                'x': window.x,
                'y': window.y,
                'width': window.width,
                'height': window.height,
            },
        }
        if window.stack_index is not None:
            data['stack_index'] = window.stack_index
        return data

    def _match_entry(self, entry: dict, active_windows: list[WindowInfo]) -> WindowInfo | None:
        title = str(entry.get('title') or '').strip()
        normalized_title = title.lower()
        saved_classes = [str(c).strip().lower() for c in entry.get('wm_class', []) if str(c).strip()]
        saved_pid = entry.get('pid')
        try:
            saved_pid = int(saved_pid) if saved_pid is not None else None
        except (TypeError, ValueError):
            saved_pid = None

        best: WindowInfo | None = None
        best_score = 0
        for win in active_windows:
            win_title = win.title.lower()
            if normalized_title and normalized_title not in win_title:
                continue
            score = 0
            if saved_classes and win.wm_class:
                overlap = set(saved_classes) & {cls.lower() for cls in win.wm_class}
                if overlap:
                    score += 5 * len(overlap)
            if normalized_title:
                if win_title == normalized_title:
                    score += 3
                else:
                    score += 1
            if saved_pid is not None and win.pid == saved_pid:
                score += 1
            if score > best_score:
                best = win
                best_score = score
        return best if best_score > 0 else None

    def _apply_entry(self, entry: dict, window: WindowInfo):
        geometry = entry.get('geometry', {})
        x = geometry.get('x')
        y = geometry.get('y')
        w = geometry.get('width')
        h = geometry.get('height')
        try:
            coords = [int(x), int(y), int(w), int(h)]
        except Exception:
            coords = []
        if len(coords) == 4:
            self._backend.move_resize(window.wid, *coords)

        desktop = entry.get('desktop')
        try:
            desk_idx = int(desktop) if desktop is not None else None
        except (TypeError, ValueError):
            desk_idx = None
        if desk_idx is not None:
            self._backend.set_desktop(window.wid, desk_idx)

    def _apply_stack(self, matches: list[tuple[dict, WindowInfo]]):
        ordered = sorted(matches, key=lambda pair: self._stacking_key(pair[0]))
        self._backend.restack([win.wid for _, win in ordered])

    @staticmethod
    def _stacking_key(entry: dict) -> int:
        try:
            return int(entry.get('stack_index', 0))
        except Exception:
            return 0

    @staticmethod
    def _signature(win: WindowInfo) -> tuple[str, tuple[str, ...]] | None:
        title = win.title.strip().lower()
        classes = tuple(sorted({cls.strip().lower() for cls in win.wm_class if cls.strip()}))
        if not title and not classes:
            return None
        return (title, classes)

    @staticmethod
    def _normalize_wid(raw: str) -> str:
        return normalize_wid(raw)
