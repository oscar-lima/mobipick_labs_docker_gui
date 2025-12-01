from __future__ import annotations

import re
import shutil
import subprocess
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Callable, Iterable
import time

import yaml


@dataclass
class WindowInfo:
    wid: str
    title: str
    desktop: int | None
    pid: int | None
    x: int
    y: int
    width: int
    height: int
    wm_class: list[str]
    stack_index: int | None = None


class WindowLayoutManager:
    """Capture and re-apply window positions via wmctrl/xprop."""

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
    ):
        self.state_file = Path(state_file)
        self._wmctrl_bin = wmctrl_bin
        self._xprop_bin = xprop_bin
        self._log_info = log_info or (lambda _msg: None)
        self._log_warning = log_warning or (lambda _msg: None)
        self._log_debug = log_debug or (lambda _msg: None)
        self._apply_delay_ms = max(0, int(apply_delay_ms or 0))
        self._wmctrl_available = bool(shutil.which(wmctrl_bin))
        self._xprop_available = bool(shutil.which(xprop_bin))
        self._layout: dict = {}
        self._applied_ids: set[str] = set()
        self._auto_apply_done = False
        self._warned_missing = False
        self._last_capture_ids: set[str] = set()
        self._baseline_ids: set[str] = set()
        self._baseline_signatures: set[tuple[str, tuple[str, ...]]] = set()
        self._start_ts = time.monotonic()

    def record_baseline(self, *, exclude_titles: Iterable[str] | None = None):
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

    def capture_layout(self, exclude_titles: Iterable[str] | None = None) -> dict | None:
        if not self._wmctrl_available:
            self._warn_missing_tools()
            return None
        self._last_capture_ids = set()
        windows = self._enumerate_windows(include_classes=True, include_stack=True)
        if not windows:
            self._log_warning('No windows found to capture.')
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
        if not self._wmctrl_available:
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

    def _warn_missing_tools(self):
        if self._warned_missing:
            return
        msg_parts = []
        if not self._wmctrl_available:
            msg_parts.append('wmctrl')
        if not self._xprop_available:
            msg_parts.append('xprop')
        if msg_parts:
            self._log_warning(f"Missing tools: {', '.join(msg_parts)}. Window layout support is disabled.")
        self._warned_missing = True
        if not self._wmctrl_available:
            self._auto_apply_done = True

    def _enumerate_windows(
        self,
        *,
        include_classes: bool = False,
        include_stack: bool = False,
    ) -> list[WindowInfo]:
        try:
            cp = subprocess.run(
                [self._wmctrl_bin, '-lpG'],
                check=False,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
            )
        except FileNotFoundError:
            self._wmctrl_available = False
            self._warn_missing_tools()
            return []

        stdout = cp.stdout or ''
        stack_map = self._stacking_map() if include_stack else {}
        windows: list[WindowInfo] = []
        for idx, line in enumerate(stdout.splitlines()):
            parts = line.split(None, 8)
            if len(parts) < 9:
                continue
            wid_raw, desktop_raw, pid_raw, x_raw, y_raw, width_raw, height_raw, _host, title = parts
            try:
                desktop = int(desktop_raw)
            except ValueError:
                desktop = None
            try:
                pid = int(pid_raw)
            except ValueError:
                pid = None
            try:
                x = int(x_raw)
                y = int(y_raw)
                width = int(width_raw)
                height = int(height_raw)
            except ValueError:
                continue
            win = WindowInfo(
                wid=self._normalize_wid(wid_raw),
                title=title.strip(),
                desktop=desktop,
                pid=pid,
                x=x,
                y=y,
                width=width,
                height=height,
                wm_class=[],
                stack_index=None,
            )
            if include_classes:
                win.wm_class = self._read_wm_class(win.wid)
            if include_stack:
                win.stack_index = stack_map.get(win.wid, idx)
            windows.append(win)
        return windows

    def _stacking_map(self) -> dict[str, int]:
        if not self._xprop_available:
            return {}
        try:
            cp = subprocess.run(
                [self._xprop_bin, '-root', '_NET_CLIENT_LIST_STACKING'],
                check=False,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
            )
        except FileNotFoundError:
            self._xprop_available = False
            self._warn_missing_tools()
            return {}
        stdout = cp.stdout or ''
        order = [self._normalize_wid(match) for match in re.findall(r'0x[0-9a-fA-F]+', stdout)]
        return {wid: idx for idx, wid in enumerate(order)}

    def _read_wm_class(self, wid: str) -> list[str]:
        if not self._xprop_available:
            return []
        try:
            cp = subprocess.run(
                [self._xprop_bin, '-id', wid, 'WM_CLASS'],
                check=False,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
            )
        except FileNotFoundError:
            self._xprop_available = False
            self._warn_missing_tools()
            return []
        if cp.returncode != 0:
            return []
        stdout = cp.stdout or ''
        if '=' in stdout:
            _, raw_value = stdout.split('=', 1)
        else:
            raw_value = stdout
        parts = []
        for piece in raw_value.split(','):
            cleaned = piece.strip().strip('"')
            if cleaned:
                parts.append(cleaned)
        return parts

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
        saved_classes = [str(c).strip().lower() for c in entry.get('wm_class', []) if str(c).strip()]
        saved_pid = entry.get('pid')
        try:
            saved_pid = int(saved_pid) if saved_pid is not None else None
        except (TypeError, ValueError):
            saved_pid = None

        best: WindowInfo | None = None
        best_score = 0
        for win in active_windows:
            score = 0
            if saved_classes and win.wm_class:
                overlap = set(saved_classes) & {cls.lower() for cls in win.wm_class}
                if overlap:
                    score += 5 * len(overlap)
            if title:
                lowered = win.title.lower()
                if lowered == title.lower():
                    score += 3
                elif title.lower() in lowered:
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
            # drop maximized flags before resizing/repositioning so wmctrl can move the window
            self._run_wmctrl(['-i', '-r', window.wid, '-b', 'remove,maximized_vert,maximized_horz'])
        if len(coords) == 4:
            geometry_arg = f"0,{coords[0]},{coords[1]},{coords[2]},{coords[3]}"
            self._run_wmctrl(['-i', '-r', window.wid, '-e', geometry_arg])

        desktop = entry.get('desktop')
        try:
            desk_idx = int(desktop) if desktop is not None else None
        except (TypeError, ValueError):
            desk_idx = None
        if desk_idx is not None:
            self._run_wmctrl(['-i', '-r', window.wid, '-t', str(desk_idx)])

    def _apply_stack(self, matches: list[tuple[dict, WindowInfo]]):
        ordered = sorted(matches, key=lambda pair: self._stacking_key(pair[0]))
        for _, win in ordered:
            self._run_wmctrl(['-i', '-r', win.wid, '-b', 'remove,below,above'])
        for _, win in ordered:
            self._run_wmctrl(['-i', '-a', win.wid])

    def _run_wmctrl(self, args: list[str]):
        try:
            cmd = [self._wmctrl_bin, *args]
            self._log_info(' '.join(cmd))
            subprocess.run(
                cmd,
                check=False,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
            )
        except FileNotFoundError:
            self._wmctrl_available = False
            self._warn_missing_tools()

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
        value = str(raw).strip().lower()
        if value.startswith('0x'):
            return value
        try:
            return hex(int(value))
        except ValueError:
            return value
