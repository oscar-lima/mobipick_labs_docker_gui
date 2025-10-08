"""Custom QTextEdit used for logging output."""
from __future__ import annotations

from collections import deque
from typing import Deque

from PyQt5.QtCore import QTimer
from PyQt5.QtGui import QTextCursor
from PyQt5.QtWidgets import QTextEdit

from .config import CONFIG


class LogTextEdit(QTextEdit):
    """A QTextEdit configured for high-volume log output."""

    def __init__(self):
        super().__init__()
        self.setAcceptRichText(True)
        self.setReadOnly(True)
        self.setUndoRedoEnabled(False)

        log_cfg = CONFIG['log']
        self.document().setMaximumBlockCount(log_cfg['max_block_count'])
        self.setStyleSheet(
            f"QTextEdit {{ background-color: {log_cfg['background_color']}; "
            f"color: {log_cfg['text_color']}; font-family: {log_cfg['font_family']}; }}"
        )
        self._scroll_tolerance_min = max(0, int(log_cfg.get('scroll_tolerance_min', 2)))

        self._buf: Deque[tuple[bool, str]] = deque()
        self._flush_timer = QTimer(self)
        self._flush_timer.setInterval(int(log_cfg['flush_interval_ms']))
        self._flush_timer.timeout.connect(self._flush)

    def enqueue(self, is_html: bool, text: str):
        self._buf.append((is_html, text))
        if not self._flush_timer.isActive():
            self._flush_timer.start()

    def _flush(self):
        if not self._buf:
            self._flush_timer.stop()
            return
        bar = self.verticalScrollBar()
        prev_value = bar.value()
        prev_max = bar.maximum()
        tolerance = max(self._scroll_tolerance_min, bar.singleStep())
        at_bottom = prev_value >= max(0, prev_max - tolerance)

        self.setUpdatesEnabled(False)
        doc = self.document()
        doc.blockSignals(True)
        cursor = QTextCursor(doc)
        cursor.movePosition(QTextCursor.End)
        try:
            while self._buf:
                is_html, s = self._buf.popleft()
                if is_html:
                    cursor.insertHtml(s)
                else:
                    cursor.insertText(s)
            if at_bottom:
                bar.setValue(bar.maximum())
            else:
                bar.setValue(min(prev_value, bar.maximum()))
        finally:
            doc.blockSignals(False)
            self.setUpdatesEnabled(True)
            if not self._buf:
                self._flush_timer.stop()


__all__ = ['LogTextEdit']
