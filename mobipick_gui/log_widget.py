"""Custom QTextEdit used for logging output."""
from __future__ import annotations

from collections import deque
from typing import Deque

from PyQt5.QtCore import QTimer
from PyQt5.QtGui import QTextCursor
from PyQt5.QtWidgets import QTextEdit

from .config import CONFIG

NOTICE_COLOR = '#ffa94d'
_LINE_BREAKS = ('\n', '\u2028', '\u2029')


class LogTextEdit(QTextEdit):
    """A QTextEdit configured for high-volume log output.

    Three guards keep the widget responsive when a process floods it: the
    pending buffer is bounded (the oldest entries are dropped with a notice),
    each flush tick renders at most ``max_entries_per_flush`` entries, and the
    document is trimmed to ``max_characters`` (``setMaximumBlockCount`` alone
    does not bound HTML lines, which share one block).
    """

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
        self._max_characters = max(0, int(log_cfg.get('max_characters', 0) or 0))
        flood_cfg = log_cfg.get('flood') or {}
        self._max_pending_entries = max(
            1, int(flood_cfg.get('max_pending_entries', 20000))
        )
        self._max_entries_per_flush = max(
            1, int(flood_cfg.get('max_entries_per_flush', 2000))
        )

        self._buf: Deque[tuple[bool, str]] = deque()
        self._dropped_pending = 0
        self._flush_timer = QTimer(self)
        self._flush_timer.setInterval(int(log_cfg['flush_interval_ms']))
        self._flush_timer.timeout.connect(self._flush_tick)

    def enqueue(self, is_html: bool, text: str):
        self._buf.append((is_html, text))
        while len(self._buf) > self._max_pending_entries:
            self._buf.popleft()
            self._dropped_pending += 1
        if not self._flush_timer.isActive():
            self._flush_timer.start()

    def pending_count(self) -> int:
        return len(self._buf)

    def _flush_tick(self):
        self._flush(limit=self._max_entries_per_flush)

    def _flush(self, limit: int | None = None):
        """Render buffered entries; ``limit=None`` drains the whole buffer."""
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
            if self._dropped_pending:
                dropped, self._dropped_pending = self._dropped_pending, 0
                cursor.insertHtml(
                    f'<span style="color:{NOTICE_COLOR}"><i>... dropped '
                    f'{dropped} buffered lines; the log widget cannot keep '
                    'up with the process output</i></span><br>'
                )
            remaining = limit if limit is not None else len(self._buf)
            while self._buf and remaining > 0:
                is_html, s = self._buf.popleft()
                remaining -= 1
                if is_html:
                    cursor.insertHtml(s)
                else:
                    cursor.insertText(s)
            self._trim_document(doc)
            if at_bottom:
                bar.setValue(bar.maximum())
            else:
                bar.setValue(min(prev_value, bar.maximum()))
        finally:
            doc.blockSignals(False)
            self.setUpdatesEnabled(True)
            if not self._buf:
                self._flush_timer.stop()

    def _trim_document(self, doc) -> None:
        if not self._max_characters:
            return
        excess = doc.characterCount() - self._max_characters
        if excess <= 0:
            return
        # Drop a little more than needed so the trim does not run every tick,
        # and extend the cut to the next line break (a paragraph separator for
        # plain lines, U+2028 for HTML ``<br>`` lines) so no line is torn.
        last = doc.characterCount() - 1
        end = min(last, excess + self._max_characters // 10)
        while end < last and doc.characterAt(end) not in _LINE_BREAKS:
            end += 1
        cursor = QTextCursor(doc)
        cursor.setPosition(0)
        cursor.setPosition(min(last, end + 1), QTextCursor.KeepAnchor)
        cursor.removeSelectedText()


__all__ = ['LogTextEdit', 'NOTICE_COLOR']
