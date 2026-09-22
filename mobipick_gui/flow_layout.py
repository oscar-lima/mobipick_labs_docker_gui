"""A wrapping (flow) layout used by the toolbar rows.

A ``QHBoxLayout`` forces the window to stay at least as wide as the sum of
its children, so a long toolbar row makes the main window impossible to
shrink (or even to fit) on a laptop panel.  ``FlowLayout`` lays its items out
left to right and starts a new line whenever the next item no longer fits, so
the window's minimum width is only as wide as its widest single item.

Items whose size policy expands horizontally still share the leftover space of
their own line, which keeps the wide-monitor appearance of the toolbar
buttons unchanged.
"""
from __future__ import annotations

from PyQt5.QtCore import QRect, QSize, Qt
from PyQt5.QtWidgets import QLayout, QWidget, QWidgetItem


class FlowLayout(QLayout):
    """Left-to-right layout that wraps into additional lines when needed."""

    def __init__(
        self,
        parent: QWidget | None = None,
        *,
        margin: int = 0,
        horizontal_spacing: int = 6,
        vertical_spacing: int = 6,
    ):
        super().__init__(parent)
        self._items: list = []
        self._h_space = max(0, int(horizontal_spacing))
        self._v_space = max(0, int(vertical_spacing))
        self.setContentsMargins(margin, margin, margin, margin)

    # ---------- QLayout API ----------

    def addItem(self, item):  # noqa: N802 - Qt API
        self._items.append(item)

    def insertWidget(self, index: int, widget: QWidget) -> None:  # noqa: N802
        """Insert ``widget`` at ``index`` (``QBoxLayout``-compatible)."""
        self.addChildWidget(widget)
        item = QWidgetItem(widget)
        if index < 0 or index > len(self._items):
            self._items.append(item)
        else:
            self._items.insert(index, item)
        self.invalidate()

    def count(self) -> int:
        return len(self._items)

    def itemAt(self, index: int):  # noqa: N802 - Qt API
        if 0 <= index < len(self._items):
            return self._items[index]
        return None

    def takeAt(self, index: int):  # noqa: N802 - Qt API
        if 0 <= index < len(self._items):
            return self._items.pop(index)
        return None

    def expandingDirections(self):  # noqa: N802 - Qt API
        return Qt.Orientations(Qt.Horizontal)

    def hasHeightForWidth(self) -> bool:  # noqa: N802 - Qt API
        return True

    def heightForWidth(self, width: int) -> int:  # noqa: N802 - Qt API
        return self._lay_out(QRect(0, 0, width, 0), apply_geometry=False)

    def setGeometry(self, rect: QRect) -> None:  # noqa: N802 - Qt API
        super().setGeometry(rect)
        self._lay_out(rect, apply_geometry=True)

    def minimumSize(self) -> QSize:  # noqa: N802 - Qt API
        size = QSize(0, 0)
        for item in self._items:
            size = size.expandedTo(item.minimumSize())
        left, top, right, bottom = self.getContentsMargins()
        return size + QSize(left + right, top + bottom)

    def sizeHint(self) -> QSize:  # noqa: N802 - Qt API
        """Preferred size: every item on a single line."""
        width = 0
        height = 0
        for index, item in enumerate(self._items):
            hint = item.sizeHint()
            width += hint.width() + (self._h_space if index else 0)
            height = max(height, hint.height())
        left, top, right, bottom = self.getContentsMargins()
        return QSize(width + left + right, height + top + bottom)

    # ---------- layout helper ----------

    def _lay_out(self, rect: QRect, *, apply_geometry: bool) -> int:
        """Place the items inside ``rect`` and return the required height."""
        left, top, right, bottom = self.getContentsMargins()
        effective = rect.adjusted(left, top, -right, -bottom)
        available = max(0, effective.width())

        rows: list[list[tuple[object, int, int]]] = []
        current: list[tuple[object, int, int]] = []
        current_width = 0
        for item in self._items:
            hint = item.sizeHint()
            # Never let a single item force the row to overflow: a wide child
            # (for example a nested wrapping row) shrinks to the line width
            # and reports the height it needs there.
            item_width = max(
                item.minimumSize().width(),
                min(hint.width(), available),
            )
            if item.hasHeightForWidth():
                item_height = item.heightForWidth(item_width)
            else:
                item_height = hint.height()
            spacing = self._h_space if current else 0
            if current and current_width + spacing + item_width > available:
                rows.append(current)
                current = []
                current_width = 0
                spacing = 0
            current.append((item, item_width, item_height))
            current_width += spacing + item_width
        if current:
            rows.append(current)

        y = effective.y()
        total_height = 0
        for row_index, row in enumerate(rows):
            row_height = max((height for _, _, height in row), default=0)
            if row_index:
                y += self._v_space
                total_height += self._v_space
            if apply_geometry:
                self._place_row(row, effective.x(), y, available, row_height)
            y += row_height
            total_height += row_height
        return total_height + top + bottom

    def _place_row(
        self,
        row: list[tuple[object, int, int]],
        start_x: int,
        y: int,
        available: int,
        row_height: int,
    ) -> None:
        used = sum(width for _, width, _ in row)
        used += self._h_space * (len(row) - 1)
        extra = max(0, available - used)
        expanding = [
            index
            for index, (item, _, _) in enumerate(row)
            if item.expandingDirections() & Qt.Horizontal
        ]
        # A zero-width expanding item is a deliberate spacer, so let it keep
        # the slack of its line instead of blowing up the widgets beside it.
        spacers = [index for index in expanding if row[index][1] == 0]
        if spacers:
            expanding = spacers
        bonuses = [0] * len(row)
        if expanding and extra:
            share, remainder = divmod(extra, len(expanding))
            for position, index in enumerate(expanding):
                bonuses[index] = share + (1 if position < remainder else 0)

        x = start_x
        for index, (item, width, _height) in enumerate(row):
            item_width = width + bonuses[index]
            item.setGeometry(QRect(x, y, item_width, row_height))
            x += item_width + self._h_space
