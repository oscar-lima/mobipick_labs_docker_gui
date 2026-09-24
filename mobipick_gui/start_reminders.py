"""One window for the option rules' start reminders.

Starting several buttons (e.g. during Auto Launch) can raise several
``remind_start`` notices. They are collected in a single non-modal window
instead of one popup each: every notice has its own button that copies its
clipboard text, and Close dismisses the window without copying anything (the
reminded process may already be running).
"""

from __future__ import annotations

from PyQt5.QtCore import Qt
from PyQt5.QtWidgets import (
    QApplication,
    QDialog,
    QDialogButtonBox,
    QHBoxLayout,
    QLabel,
    QPushButton,
    QSizePolicy,
    QVBoxLayout,
    QWidget,
)


class StartReminderDialog(QDialog):
    """Non-modal list of start reminders, each with an optional copy button."""

    def __init__(self, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self.setWindowTitle('Start reminders')
        self.setModal(False)
        self.setAttribute(Qt.WA_DeleteOnClose)
        self.setMinimumWidth(520)
        self._items: list[tuple[str, str, str]] = []
        self._copy_buttons: dict[str, QPushButton] = {}
        layout = QVBoxLayout(self)
        intro = QLabel('Review these reminders for the buttons you started:')
        intro.setWordWrap(True)
        layout.addWidget(intro)
        self._rows = QVBoxLayout()
        layout.addLayout(self._rows)
        buttons = QDialogButtonBox(QDialogButtonBox.Close)
        buttons.rejected.connect(self.close)
        layout.addWidget(buttons)

    def items(self) -> list[tuple[str, str, str]]:
        """``(button label, notice, clipboard)`` of every reminder shown."""
        return list(self._items)

    def copy_button(self, clipboard: str) -> QPushButton | None:
        return self._copy_buttons.get(clipboard)

    def add(self, label: str, notice: str, clipboard: str = '') -> bool:
        """Add a reminder unless the same one is already shown; returns whether it was added."""
        if any(item[1:] == (notice, clipboard) for item in self._items):
            return False
        self._items.append((label, notice, clipboard))
        row = QHBoxLayout()
        text = QLabel(f'<b>{label}</b>: {notice}')
        text.setWordWrap(True)
        text.setTextFormat(Qt.RichText)
        text.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Preferred)
        row.addWidget(text, 1)
        if clipboard:
            button = QPushButton(f'Copy "{clipboard}"')
            button.setToolTip(f'Copy {clipboard} to the clipboard')
            button.clicked.connect(lambda _checked=False, value=clipboard: self._copy(value))
            self._copy_buttons[clipboard] = button
            row.addWidget(button, 0, Qt.AlignTop)
        self._rows.addLayout(row)
        self.adjustSize()
        return True

    def _copy(self, value: str) -> None:
        QApplication.clipboard().setText(value)
        button = self._copy_buttons.get(value)
        if button is not None:
            button.setText(f'Copied "{value}"')
