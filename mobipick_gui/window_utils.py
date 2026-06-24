"""Window helpers shared by the Qt widgets."""
from __future__ import annotations

from PyQt5.QtCore import Qt
from PyQt5.QtWidgets import QDialog as QtDialog
from PyQt5.QtWidgets import QWidget


def configure_maximizable_window(window: QWidget) -> None:
    """Give a top-level window standard minimize/maximize controls."""
    window.setWindowFlag(Qt.WindowMinimizeButtonHint, True)
    window.setWindowFlag(Qt.WindowMaximizeButtonHint, True)
    window.setWindowFlag(Qt.WindowCloseButtonHint, True)
    if isinstance(window, QtDialog):
        window.setSizeGripEnabled(True)


class MaximizableDialog(QtDialog):
    """QDialog variant with normal window-manager maximize affordances."""

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        configure_maximizable_window(self)
