"""Message boxes that move independently of the main window.

``QMessageBox.warning(self, ...)`` opens a modal box that is transient for its parent: GNOME attaches it to that
window, so dragging the box drags the main window along and the user cannot look behind it. The functions here take
the same arguments, but open the box as a top-level window without a parent and non-modal; the call returns at once.

While a modal dialog is open (a dialog still run with ``exec_()``), Qt blocks every window outside it, so a box
without a parent could not be clicked: the box then opens over that dialog and waits, as before.

A question whose answer the caller needs (``question``, ``ask``) stays modal, since the code waits for the answer,
but it also opens without a parent, so it is a window of its own and not attached to the main window.

The remote control lists the open boxes (``open_boxes``) next to the modal dialog, so ``GET /dialogs`` and
``POST /dialogs/dismiss`` reach them too.
"""
from __future__ import annotations

from PyQt5.QtCore import Qt
from PyQt5.QtWidgets import QApplication, QMessageBox

_open_boxes: list[QMessageBox] = []


def open_boxes() -> list[QMessageBox]:
    """The non-modal boxes still open, oldest first."""
    return list(_open_boxes)


def _forget(box: QMessageBox) -> None:
    if box in _open_boxes:
        _open_boxes.remove(box)


def show_box(icon, title: str, text: str, buttons=QMessageBox.Ok, default_button=QMessageBox.NoButton) -> QMessageBox:
    """Open a message box with ``icon`` that does not block (see the module text) and return it."""
    app = QApplication.instance()
    modal = app.activeModalWidget() if app is not None else None
    box = QMessageBox(icon, title, text, buttons, modal)
    box.setDefaultButton(default_button)
    if modal is not None:
        box.exec_()
        return box
    box.setWindowModality(Qt.NonModal)
    box.setAttribute(Qt.WA_DeleteOnClose)
    _open_boxes.append(box)
    box.finished.connect(lambda _result, box=box: _forget(box))
    box.show()
    return box


def information(parent, title: str, text: str, buttons=QMessageBox.Ok,
                defaultButton=QMessageBox.NoButton) -> QMessageBox:
    """Like ``QMessageBox.information``, but non-modal and without a parent (``parent`` is ignored)."""
    return show_box(QMessageBox.Information, title, text, buttons, defaultButton)


def warning(parent, title: str, text: str, buttons=QMessageBox.Ok,
            defaultButton=QMessageBox.NoButton) -> QMessageBox:
    """Like ``QMessageBox.warning``, but non-modal and without a parent (``parent`` is ignored)."""
    return show_box(QMessageBox.Warning, title, text, buttons, defaultButton)


def critical(parent, title: str, text: str, buttons=QMessageBox.Ok,
             defaultButton=QMessageBox.NoButton) -> QMessageBox:
    """Like ``QMessageBox.critical``, but non-modal and without a parent (``parent`` is ignored)."""
    return show_box(QMessageBox.Critical, title, text, buttons, defaultButton)


def ask(icon, title: str, text: str, buttons=QMessageBox.Yes | QMessageBox.No,
        defaultButton=QMessageBox.NoButton) -> int:
    """Ask modally in a box without a parent and return the standard button that answered it."""
    box = QMessageBox(icon, title, text, buttons, None)
    box.setDefaultButton(defaultButton)
    return box.exec_()


def question(parent, title: str, text: str, buttons=QMessageBox.Yes | QMessageBox.No,
             defaultButton=QMessageBox.NoButton) -> int:
    """Like ``QMessageBox.question``, modal but without a parent (``parent`` is ignored)."""
    return ask(QMessageBox.Question, title, text, buttons, defaultButton)


__all__ = ['ask', 'critical', 'information', 'open_boxes', 'question', 'show_box', 'warning']
