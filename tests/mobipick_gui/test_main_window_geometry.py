import os
from pathlib import Path
from types import MethodType

os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')

from PyQt5.QtCore import Qt
from PyQt5.QtWidgets import QApplication, QMainWindow

import mobipick_gui.main_window as main_window_module
from mobipick_gui.main_window import MainWindow


def test_save_window_state_persists_geometry(monkeypatch):
    saved_updates = []
    monkeypatch.setattr(
        main_window_module,
        'save_user_config_update',
        lambda updates: saved_updates.append(updates),
    )

    app = QApplication.instance() or QApplication([])
    window = QMainWindow()
    window._console_log = MethodType(lambda self, *_args: None, window)
    window.setGeometry(12, 34, 640, 480)

    MainWindow._save_window_state(window)

    assert saved_updates == [
        {
            'window': {
                'geometry': [12, 34, 640, 480],
                'maximized': False,
            },
        },
    ]

    window.deleteLater()
    app.processEvents()


def test_restore_window_state_applies_geometry_and_maximized():
    app = QApplication.instance() or QApplication([])
    window = QMainWindow()

    MainWindow._restore_window_state(
        window,
        {'geometry': ['20', '40', '800', '600'], 'maximized': True},
    )

    assert window.geometry().x() == 20
    assert window.geometry().y() == 40
    assert window.geometry().width() == 800
    assert window.geometry().height() == 600
    assert window.windowState() & Qt.WindowMaximized

    window.deleteLater()
    app.processEvents()


def test_copy_full_reset_command_uses_warning_dialog(monkeypatch):
    app = QApplication.instance() or QApplication([])
    window = QMainWindow()
    clipboard_text = []
    information_calls = []
    created_messages = []

    class Clipboard:
        def setText(self, text):
            clipboard_text.append(text)

    class FakeMessageBox:
        Warning = object()
        AcceptRole = object()
        Cancel = object()

        def __init__(self, parent):
            self.parent = parent
            self.text = ''
            self.informative_text = ''
            self.detailed_text = ''
            self.copy_button = object()
            created_messages.append(self)

        def setIcon(self, icon):
            self.icon = icon

        def setWindowTitle(self, title):
            self.title = title

        def setText(self, text):
            self.text = text

        def setInformativeText(self, text):
            self.informative_text = text

        def addButton(self, *args):
            if args and args[0] == 'Copy Command':
                return self.copy_button
            return object()

        def setDetailedText(self, text):
            self.detailed_text = text

        def exec_(self):
            return None

        def clickedButton(self):
            return self.copy_button

        @staticmethod
        def information(parent, title, text):
            information_calls.append((parent, title, text))

    command = 'printf reset; rm -rf -- /tmp/mobipick-reset-test'
    monkeypatch.setattr(main_window_module, 'QMessageBox', FakeMessageBox)
    monkeypatch.setattr(
        main_window_module.QApplication,
        'clipboard',
        staticmethod(lambda: Clipboard()),
    )
    monkeypatch.setattr(
        main_window_module,
        'user_state_reset_paths',
        lambda: [Path('/tmp/mobipick config'), Path('/tmp/mobipick data')],
    )
    monkeypatch.setattr(
        main_window_module,
        'user_state_reset_command',
        lambda: command,
    )

    MainWindow._copy_full_reset_command(window)

    assert clipboard_text == [command]
    assert information_calls
    assert created_messages[0].parent is window
    assert created_messages[0].detailed_text == command
    assert '/tmp/mobipick config' in created_messages[0].informative_text
    assert (
        'does not delete Docker images'
        in created_messages[0].informative_text
    )

    window.deleteLater()
    app.processEvents()
