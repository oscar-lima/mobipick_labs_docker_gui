import os
from pathlib import Path
from types import MethodType

os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')

from PyQt5.QtCore import Qt
from PyQt5.QtWidgets import QApplication, QMainWindow

import mobipick_gui.main_window as main_window_module
from mobipick_gui.main_window import MainWindow
from mobipick_gui.window_utils import MaximizableDialog


def test_maximizable_dialog_has_standard_window_controls():
    app = QApplication.instance() or QApplication([])
    dialog = MaximizableDialog()

    flags = dialog.windowFlags()

    assert flags & Qt.WindowMinimizeButtonHint
    assert flags & Qt.WindowMaximizeButtonHint
    assert flags & Qt.WindowCloseButtonHint

    dialog.deleteLater()
    app.processEvents()


def test_helper_windows_can_be_maximized(tmp_path):
    app = QApplication.instance() or QApplication([])

    class FakeMainWindow:
        _window_layout_dialog = None
        _recording_window = None
        _recording_path_label = None
        _recording_stop_button = None

        def __init__(self):
            self._window_layout_path = tmp_path / 'window_layout.yaml'

        def _on_save_window_state_clicked(self):
            pass

        def _on_recording_stop_clicked(self):
            pass

    fake = FakeMainWindow()

    layout_dialog = MainWindow._ensure_window_layout_dialog(fake)
    recording_dialog = MainWindow._ensure_recording_window(fake)

    assert layout_dialog.windowFlags() & Qt.WindowMaximizeButtonHint
    assert recording_dialog.windowFlags() & Qt.WindowMaximizeButtonHint
    assert (
        recording_dialog.windowFlags() & Qt.WindowType_Mask
    ) != Qt.Tool
    assert layout_dialog.maximumWidth() > 1000
    assert recording_dialog.maximumWidth() > 1000

    layout_dialog.deleteLater()
    recording_dialog.deleteLater()
    app.processEvents()


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
