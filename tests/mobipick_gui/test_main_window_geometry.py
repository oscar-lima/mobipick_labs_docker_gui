import os
from pathlib import Path
from types import MethodType

os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')

import pytest
from PyQt5.QtCore import Qt
from PyQt5.QtWidgets import (
    QApplication,
    QComboBox,
    QMainWindow,
    QPushButton,
    QSizePolicy,
)

import mobipick_gui.main_window as main_window_module
import mobipick_gui.window_utils as window_utils_module
from mobipick_gui import window_control
from mobipick_gui.main_window import (
    MainWindow,
    _configure_expanding_toolbar_button,
    _configure_shrinkable_combo,
)
from mobipick_gui.flow_layout import FlowLayout
from mobipick_gui.window_utils import (
    MaximizableDialog,
    maximize_after_window_is_exposed,
    restore_window_geometry,
    saved_window_state,
)


def test_maximizable_dialog_has_standard_window_controls():
    app = QApplication.instance() or QApplication([])
    dialog = MaximizableDialog()

    flags = dialog.windowFlags()

    assert flags & Qt.WindowMinimizeButtonHint
    assert flags & Qt.WindowMaximizeButtonHint
    assert flags & Qt.WindowCloseButtonHint

    dialog.deleteLater()
    app.processEvents()


def test_dialog_restores_and_saves_its_own_window_state(monkeypatch):
    app = QApplication.instance() or QApplication([])
    key = 'dialog.Configure Toolbar Buttons'
    saved_updates = []
    monkeypatch.setitem(
        window_utils_module.CONFIG,
        'window_states',
        {key: {'geometry': [30, 40, 720, 510], 'maximized': False}},
    )
    monkeypatch.setattr(
        window_utils_module,
        'save_user_config_update',
        lambda updates: saved_updates.append(updates),
    )
    monkeypatch.setenv('XDG_SESSION_TYPE', 'x11')

    dialog = MaximizableDialog()
    dialog.setWindowTitle('Configure Toolbar Buttons')
    dialog.show()
    app.processEvents()

    assert dialog.geometry().getRect() == (30, 40, 720, 510)

    dialog.setGeometry(50, 60, 800, 600)
    dialog.hide()
    app.processEvents()

    assert saved_updates[-1] == {
        'window_states': {
            key: {
                'geometry': [50, 60, 800, 600],
                'maximized': False,
            },
        },
    }
    dialog.deleteLater()
    app.processEvents()


def test_wayland_restores_size_without_requesting_window_position():
    app = QApplication.instance() or QApplication([])

    class RecordingWindow(QMainWindow):
        def __init__(self):
            super().__init__()
            self.set_geometry_calls = []

        def setGeometry(self, *args):  # noqa: N802 - Qt API
            self.set_geometry_calls.append(args)
            super().setGeometry(*args)

    window = RecordingWindow()
    window.setGeometry(1, 2, 300, 200)
    window.set_geometry_calls.clear()

    maximized = restore_window_geometry(
        window,
        {'geometry': [30, 40, 720, 510], 'maximized': True},
        desktop_session='wayland',
    )

    assert window.set_geometry_calls == []
    assert window.size().width() == 720
    assert window.size().height() == 510
    assert maximized is True

    state = saved_window_state(
        window,
        {'geometry': [30, 40, 700, 500]},
        desktop_session='wayland',
    )
    assert state['geometry'][:2] == [30, 40]

    window.deleteLater()
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

        def _on_recording_pause_clicked(self):
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


def test_save_window_state_persists_combo_selections(monkeypatch):
    saved_updates = []
    monkeypatch.setattr(
        main_window_module,
        'save_user_config_update',
        lambda updates: saved_updates.append(updates),
    )

    app = QApplication.instance() or QApplication([])
    window = QMainWindow()
    window._console_log = MethodType(lambda self, *_args: None, window)
    window._selected_image = 'example/mobipick:dev'
    window._selected_world = 'lab'
    window.script_combo = QComboBox(window)
    window.script_combo.addItems(['first.py', 'chosen.py'])
    window.script_combo.setCurrentText('chosen.py')
    window.record_resolution_combo = QComboBox(window)
    window.record_resolution_combo.addItems(['1920x1080', '2560x1440'])
    window.record_resolution_combo.setCurrentText('2560x1440')
    argument = QComboBox(window)
    argument.addItems(['thor', 'mobipick'])
    argument.setCurrentText('mobipick')
    window._generic_arg_inputs = {1: argument}
    window._generic_arg_names_by_slot = {1: 'robot'}

    window._ui_selection_state = MethodType(
        MainWindow._ui_selection_state,
        window,
    )
    MainWindow._save_window_state(window)

    assert saved_updates[0]['selections'] == {
        'image': 'example/mobipick:dev',
        'world': 'lab',
        'script': 'chosen.py',
        'recording_resolution': '2560x1440',
        'generic_args': {
            '1': {'name': 'robot', 'value': 'mobipick'},
        },
    }

    window.deleteLater()
    app.processEvents()


def test_restore_window_state_applies_geometry_and_defers_maximized():
    app = QApplication.instance() or QApplication([])
    window = QMainWindow()

    MainWindow._restore_window_state(
        window,
        {'geometry': ['20', '40', '700', '500'], 'maximized': True},
    )

    assert window.geometry().x() == 20
    assert window.geometry().y() == 40
    assert window.geometry().width() == 700
    assert window.geometry().height() == 500
    # Maximizing is deferred until the window is mapped; requesting it before
    # the map is ignored by Mutter on X11.
    assert not window.windowState() & Qt.WindowMaximized
    assert window._restore_maximized is True

    window.deleteLater()
    app.processEvents()


@pytest.mark.parametrize(
    ('maximized', 'expected_calls'),
    [(False, ['show']), (True, ['show', 'showMaximized'])],
)
def test_show_with_restored_state_maximizes_after_the_window_is_mapped(
    maximized,
    expected_calls,
):
    app = QApplication.instance() or QApplication([])

    class RecordingWindow(QMainWindow):
        def __init__(self):
            super().__init__()
            self.calls = []

        def show(self):
            self.calls.append('show')
            super().show()

        def showMaximized(self):  # noqa: N802 - Qt API
            self.calls.append('showMaximized')
            super().showMaximized()

    window = RecordingWindow()
    window._restore_maximized = maximized

    MainWindow.show_with_restored_state(window)

    # The maximize request must not be issued synchronously with show(): the
    # window manager only honours it once the window has been mapped.
    assert window.calls == ['show']
    app.processEvents()
    app.processEvents()
    assert window.calls == expected_calls
    window.deleteLater()
    app.processEvents()


def test_show_with_restored_state_keeps_saved_normal_geometry_when_maximized():
    app = QApplication.instance() or QApplication([])
    window = QMainWindow()
    MainWindow._restore_window_state(
        window,
        {'geometry': [20, 40, 700, 500], 'maximized': True},
    )

    MainWindow.show_with_restored_state(window)
    app.processEvents()
    app.processEvents()

    assert window.windowState() & Qt.WindowMaximized
    normal = window.normalGeometry()
    assert (normal.x(), normal.y(), normal.width(), normal.height()) == (
        20, 40, 700, 500,
    )
    window.close()
    window.deleteLater()
    app.processEvents()


@pytest.mark.parametrize('desktop_session', ['x11', 'wayland'])
def test_maximize_restore_waits_for_native_window_exposure(desktop_session):
    app = QApplication.instance() or QApplication([])

    class RecordingWindow(QMainWindow):
        def __init__(self):
            super().__init__()
            self.maximize_calls = 0

        def showMaximized(self):  # noqa: N802 - Qt API
            self.maximize_calls += 1
            super().showMaximized()

    window = RecordingWindow()
    restore_window_geometry(
        window,
        {'geometry': [20, 40, 700, 500], 'maximized': True},
        desktop_session=desktop_session,
    )
    window.show()
    maximize_after_window_is_exposed(window)

    # show() only requests mapping; no maximize request should race ahead of
    # the native expose event from X11 or Wayland.
    assert window.maximize_calls == 0
    app.processEvents()
    app.processEvents()

    assert window.maximize_calls == 1
    assert window.isMaximized()
    window.close()
    window.deleteLater()
    app.processEvents()


@pytest.mark.parametrize('desktop_session', ['x11', 'wayland'])
def test_roscore_transient_text_does_not_increase_button_width(
    monkeypatch,
    desktop_session,
):
    monkeypatch.setenv('XDG_SESSION_TYPE', desktop_session)
    assert window_control.session_type() == desktop_session

    app = QApplication.instance() or QApplication([])
    window = QMainWindow()
    button = QPushButton('Start Roscore', window)
    _configure_expanding_toolbar_button(button)
    assert button.sizePolicy().horizontalPolicy() == QSizePolicy.Expanding
    start_width = button.sizeHint().width()

    button.setText('Starting...')
    assert button.sizeHint().width() <= start_width

    button.setText('Stop Roscore')
    stop_width = button.sizeHint().width()
    button.setText('Stopping...')
    assert button.sizeHint().width() <= stop_width

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


def test_geometry_saved_on_a_larger_monitor_is_fitted_to_the_screen():
    app = QApplication.instance() or QApplication([])
    window = QMainWindow()
    bounds = app.primaryScreen().availableGeometry()

    MainWindow._restore_window_state(
        window,
        {
            'geometry': [
                bounds.width() + 400,
                bounds.height() + 300,
                bounds.width() + 1200,
                bounds.height() + 800,
            ],
            'maximized': False,
        },
    )

    geometry = window.geometry()
    assert geometry.width() == bounds.width()
    assert geometry.height() == bounds.height()
    assert geometry.x() == bounds.x()
    assert geometry.y() == bounds.y()

    window.deleteLater()
    app.processEvents()


def test_oversized_geometry_is_shrunk_on_wayland_without_a_move_request():
    app = QApplication.instance() or QApplication([])
    bounds = app.primaryScreen().availableGeometry()

    class RecordingWindow(QMainWindow):
        def __init__(self):
            super().__init__()
            self.set_geometry_calls = []

        def setGeometry(self, *args):  # noqa: N802 - Qt API
            self.set_geometry_calls.append(args)
            super().setGeometry(*args)

    window = RecordingWindow()
    window.set_geometry_calls.clear()

    restore_window_geometry(
        window,
        {
            'geometry': [0, 0, bounds.width() * 2, bounds.height() * 2],
            'maximized': False,
        },
        desktop_session='wayland',
    )

    assert window.set_geometry_calls == []
    assert window.width() == bounds.width()
    assert window.height() == bounds.height()

    window.deleteLater()
    app.processEvents()


def test_toolbar_rows_wrap_so_the_window_fits_a_laptop_screen(
    tmp_path,
    monkeypatch,
):
    """The toolbar buttons must not pin the window to a huge minimum width."""
    monkeypatch.setenv(
        'MOBIPICK_WORKSPACE_CONFIG',
        str(tmp_path / 'workspaces.yaml'),
    )
    monkeypatch.setattr(
        MainWindow,
        '_discover_filtered_image_records',
        lambda self: ([{'ref': main_window_module.CONFIG['images']['default']}], None),
    )
    monkeypatch.setattr(
        MainWindow,
        'update_sim_status_from_poll',
        lambda self, force=False: None,
    )
    monkeypatch.setitem(
        main_window_module.CONFIG['remote_control'],
        'enabled',
        False,
    )

    app = QApplication.instance() or QApplication([])
    window = MainWindow(verbosity=0)
    window.poll_timer.stop()
    window._sigint_timer.stop()

    assert isinstance(window._top_controls_layout, FlowLayout)
    assert isinstance(window._generic_arg_controls_layout, FlowLayout)
    # Every configured toolbar button plus the terminal controls live in the
    # wrapping row, so the window stays narrower than a small laptop panel.
    single_row = window._top_controls_layout.sizeHint().width()
    minimum = window._top_controls_layout.minimumSize().width()
    assert minimum < single_row
    assert window.minimumSizeHint().width() <= 800

    window.deleteLater()
    app.processEvents()


def test_long_entries_do_not_pin_a_combo_to_a_huge_minimum_width():
    app = QApplication.instance() or QApplication([])
    combo = QComboBox()
    combo.addItem(
        'ozkrelo/x_mobipick_labs:local_gpt_from_oscar_user  [workspace match]'
    )
    natural = combo.sizeHint().width()

    _configure_shrinkable_combo(combo, minimum_chars=16)

    assert combo.minimumSizeHint().width() < natural
    assert combo.sizePolicy().horizontalPolicy() == QSizePolicy.Expanding

    combo.deleteLater()
    app.processEvents()
