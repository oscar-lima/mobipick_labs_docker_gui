import os

os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')

from PyQt5.QtCore import QEvent, Qt
from PyQt5.QtGui import QKeySequence
from PyQt5.QtTest import QTest
from PyQt5.QtWidgets import QApplication, QDialog, QPushButton, QToolTip

from mobipick_gui.config import CONFIG
from mobipick_gui.main_window import MainWindow


def _find_menu_action(menu, text):
    for action in menu.actions():
        if action.text() == text:
            return menu, action
        submenu = action.menu()
        if submenu is not None:
            found = _find_menu_action(submenu, text)
            if found is not None:
                return found
    return None


def _top_menu(window, text):
    return next(
        action.menu()
        for action in window.menuBar().actions()
        if action.text() == text
    )


def test_update_status_is_menu_only(tmp_path, monkeypatch):
    monkeypatch.setenv('MOBIPICK_WORKSPACE_CONFIG', str(tmp_path / 'workspaces.yaml'))
    monkeypatch.setattr(
        MainWindow,
        '_discover_filtered_image_records',
        lambda self: ([{'ref': CONFIG['images']['default']}], None),
    )
    monkeypatch.setattr(
        MainWindow,
        'update_sim_status_from_poll',
        lambda self, force=False: None,
    )

    app = QApplication.instance() or QApplication([])
    window = MainWindow(verbosity=1)
    window.poll_timer.stop()
    window._sigint_timer.stop()

    tools_menu = _top_menu(window, 'Tools')

    assert any(action.text() == 'Update Status' for action in tools_menu.actions())
    assert 'Update Status' not in {
        button.text()
        for button in window.findChildren(QPushButton)
    }

    window.deleteLater()
    app.processEvents()


def test_file_quit_action_uses_quit_shortcut(tmp_path, monkeypatch):
    monkeypatch.setenv('MOBIPICK_WORKSPACE_CONFIG', str(tmp_path / 'workspaces.yaml'))
    monkeypatch.setattr(
        MainWindow,
        '_discover_filtered_image_records',
        lambda self: ([{'ref': CONFIG['images']['default']}], None),
    )
    monkeypatch.setattr(
        MainWindow,
        'update_sim_status_from_poll',
        lambda self, force=False: None,
    )

    app = QApplication.instance() or QApplication([])
    window = MainWindow(verbosity=1)
    window.poll_timer.stop()
    window._sigint_timer.stop()
    close_calls = []
    monkeypatch.setattr(
        window,
        '_begin_exit_sequence',
        lambda: close_calls.append(True),
    )

    file_menu = _top_menu(window, 'File')
    match = _find_menu_action(file_menu, 'Quit')

    assert match is not None
    _, action = match
    assert [shortcut.toString() for shortcut in action.shortcuts()] == [
        QKeySequence('Ctrl+Q').toString(),
    ]
    assert action.property('mobipick_menu_tooltip') == (
        'Close the GUI and clean up running containers'
    )

    action.trigger()
    app.processEvents()

    assert close_calls == [True]

    window.deleteLater()
    app.processEvents()


def test_ctrl_w_closes_active_secondary_window(tmp_path, monkeypatch):
    monkeypatch.setenv('MOBIPICK_WORKSPACE_CONFIG', str(tmp_path / 'workspaces.yaml'))
    monkeypatch.setattr(
        MainWindow,
        '_discover_filtered_image_records',
        lambda self: ([{'ref': CONFIG['images']['default']}], None),
    )
    monkeypatch.setattr(
        MainWindow,
        'update_sim_status_from_poll',
        lambda self, force=False: None,
    )

    app = QApplication.instance() or QApplication([])
    window = MainWindow(verbosity=1)
    window.poll_timer.stop()
    window._sigint_timer.stop()
    close_calls = []
    monkeypatch.setattr(
        window,
        '_begin_exit_sequence',
        lambda: close_calls.append(True),
    )
    window.show()

    dialog = QDialog(window)
    dialog.setWindowTitle('Secondary Window')
    dialog.show()
    dialog.activateWindow()
    app.processEvents()

    QTest.keyClick(dialog, Qt.Key_W, Qt.ControlModifier)
    app.processEvents()

    assert not dialog.isVisible()
    assert close_calls == []

    window.deleteLater()
    app.processEvents()


def test_top_menu_actions_have_tooltips(tmp_path, monkeypatch):
    monkeypatch.setenv('MOBIPICK_WORKSPACE_CONFIG', str(tmp_path / 'workspaces.yaml'))
    monkeypatch.setattr(
        MainWindow,
        '_discover_filtered_image_records',
        lambda self: ([{'ref': CONFIG['images']['default']}], None),
    )
    monkeypatch.setattr(
        MainWindow,
        'update_sim_status_from_poll',
        lambda self, force=False: None,
    )

    app = QApplication.instance() or QApplication([])
    window = MainWindow(verbosity=1)
    window.poll_timer.stop()
    window._sigint_timer.stop()

    tools_menu = _top_menu(window, 'Tools')

    expected = {
        'Setup Wizard': (
            'Configure first-run setup, host dependencies, workspaces, '
            'and images'
        ),
        'Build Custom Image': 'Build a host-user development Docker image',
        'Execute Docker cp': 'Copy configured paths from the active container to the host',
        'Window Layout': 'Open helper to save window positions for wmctrl replay',
        'Update Status': 'Refresh Docker container status',
    }
    for text, tooltip in expected.items():
        match = _find_menu_action(tools_menu, text)
        assert match is not None
        menu, action = match
        assert menu.hasMouseTracking()
        assert action.property('mobipick_menu_tooltip') == tooltip
        assert action.toolTip() == tooltip
        assert action.statusTip() == tooltip

    unrequested = [
        'Manage Images',
        'Commit Current Tab',
        'Configure Auto Launch',
    ]
    for text in unrequested:
        match = _find_menu_action(tools_menu, text)
        assert match is not None
        _, action = match
        assert action.property('mobipick_menu_tooltip') is None

    window.deleteLater()
    app.processEvents()


def test_setup_wizard_is_top_level_tools_action(tmp_path, monkeypatch):
    monkeypatch.setenv('MOBIPICK_WORKSPACE_CONFIG', str(tmp_path / 'workspaces.yaml'))
    monkeypatch.setattr(
        MainWindow,
        '_discover_filtered_image_records',
        lambda self: ([{'ref': CONFIG['images']['default']}], None),
    )
    monkeypatch.setattr(
        MainWindow,
        'update_sim_status_from_poll',
        lambda self, force=False: None,
    )

    app = QApplication.instance() or QApplication([])
    window = MainWindow(verbosity=1)
    window.poll_timer.stop()
    window._sigint_timer.stop()

    tools_menu = _top_menu(window, 'Tools')
    docker_menu = next(
        action.menu()
        for action in tools_menu.actions()
        if action.text() == 'Docker'
    )

    assert any(action.text() == 'Setup Wizard' for action in tools_menu.actions())
    assert all(action.text() != 'Setup Wizard' for action in docker_menu.actions())

    window.deleteLater()
    app.processEvents()


def test_window_layout_dialog_is_independent_top_level(tmp_path, monkeypatch):
    monkeypatch.setenv('MOBIPICK_WORKSPACE_CONFIG', str(tmp_path / 'workspaces.yaml'))
    monkeypatch.setattr(
        MainWindow,
        '_discover_filtered_image_records',
        lambda self: ([{'ref': CONFIG['images']['default']}], None),
    )
    monkeypatch.setattr(
        MainWindow,
        'update_sim_status_from_poll',
        lambda self, force=False: None,
    )

    app = QApplication.instance() or QApplication([])
    window = MainWindow(verbosity=1)
    window.poll_timer.stop()
    window._sigint_timer.stop()

    dialog = window._ensure_window_layout_dialog()

    assert dialog.parentWidget() is None
    assert dialog.windowModality() == Qt.NonModal
    assert dialog.windowFlags() & Qt.WindowStaysOnTopHint

    dialog.close()
    window.deleteLater()
    app.processEvents()


def test_window_layout_dialog_closes_after_successful_save(tmp_path, monkeypatch):
    monkeypatch.setenv('MOBIPICK_WORKSPACE_CONFIG', str(tmp_path / 'workspaces.yaml'))
    monkeypatch.setattr(
        MainWindow,
        '_discover_filtered_image_records',
        lambda self: ([{'ref': CONFIG['images']['default']}], None),
    )
    monkeypatch.setattr(
        MainWindow,
        'update_sim_status_from_poll',
        lambda self, force=False: None,
    )

    app = QApplication.instance() or QApplication([])
    window = MainWindow(verbosity=1)
    window.poll_timer.stop()
    window._sigint_timer.stop()
    dialog = window._ensure_window_layout_dialog()
    dialog.show()

    monkeypatch.setattr(
        window._window_layout_manager,
        'capture_and_save',
        lambda exclude_titles=None: True,
    )

    window._on_save_window_state_clicked()
    app.processEvents()

    assert not dialog.isVisible()

    window.deleteLater()
    app.processEvents()


def test_menu_tooltips_hide_when_pointer_leaves(tmp_path, monkeypatch):
    monkeypatch.setenv('MOBIPICK_WORKSPACE_CONFIG', str(tmp_path / 'workspaces.yaml'))
    monkeypatch.setattr(
        MainWindow,
        '_discover_filtered_image_records',
        lambda self: ([{'ref': CONFIG['images']['default']}], None),
    )
    monkeypatch.setattr(
        MainWindow,
        'update_sim_status_from_poll',
        lambda self, force=False: None,
    )

    hide_calls = []
    monkeypatch.setattr(QToolTip, 'hideText', lambda: hide_calls.append(True))

    app = QApplication.instance() or QApplication([])
    window = MainWindow(verbosity=1)
    window.poll_timer.stop()
    window._sigint_timer.stop()

    tools_menu = next(
        action.menu()
        for action in window.menuBar().actions()
        if action.text() == 'Tools'
    )

    window.eventFilter(tools_menu, QEvent(QEvent.Leave))

    assert hide_calls

    window.deleteLater()
    app.processEvents()


def test_rare_controls_are_hidden_behind_view_menu(tmp_path, monkeypatch):
    monkeypatch.setenv('MOBIPICK_WORKSPACE_CONFIG', str(tmp_path / 'workspaces.yaml'))
    monkeypatch.setattr(
        MainWindow,
        '_discover_filtered_image_records',
        lambda self: ([{'ref': CONFIG['images']['default']}], None),
    )
    monkeypatch.setattr(
        MainWindow,
        'update_sim_status_from_poll',
        lambda self, force=False: None,
    )

    app = QApplication.instance() or QApplication([])
    window = MainWindow(verbosity=1)
    window.poll_timer.stop()
    window._sigint_timer.stop()
    window.show()
    app.processEvents()

    view_menu = next(
        action.menu()
        for action in window.menuBar().actions()
        if action.text() == 'View'
    )

    assert not window.recording_controls.isVisible()
    assert not window.script_controls.isVisible()
    assert not window.command_controls.isVisible()
    assert not window.ros_master_controls.isVisible()
    assert 'Refresh Images' not in {
        button.text()
        for button in window.findChildren(QPushButton)
    }
    assert any(action.text() == 'Refresh Images' for action in view_menu.actions())

    recording_action = next(
        action for action in view_menu.actions()
        if action.text() == 'Recording Controls'
    )
    recording_action.setChecked(True)
    app.processEvents()

    assert window.recording_controls.isVisible()
    assert window.record_resolution_combo.isVisible()

    window.deleteLater()
    app.processEvents()
