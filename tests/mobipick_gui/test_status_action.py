import os

os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')

from PyQt5.QtWidgets import QApplication, QPushButton

from mobipick_gui.config import CONFIG
from mobipick_gui.main_window import MainWindow


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

    tools_menu = next(
        action.menu()
        for action in window.menuBar().actions()
        if action.text() == 'Tools'
    )

    assert any(action.text() == 'Update Status' for action in tools_menu.actions())
    assert 'Update Status' not in {
        button.text()
        for button in window.findChildren(QPushButton)
    }

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
