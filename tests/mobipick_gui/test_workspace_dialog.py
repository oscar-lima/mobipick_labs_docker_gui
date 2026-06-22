import os

os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')

from PyQt5.QtWidgets import QApplication, QHeaderView

from mobipick_gui.workspace_dialog import WorkspaceManagerDialog
from mobipick_gui.workspaces import (
    IMAGE_WORKSPACE_EXTENDS,
    RosWorkspace,
    WorkspaceRegistry,
)


def _dialog_with_workspace(tmp_path):
    app = QApplication.instance() or QApplication([])
    workspace_path = tmp_path / 'ros_ws' / 'demo_ws'
    (workspace_path / 'src').mkdir(parents=True)
    registry = WorkspaceRegistry(tmp_path / 'workspaces.yaml')
    registry.upsert(
        RosWorkspace(
            name='demo_ws',
            path=str(workspace_path),
            image='example/existing:image',
            button_config=str(tmp_path / 'buttons.yaml'),
            launch_config=str(tmp_path / 'launch.yaml'),
        )
    )
    dialog = WorkspaceManagerDialog(
        registry,
        image_choices=['example/discovered:image'],
    )
    return app, dialog, registry


def test_workspace_dialog_offers_known_combo_values(tmp_path):
    app, dialog, _registry = _dialog_with_workspace(tmp_path)

    assert dialog.image_edit.findData('example/discovered:image') >= 0
    assert dialog.image_edit.findData('example/existing:image') < 0
    assert dialog.image_edit.currentText() == 'example/existing:image'
    assert dialog.button_config_edit.findData(str(tmp_path / 'buttons.yaml')) >= 0
    assert dialog.launch_config_edit.findData(str(tmp_path / 'launch.yaml')) >= 0

    dialog.deleteLater()
    app.processEvents()


def test_workspace_dialog_path_column_stretches_with_window(tmp_path):
    app, dialog, _registry = _dialog_with_workspace(tmp_path)
    header = dialog.tree.header()

    assert header.sectionResizeMode(0) == QHeaderView.ResizeToContents
    assert header.sectionResizeMode(1) == QHeaderView.Stretch
    assert header.sectionResizeMode(4) == QHeaderView.ResizeToContents

    dialog.deleteLater()
    app.processEvents()


def test_workspace_dialog_preserves_custom_combo_text_on_save(tmp_path):
    app, dialog, registry = _dialog_with_workspace(tmp_path)

    dialog.image_edit.setEditText('example/custom:image')
    dialog.button_config_edit.setEditText(str(tmp_path / 'custom_buttons.yaml'))
    dialog.launch_config_edit.setEditText(str(tmp_path / 'custom_launch.yaml'))
    dialog._save_selected()

    workspace = registry.get('demo_ws')
    assert workspace.image == 'example/custom:image'
    assert workspace.button_config == str(tmp_path / 'custom_buttons.yaml')
    assert workspace.launch_config == str(tmp_path / 'custom_launch.yaml')

    dialog.deleteLater()
    app.processEvents()


def test_workspace_dialog_build_cancel_blocks_dirty_workspace(tmp_path):
    app, dialog, _registry = _dialog_with_workspace(tmp_path)
    emitted = []
    dialog.build_requested.connect(emitted.append)

    dialog.sim_command_edit.setText('roslaunch changed sim.launch')
    dialog._resolve_unsaved_changes_before_build = lambda: False
    dialog._build_selected()

    assert emitted == []
    assert dialog._editor_is_dirty()

    dialog.deleteLater()
    app.processEvents()


def test_workspace_dialog_build_can_save_dirty_workspace(tmp_path):
    app, dialog, registry = _dialog_with_workspace(tmp_path)
    emitted = []
    dialog.build_requested.connect(emitted.append)

    dialog.sim_command_edit.setText('roslaunch changed sim.launch')
    dialog._resolve_unsaved_changes_before_build = dialog._save_selected
    dialog._build_selected()

    assert emitted == ['demo_ws']
    assert registry.get('demo_ws').sim_command == 'roslaunch changed sim.launch'
    assert not dialog._editor_is_dirty()

    dialog.deleteLater()
    app.processEvents()


def test_workspace_dialog_build_can_discard_dirty_workspace(tmp_path):
    app, dialog, registry = _dialog_with_workspace(tmp_path)
    emitted = []
    dialog.build_requested.connect(emitted.append)

    dialog.sim_command_edit.setText('roslaunch changed sim.launch')

    def _discard():
        dialog._load_selected()
        return True

    dialog._resolve_unsaved_changes_before_build = _discard
    dialog._build_selected()

    assert emitted == ['demo_ws']
    assert registry.get('demo_ws').sim_command == ''
    assert dialog.sim_command_edit.text() == ''
    assert not dialog._editor_is_dirty()

    dialog.deleteLater()
    app.processEvents()


def test_workspace_dialog_does_not_show_noetic_as_unbuilt_extends(tmp_path):
    app, dialog, _registry = _dialog_with_workspace(tmp_path)

    item = dialog.tree.currentItem()

    assert item.text(3) == ''
    assert dialog.extends_combo.currentData() == ''
    assert dialog.extends_combo.currentText() == 'ROS noetic only (/opt/ros/noetic)'

    dialog.deleteLater()
    app.processEvents()


def test_workspace_dialog_extends_combo_lists_only_built_workspaces(tmp_path):
    app = QApplication.instance() or QApplication([])
    base_path = tmp_path / 'ros_ws' / 'base_ws'
    base_setup = base_path / 'devel' / 'setup.bash'
    base_setup.parent.mkdir(parents=True)
    base_setup.write_text('# generated by catkin\n', encoding='utf-8')
    workspace_path = tmp_path / 'ros_ws' / 'overlay_ws'
    (workspace_path / 'src').mkdir(parents=True)
    unbuilt_path = tmp_path / 'ros_ws' / 'unbuilt_ws'
    (unbuilt_path / 'src').mkdir(parents=True)
    child_path = tmp_path / 'ros_ws' / 'child_ws'
    child_setup = child_path / 'devel' / 'setup.bash'
    child_setup.parent.mkdir(parents=True)
    child_setup.write_text('# generated by catkin\n', encoding='utf-8')
    registry = WorkspaceRegistry(tmp_path / 'workspaces.yaml')
    registry.upsert(
        RosWorkspace(
            name='base_ws',
            path=str(base_path),
        )
    )
    registry.upsert(RosWorkspace(name='unbuilt_ws', path=str(unbuilt_path)))
    registry.upsert(
        RosWorkspace(
            name='overlay_ws',
            path=str(workspace_path),
            extends=['base_ws'],
        )
    )
    registry.upsert(
        RosWorkspace(
            name='child_ws',
            path=str(child_path),
            extends=['overlay_ws'],
        )
    )

    dialog = WorkspaceManagerDialog(registry)
    dialog.refresh('overlay_ws')

    assert not dialog.extends_combo.isEditable()
    assert dialog.extends_combo.findData(IMAGE_WORKSPACE_EXTENDS) >= 0
    assert dialog.extends_combo.findData('base_ws') >= 0
    assert dialog.extends_combo.findData('unbuilt_ws') < 0
    assert dialog.extends_combo.findData('overlay_ws') < 0
    assert dialog.extends_combo.findData('child_ws') < 0
    assert dialog.extends_combo.currentData() == 'base_ws'
    dialog.extends_combo.setCurrentIndex(dialog.extends_combo.findData(''))
    dialog._save_selected()
    assert registry.get('overlay_ws').extends == []

    dialog.extends_combo.setCurrentIndex(
        dialog.extends_combo.findData(IMAGE_WORKSPACE_EXTENDS)
    )
    dialog._save_selected()
    assert registry.get('overlay_ws').extends == [IMAGE_WORKSPACE_EXTENDS]

    dialog.deleteLater()
    app.processEvents()
