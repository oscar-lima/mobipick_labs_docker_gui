import os
from pathlib import Path

os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')

from PyQt5.QtWidgets import QApplication, QDialog, QHeaderView

from mobipick_gui.config import save_button_layout
import mobipick_gui.main_window as main_window_module
from mobipick_gui.main_window import (
    ButtonProfileDialog,
    DockerCpContainerSelectDialog,
    DockerCpConfigDialog,
    DockerCpPathDialog,
)


def test_button_profile_dialog_prioritizes_command_columns(tmp_path):
    app = QApplication.instance() or QApplication([])
    dialog = ButtonProfileDialog(
        [
            {
                'key': 'demo_tool',
                'label': 'Demo Tool',
                'kind': 'command',
                'command': 'rosrun demo_package very_long_command_name --flag value',
                'tooltip': (
                    'This tooltip is intentionally much longer than the '
                    'other fields and should not dominate the visible width.'
                ),
            },
        ],
        Path(tmp_path / 'source.yaml'),
        Path(tmp_path / 'target.yaml'),
    )
    dialog.table.resize(720, 240)
    dialog.table.apply_column_widths()

    widths = {
        field: dialog.table.columnWidth(index)
        for index, (field, _label) in enumerate(dialog.COLUMNS)
    }

    assert widths['command'] > widths['tooltip']
    assert widths['label'] > widths['tooltip']
    assert widths['key'] >= 96

    dialog.deleteLater()
    app.processEvents()


def test_button_profile_dialog_saves_command_service(tmp_path):
    app = QApplication.instance() or QApplication([])
    target = tmp_path / 'target.yaml'
    dialog = ButtonProfileDialog(
        [
            {
                'key': 'gazebo_launcher',
                'label': 'Gazebo Launcher',
                'kind': 'command',
                'command': 'roslaunch demo gazebo.launch',
                'service': 'mobipick',
            },
        ],
        Path(tmp_path / 'source.yaml'),
        target,
    )

    service_column = next(
        index
        for index, (field, _label) in enumerate(dialog.COLUMNS)
        if field == 'service'
    )
    assert dialog.table.item(0, service_column).text() == 'mobipick'

    save_button_layout(target, dialog.button_layout())

    assert 'service: mobipick' in target.read_text(encoding='utf-8')

    dialog.deleteLater()
    app.processEvents()


def test_docker_cp_dialog_path_columns_stretch_with_window(tmp_path):
    app = QApplication.instance() or QApplication([])
    dialog = DockerCpConfigDialog(
        {
            'default': {
                'host_to_container': [
                    {
                        'host': '~/Downloads/pick_n_place.rviz',
                        'container': '/root/catkin_ws/config/pick_n_place.rviz',
                    }
                ],
                'container_to_host': [],
            }
        },
        {},
        'clean_mobipick_labs_ws',
        ['clean_mobipick_labs_ws'],
        Path(tmp_path / 'docker_cp_image_tag.yaml'),
    )

    header = dialog.host_to_container_table.horizontalHeader()

    assert header.sectionResizeMode(0) == QHeaderView.Stretch
    assert header.sectionResizeMode(1) == QHeaderView.Stretch
    assert not header.stretchLastSection()

    dialog.deleteLater()
    app.processEvents()


def test_docker_cp_dialog_prefers_clean_workspace_default_if_present(
    monkeypatch,
    tmp_path,
):
    preferred_text = (
        'clean_mobipick_labs_ws/'
        + DockerCpConfigDialog.CONTAINER_CONFIG_SUFFIX
    )
    preferred = tmp_path / preferred_text
    preferred.parent.mkdir(parents=True)
    preferred.write_text('rviz config', encoding='utf-8')
    monkeypatch.chdir(tmp_path)

    assert DockerCpConfigDialog._default_container_path(
        'clean_mobipick_labs_ws'
    ) == preferred_text


def test_docker_cp_add_row_opens_path_dialog(monkeypatch, tmp_path):
    app = QApplication.instance() or QApplication([])
    start_path = tmp_path / 'master' / 'demo_ws'
    start_path.mkdir(parents=True)
    captured = {}
    dialog = DockerCpConfigDialog(
        {},
        {},
        'demo_ws',
        ['demo_ws'],
        tmp_path / 'docker_cp.yaml',
        host_start_provider=lambda workspace: start_path,
    )

    class FakePathDialog:
        def __init__(self, *, host_first, container_path, parent, **kwargs):
            self.host_first = host_first
            self.container_path = container_path
            self.parent = parent
            captured.update(kwargs)

        def exec_(self):
            return QDialog.Accepted

        def paths(self):
            return ('/tmp/source.rviz', '/container/source.rviz')

    monkeypatch.setattr(main_window_module, 'DockerCpPathDialog', FakePathDialog)

    dialog._add_row_from_dialog(dialog.host_to_container_table)

    assert dialog.host_to_container_table.item(0, 0).text() == '/tmp/source.rviz'
    assert dialog.host_to_container_table.item(0, 1).text() == (
        '/container/source.rviz'
    )
    assert captured['host_start_path'] == start_path

    dialog.deleteLater()
    app.processEvents()


def test_docker_cp_path_dialog_orders_paths_for_both_directions(tmp_path):
    app = QApplication.instance() or QApplication([])
    host_file = tmp_path / 'pick_n_place.rviz'
    host_file.write_text('rviz config', encoding='utf-8')

    host_to_container = DockerCpPathDialog(
        host_first=True,
        container_path='/container/pick_n_place.rviz',
    )
    host_to_container.host_path_edit.setText(str(host_file))
    assert host_to_container.paths() == (
        str(host_file),
        '/container/pick_n_place.rviz',
    )

    container_to_host = DockerCpPathDialog(
        host_first=False,
        container_path='/container/pick_n_place.rviz',
    )
    container_to_host.host_path_edit.setText(str(host_file))
    assert container_to_host.paths() == (
        '/container/pick_n_place.rviz',
        str(host_file),
    )

    host_to_container.deleteLater()
    container_to_host.deleteLater()
    app.processEvents()


def test_docker_cp_host_browse_prefers_current_then_workspace(tmp_path):
    workspace = tmp_path / 'master' / 'demo_ws'
    workspace.mkdir(parents=True)
    selected_parent = tmp_path / 'selected'
    selected_parent.mkdir()
    selected_file = selected_parent / 'config.rviz'

    assert DockerCpPathDialog._host_browse_start('', workspace) == workspace
    assert DockerCpPathDialog._host_browse_start(
        str(selected_file),
        workspace,
    ) == selected_parent


def test_docker_cp_container_browse_uses_clean_workspace_file(
    monkeypatch,
    tmp_path,
):
    preferred_text = (
        'clean_mobipick_labs_ws/'
        + DockerCpConfigDialog.CONTAINER_CONFIG_SUFFIX
    )
    preferred = tmp_path / preferred_text
    preferred.parent.mkdir(parents=True)
    preferred.write_text('rviz config', encoding='utf-8')
    monkeypatch.chdir(tmp_path)

    assert DockerCpPathDialog._container_browse_start(
        'clean_mobipick_labs_ws'
    ) == str(preferred)
    assert DockerCpPathDialog._container_path_from_selection(preferred) == (
        preferred_text
    )


def test_docker_cp_dialog_lists_workspaces_not_images(tmp_path):
    app = QApplication.instance() or QApplication([])
    dialog = DockerCpConfigDialog(
        {'example/image:tag': {'host_to_container': [], 'container_to_host': []}},
        {},
        'demo_ws',
        ['demo_ws', 'other_ws'],
        tmp_path / 'docker_cp.yaml',
    )

    keys = [
        dialog.profile_combo.itemData(index)
        for index in range(dialog.profile_combo.count())
    ]

    assert keys == ['demo_ws', 'default', 'other_ws']
    assert 'example/image:tag' not in keys
    assert not hasattr(dialog, 'profile_key_input')

    dialog.deleteLater()
    app.processEvents()


def test_docker_cp_path_dialog_can_use_container_provider(monkeypatch):
    app = QApplication.instance() or QApplication([])
    calls = []
    dialog = DockerCpPathDialog(
        host_first=True,
        container_path='demo_ws/src/mobipick_labs/config.rviz',
        container_options_provider=lambda: [
            ('Workspace match container', 'container-id'),
        ],
        container_path_provider=lambda container, default: calls.append(
            (container, default)
        ) or '/container/selected.rviz',
    )
    monkeypatch.setattr(
        DockerCpContainerSelectDialog,
        'exec_',
        lambda self: QDialog.Accepted,
    )
    monkeypatch.setattr(
        DockerCpContainerSelectDialog,
        'container_ref',
        lambda self: 'container-id',
    )

    dialog._browse_container_path()

    assert calls == [
        ('container-id', 'demo_ws/src/mobipick_labs/config.rviz')
    ]
    assert dialog.container_path_edit.text() == '/container/selected.rviz'

    dialog.deleteLater()
    app.processEvents()
