import os
from pathlib import Path

os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')

from PyQt5.QtWidgets import QApplication, QDialog, QHeaderView

from mobipick_gui.config import save_button_layout
import mobipick_gui.main_window as main_window_module
from mobipick_gui.main_window import (
    ButtonProfileDialog,
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
        'example/image:tag',
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
    preferred = tmp_path / DockerCpConfigDialog.PREFERRED_CONTAINER_PATH
    preferred.parent.mkdir(parents=True)
    preferred.write_text('rviz config', encoding='utf-8')
    monkeypatch.chdir(tmp_path)

    assert DockerCpConfigDialog._default_container_path() == (
        DockerCpConfigDialog.PREFERRED_CONTAINER_PATH
    )


def test_docker_cp_add_row_opens_path_dialog(monkeypatch, tmp_path):
    app = QApplication.instance() or QApplication([])
    dialog = DockerCpConfigDialog({}, {}, '', tmp_path / 'docker_cp.yaml')

    class FakePathDialog:
        def __init__(self, *, host_first, container_path, parent):
            self.host_first = host_first
            self.container_path = container_path
            self.parent = parent

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


def test_docker_cp_container_browse_uses_clean_workspace_file(
    monkeypatch,
    tmp_path,
):
    preferred = tmp_path / DockerCpConfigDialog.PREFERRED_CONTAINER_PATH
    preferred.parent.mkdir(parents=True)
    preferred.write_text('rviz config', encoding='utf-8')
    monkeypatch.chdir(tmp_path)

    assert DockerCpPathDialog._container_browse_start() == str(preferred)
    assert DockerCpPathDialog._container_path_from_selection(preferred) == (
        DockerCpConfigDialog.PREFERRED_CONTAINER_PATH
    )
