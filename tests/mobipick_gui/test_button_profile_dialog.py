import os
from pathlib import Path

os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')

from PyQt5.QtWidgets import QApplication, QHeaderView

from mobipick_gui.config import save_button_layout
from mobipick_gui.main_window import ButtonProfileDialog, DockerCpConfigDialog


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
