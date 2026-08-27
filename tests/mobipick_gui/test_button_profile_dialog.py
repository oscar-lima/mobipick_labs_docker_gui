import os
from pathlib import Path

os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')

from PyQt5.QtCore import Qt
from PyQt5.QtWidgets import (
    QApplication,
    QDialog,
    QHBoxLayout,
    QHeaderView,
    QLineEdit,
    QWIDGETSIZE_MAX,
    QSizePolicy,
    QWidget,
)

from mobipick_gui.config import (
    load_docker_cp_config,
    save_button_layout,
    save_docker_cp_config,
)
import mobipick_gui.main_window as main_window_module
from mobipick_gui.main_window import (
    ButtonProfileDialog,
    DockerCpContainerPathDialog,
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
                'stop_command': 'rosrun demo_package stop_tool --flag value',
                'host': True,
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
    assert widths['stop_command'] > widths['tooltip']
    assert widths['label'] > widths['tooltip']
    assert widths['key'] >= 96

    dialog.deleteLater()
    app.processEvents()


def test_button_profile_dialog_is_a_maximizable_top_level_window(tmp_path):
    app = QApplication.instance() or QApplication([])
    dialog = ButtonProfileDialog(
        [],
        Path(tmp_path / 'source.yaml'),
        Path(tmp_path / 'target.yaml'),
    )

    flags = dialog.windowFlags()

    assert flags & Qt.WindowMaximizeButtonHint
    assert (flags & Qt.WindowType_Mask) == Qt.Window
    assert dialog.maximumWidth() == QWIDGETSIZE_MAX
    assert dialog.maximumHeight() == QWIDGETSIZE_MAX

    dialog.show()
    app.processEvents()
    dialog.showMaximized()
    app.processEvents()

    assert dialog.isMaximized()

    dialog.close()
    dialog.deleteLater()
    app.processEvents()


def test_button_profile_dialog_edits_and_saves_stop_command(tmp_path):
    app = QApplication.instance() or QApplication([])
    target = tmp_path / 'target.yaml'
    dialog = ButtonProfileDialog(
        [
            {
                'key': 'ollama',
                'label': 'Ollama',
                'kind': 'command',
                'command': 'manage_ollama.sh start',
                'stop_command': 'manage_ollama.sh old-stop',
                'host': True,
            },
        ],
        Path(tmp_path / 'source.yaml'),
        target,
    )

    stop_column = next(
        index
        for index, (field, _label) in enumerate(dialog.COLUMNS)
        if field == 'stop_command'
    )
    stop_item = dialog.table.item(0, stop_column)
    assert stop_item.flags() & Qt.ItemIsEditable
    stop_item.setText('manage_ollama.sh stop')

    save_button_layout(target, dialog.button_layout())

    assert 'stop_command: manage_ollama.sh stop' in target.read_text(
        encoding='utf-8'
    )

    dialog.deleteLater()
    app.processEvents()


def test_button_profile_dialog_hides_and_disables_stop_for_non_host(tmp_path):
    app = QApplication.instance() or QApplication([])
    target = tmp_path / 'target.yaml'
    dialog = ButtonProfileDialog(
        [
            {
                'key': 'ollama',
                'label': 'Ollama',
                'kind': 'command',
                'command': 'manage_ollama.sh start',
                'stop_command': 'manage_ollama.sh stop',
                'host': False,
            },
        ],
        Path(tmp_path / 'source.yaml'),
        target,
    )
    stop_column = dialog._field_column('stop_command')
    host_column = dialog._field_column('host')
    stop_item = dialog.table.item(0, stop_column)

    assert dialog.table.isColumnHidden(stop_column)
    assert stop_item.text() == ''
    assert not stop_item.flags() & Qt.ItemIsEditable
    assert dialog.button_layout()[0]['stop_command'] == ''

    dialog.table.item(0, host_column).setCheckState(Qt.Checked)

    assert not dialog.table.isColumnHidden(stop_column)
    assert stop_item.text() == 'manage_ollama.sh stop'
    assert stop_item.flags() & Qt.ItemIsEditable

    dialog.table.item(0, host_column).setCheckState(Qt.Unchecked)
    save_button_layout(target, dialog.button_layout())

    assert dialog.table.isColumnHidden(stop_column)
    assert 'stop_command:' not in target.read_text(encoding='utf-8')

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


def test_button_profile_dialog_saves_host_checkbox(tmp_path):
    app = QApplication.instance() or QApplication([])
    target = tmp_path / 'target.yaml'
    dialog = ButtonProfileDialog(
        [
            {
                'key': 'litellm',
                'label': 'LiteLLM',
                'kind': 'command',
                'command': 'docker run --rm litellm',
                'host': False,
            },
        ],
        Path(tmp_path / 'source.yaml'),
        target,
    )

    host_column = next(
        index
        for index, (field, _label) in enumerate(dialog.COLUMNS)
        if field == 'host'
    )
    host_item = dialog.table.item(0, host_column)
    assert host_item.flags() & Qt.ItemIsUserCheckable
    assert host_item.checkState() == Qt.Unchecked

    host_item.setCheckState(Qt.Checked)
    save_button_layout(target, dialog.button_layout())

    assert 'host: true' in target.read_text(encoding='utf-8')

    dialog.deleteLater()
    app.processEvents()


def test_button_profile_dialog_configures_args_in_popup(monkeypatch, tmp_path):
    app = QApplication.instance() or QApplication([])
    target = tmp_path / 'target.yaml'
    dialog = ButtonProfileDialog(
        [
            {
                'key': 'demo',
                'label': 'Demo',
                'kind': 'command',
                'command': 'roslaunch demo run.launch',
            },
            {
                'key': 'monitor',
                'label': 'Monitor',
                'kind': 'command',
                'command': 'rosrun demo monitor.py',
            },
        ],
        Path(tmp_path / 'source.yaml'),
        target,
    )
    class FakeArgumentsDialog:
        def __init__(self, entry, parent):
            assert entry['key'] == 'demo'
            assert parent is dialog

        def exec_(self):
            return QDialog.Accepted

        def arguments(self):
            return {
                'arg_1_name': 'model_profile',
                'arg_1_options': ['thor', 'panda'],
                'arg_1_applies': True,
                'arg_2_name': '',
                'arg_2_options': [],
                'arg_2_applies': False,
                'arg_3_name': '',
                'arg_3_options': [],
                'arg_3_applies': False,
            }

    monkeypatch.setattr(
        main_window_module,
        'ButtonArgumentsDialog',
        FakeArgumentsDialog,
    )
    assert all(
        not field.startswith('arg_') for field, _label in dialog.COLUMNS
    )
    dialog.table.selectRow(0)
    dialog._configure_selected_arguments()
    other_entry = dialog.button_layout()[1]
    assert other_entry['arg_1_name'] == 'model_profile'
    assert other_entry['arg_1_options'] == ['thor', 'panda']
    assert other_entry['arg_1_applies'] is False
    save_button_layout(target, dialog.button_layout())

    text = target.read_text(encoding='utf-8')
    assert 'arg_1_name: model_profile' in text
    assert 'arg_1_options:' in text
    assert '- thor' in text
    assert '- panda' in text
    assert 'arg_1_applies: true' in text

    dialog.deleteLater()
    app.processEvents()


def test_generic_arg_controls_are_hidden_without_config_and_append_values():
    app = QApplication.instance() or QApplication([])
    harness = QWidget()
    harness.generic_arg_controls = QWidget(harness)
    harness._generic_arg_controls_layout = QHBoxLayout(
        harness.generic_arg_controls
    )
    harness._generic_arg_inputs = {}
    harness._button_layout = []
    harness._refresh_generic_arg_controls = (
        main_window_module.MainWindow._refresh_generic_arg_controls.__get__(
            harness
        )
    )
    harness._refresh_generic_arg_controls()

    assert harness.generic_arg_controls.isHidden()
    assert harness._generic_arg_inputs == {}

    harness._button_layout = [{
        'arg_1_name': 'robot',
        'arg_1_options': ['thor', 'mobipick 1'],
        'arg_1_applies': True,
    }]
    harness._refresh_generic_arg_controls()
    harness._generic_arg_inputs[1].setCurrentText('mobipick 1')
    harness._sh_quote = main_window_module.MainWindow._sh_quote
    command = main_window_module.MainWindow._command_with_generic_args(
        harness,
        'roslaunch demo run.launch',
        {
            'arg_1_name': 'robot',
            'arg_1_options': ['thor', 'mobipick 1'],
            'arg_1_applies': True,
        },
    )

    assert not harness.generic_arg_controls.isHidden()
    assert command == "roslaunch demo run.launch robot:='mobipick 1'"

    harness.deleteLater()
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
    provider_calls = []
    dialog = DockerCpConfigDialog(
        {},
        {},
        'demo_ws',
        ['demo_ws'],
        tmp_path / 'docker_cp.yaml',
        container_options_provider=lambda workspace: (
            provider_calls.append(workspace) or [('match', 'container-id')]
        ),
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
    assert captured['container_options_provider']() == [
        ('match', 'container-id')
    ]
    assert provider_calls == ['demo_ws']

    dialog.deleteLater()
    app.processEvents()


def test_docker_cp_dialog_updates_workspace_config_as_rows_change(tmp_path):
    app = QApplication.instance() or QApplication([])
    dialog = DockerCpConfigDialog(
        {},
        {},
        'clean_mobipick_labs_ws',
        ['clean_mobipick_labs_ws'],
        tmp_path / 'docker_cp.yaml',
    )

    dialog._add_row(
        dialog.host_to_container_table,
        '/host/pick_n_place.rviz',
        '/container/pick_n_place.rviz',
    )

    config = dialog.docker_cp_config()

    assert config['clean_mobipick_labs_ws']['host_to_container'] == [
        {
            'host': '/host/pick_n_place.rviz',
            'container': '/container/pick_n_place.rviz',
        }
    ]

    dialog.deleteLater()
    app.processEvents()


def test_docker_cp_dialog_workspace_config_round_trips_to_yaml(tmp_path):
    app = QApplication.instance() or QApplication([])
    target = tmp_path / 'clean_mobipick_labs_ws_docker_cp_image_tag.yaml'
    dialog = DockerCpConfigDialog(
        {},
        {},
        'clean_mobipick_labs_ws',
        ['clean_mobipick_labs_ws'],
        target,
    )
    dialog._add_row(
        dialog.host_to_container_table,
        '/host/pick_n_place.rviz',
        '/container/pick_n_place.rviz',
    )

    save_docker_cp_config(dialog.docker_cp_config(), target)
    loaded = load_docker_cp_config(target)

    assert loaded['clean_mobipick_labs_ws']['host_to_container'] == [
        {
            'host': '/host/pick_n_place.rviz',
            'container': '/container/pick_n_place.rviz',
        }
    ]

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


def test_docker_cp_path_dialog_expands_long_path_fields():
    app = QApplication.instance() or QApplication([])
    long_container_path = (
        '/root/catkin_ws/src/mobipick_labs/tables_demo_bringup/config/'
        'pick_n_place.rviz'
    )
    dialog = DockerCpPathDialog(
        host_first=True,
        container_path=long_container_path,
    )

    assert dialog.host_path_edit.sizePolicy().horizontalPolicy() == (
        QSizePolicy.Expanding
    )
    assert dialog.container_path_edit.sizePolicy().horizontalPolicy() == (
        QSizePolicy.Expanding
    )
    assert dialog.minimumWidth() >= 760

    dialog.deleteLater()
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
    dialog = DockerCpPathDialog(
        host_first=True,
        container_path='demo_ws/src/mobipick_labs/config.rviz',
        container_options_provider=lambda: [
            ('Workspace match container', 'container-id'),
        ],
        container_path_provider=lambda container, default: '/unused',
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

    assert dialog.container_path_edit.text() == '/unused'

    dialog.deleteLater()
    app.processEvents()


def test_container_path_browser_accepts_nonexistent_manual_path():
    app = QApplication.instance() or QApplication([])
    browser = DockerCpContainerPathDialog(
        container_ref='container-id',
        start_path='/root/catkin_ws/new_file.rviz',
        list_provider=lambda _container, _path: [],
    )
    browser.path_edit.setText('/root/catkin_ws/does_not_exist_yet.rviz')

    assert browser.selected_path() == (
        '/root/catkin_ws/does_not_exist_yet.rviz'
    )

    browser.deleteLater()
    app.processEvents()
