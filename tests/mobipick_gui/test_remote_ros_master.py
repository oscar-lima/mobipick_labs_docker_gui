import os
from pathlib import Path

import yaml

os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')

from PyQt5.QtWidgets import QApplication

from mobipick_gui.config import CONFIG
from mobipick_gui.main_window import MainWindow
from mobipick_gui.workspaces import RosWorkspace


def _create_window(monkeypatch, tmp_path):
    images = [{'ref': 'ozkrelo/mobipick_labs:noetic'}]
    monkeypatch.setenv(
        'MOBIPICK_WORKSPACE_CONFIG',
        str(tmp_path / 'workspaces.yaml'),
    )
    monkeypatch.setattr(
        MainWindow,
        '_discover_filtered_image_records',
        lambda self: (images, None),
    )
    monkeypatch.setattr(
        MainWindow,
        'update_sim_status_from_poll',
        lambda self, force=False: None,
    )
    monkeypatch.setitem(
        CONFIG,
        'ros',
        {
            'remote_master_uri': 'http://mobipick-os-sensor:11311',
            'remote_service': 'mobipick_remote_cmd',
            'remote_enabled_by_default': False,
        },
    )
    app = QApplication.instance() or QApplication([])
    window = MainWindow(verbosity=1)
    window.poll_timer.stop()
    window._sigint_timer.stop()
    return app, window


def test_remote_master_uri_normalization():
    assert MainWindow._normalize_ros_master_uri(
        'mobipick-os-sensor'
    ) == 'http://mobipick-os-sensor:11311'
    assert MainWindow._normalize_ros_master_uri(
        'http://mobipick-os-sensor:2244/'
    ) == 'http://mobipick-os-sensor:2244'
    assert not MainWindow._normalize_ros_master_uri(
        'https://mobipick-os-sensor:11311'
    )
    assert not MainWindow._normalize_ros_master_uri(
        'http://mobipick-os-sensor:invalid'
    )


def test_host_to_container_copy_logs_each_queued_path():
    window = MainWindow.__new__(MainWindow)
    logs = []
    window._append_gui_html = lambda key, message: logs.append((key, message))

    window._log_host_to_container_commands(
        'sim',
        [
            [
                'docker',
                'cp',
                '/tmp/host file.txt',
                'container-id:/opt/mobipick/host file.txt',
            ],
            [
                'docker',
                'cp',
                '/tmp/needs&escape',
                'container-id:/opt/needs&escape',
            ],
        ],
    )

    assert logs == [
        (
            'sim',
            (
                '<i>Copying configured path from host at /tmp/host file.txt '
                '-&gt; container at container-id:/opt/mobipick/host file.txt</i>'
            ),
        ),
        (
            'sim',
            (
                '<i>Copying configured path from host at /tmp/needs&amp;escape '
                '-&gt; container at container-id:/opt/needs&amp;escape</i>'
            ),
        ),
    ]


def test_docker_cp_entries_use_active_workspace_profile(tmp_path):
    window = MainWindow.__new__(MainWindow)
    active = RosWorkspace(name='gpt_ws', path=str(tmp_path / 'gpt_ws'))
    window._workspace_registry = type(
        'Registry',
        (),
        {'active_workspace': lambda self: active},
    )()
    window._selected_image = 'repo/image:tag'
    window._docker_cp_config = {
        'default': {
            'host_to_container': [
                {
                    'host': '~/Downloads/default.rviz',
                    'container': '/container/default.rviz',
                }
            ],
            'container_to_host': [],
        },
        'gpt_ws': {
            'host_to_container': [
                {
                    'host': '~/Downloads/workspace.rviz',
                    'container': '/container/workspace.rviz',
                }
            ],
            'container_to_host': [],
        },
        'repo/image:tag': {
            'host_to_container': [
                {
                    'host': '~/Downloads/image.rviz',
                    'container': '/container/image.rviz',
                }
            ],
            'container_to_host': [],
        },
    }

    assert window._workspace_docker_cp_config_path().name == (
        'gpt_ws_docker_cp_image_tag.yaml'
    )
    assert window._docker_cp_entries('host_to_container') == [
        {
            'host': '~/Downloads/workspace.rviz',
            'container': '/container/workspace.rviz',
        },
    ]


def test_remote_mode_injects_master_and_disables_local_stack(
    tmp_path,
    monkeypatch,
):
    app, window = _create_window(monkeypatch, tmp_path)

    window.remote_master_checkbox.setChecked(True)

    assert window._remote_master_enabled()
    assert window._current_master_uri() == (
        'http://mobipick-os-sensor:11311'
    )
    assert window._ros_tool_service() == 'mobipick_remote_cmd'
    assert not window.roscore_button.isEnabled()
    assert not window._get_button_widget('sim').isEnabled()
    env_args = window._compose_env_args()
    assert 'ROS_MASTER_URI=http://mobipick-os-sensor:11311' in env_args

    window.deleteLater()
    app.processEvents()


def test_rviz_uses_remote_service_without_starting_local_roscore(
    tmp_path,
    monkeypatch,
):
    app, window = _create_window(monkeypatch, tmp_path)
    window.remote_master_checkbox.setChecked(True)
    started = {}
    rviz_tab = window.tasks['rviz']

    monkeypatch.setattr(window, '_claim_xhost', lambda *args, **kwargs: None)
    monkeypatch.setattr(
        window,
        '_confirm_workspace_mismatch_warning',
        lambda *args, **kwargs: True,
    )
    monkeypatch.setattr(
        window,
        '_schedule_host_to_container_copy',
        lambda *args, **kwargs: None,
    )
    monkeypatch.setattr(
        window,
        'bring_up_roscore',
        lambda: (_ for _ in ()).throw(
            AssertionError('local roscore must not start')
        ),
    )
    monkeypatch.setattr(
        rviz_tab,
        'start_program',
        lambda program, args: started.update(
            program=program,
            args=args,
        ),
    )

    window.open_rviz()

    assert started['program'] == 'docker'
    assert 'mobipick_remote_cmd' in started['args']
    assert (
        'ROS_MASTER_URI=http://mobipick-os-sensor:11311'
        in started['args']
    )

    window.deleteLater()
    app.processEvents()


def test_configured_command_service_uses_profile_service(
    tmp_path,
    monkeypatch,
):
    app, window = _create_window(monkeypatch, tmp_path)

    assert window._configured_command_service({}) == 'mobipick_cmd'
    assert (
        window._configured_command_service({'service': 'mobipick'})
        == 'mobipick'
    )

    window.deleteLater()
    app.processEvents()


def test_remote_compose_service_uses_host_networking():
    compose_path = (
        Path(__file__).parents[2]
        / 'mobipick_gui'
        / 'resources'
        / 'docker-compose.yml'
    )
    compose = yaml.safe_load(compose_path.read_text(encoding='utf-8'))
    remote = compose['services']['mobipick_remote_cmd']

    assert remote['network_mode'] == 'host'
    assert (
        'ROS_MASTER_URI=${ROS_MASTER_URI:-'
        'http://mobipick-os-sensor:11311}'
        in remote['environment']
    )
    assert not remote.get('privileged', False)
    assert all('.ssh' not in str(volume) for volume in remote['volumes'])
