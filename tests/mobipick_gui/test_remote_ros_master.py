import os
from pathlib import Path

import yaml

os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')

from PyQt5.QtWidgets import QApplication

from mobipick_gui.config import CONFIG
from mobipick_gui.main_window import MainWindow


def _create_window(monkeypatch):
    images = [{'ref': 'ozkrelo/mobipick_labs:noetic'}]
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


def test_remote_mode_injects_master_and_disables_local_stack(
    monkeypatch,
):
    app, window = _create_window(monkeypatch)

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
    monkeypatch,
):
    app, window = _create_window(monkeypatch)
    window.remote_master_checkbox.setChecked(True)
    started = {}
    rviz_tab = window.tasks['rviz']

    monkeypatch.setattr(window, '_claim_xhost', lambda *args, **kwargs: None)
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
