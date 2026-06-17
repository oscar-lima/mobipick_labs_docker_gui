import copy
import os
from pathlib import Path

import yaml

os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')

from PyQt5.QtWidgets import QApplication

from mobipick_gui.config import CONFIG
from mobipick_gui.main_window import MainWindow
from mobipick_gui.workspaces import RosWorkspace, WorkspaceRegistry


def _create_window(monkeypatch, registry_path, images):
    monkeypatch.setenv('MOBIPICK_WORKSPACE_CONFIG', str(registry_path))
    monkeypatch.setattr(
        MainWindow,
        '_discover_filtered_image_records',
        lambda self: ([{'ref': image} for image in images], None),
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
    return app, window


def _install_private_image_profiles(monkeypatch):
    images_cfg = copy.deepcopy(CONFIG['images'])
    profiles = list(images_cfg.get('profiles', []))
    profiles.extend(
        [
            {
                'ref': 'ozkrelo/x_mobipick_labs:oscar_user_from_1.2',
                'user': 'host',
                'supports_host_workspaces': True,
                'compatible_workspaces': ['clean_mobipick_labs_ws'],
            },
            {
                'ref': 'ozkrelo/x_mobipick_labs:rae_ws_from_oscar_user',
                'user': 'host',
                'supports_host_workspaces': True,
                'compatible_workspaces': ['rae_upom_mobipick_ws'],
            },
            {
                'ref': 'ozkrelo/x_mobipick_labs:gpt_ws_from_oscar_user',
                'user': 'host',
                'supports_host_workspaces': True,
                'compatible_workspaces': ['gpt_ws'],
            },
        ]
    )
    images_cfg['profiles'] = profiles
    monkeypatch.setitem(CONFIG, 'images', images_cfg)


def _write_registry(tmp_path, image, workspace_name='clean_mobipick_labs_ws'):
    registry_path = tmp_path / 'workspaces.yaml'
    workspace_path = tmp_path / 'ros_ws' / workspace_name
    (workspace_path / 'src').mkdir(parents=True)
    registry = WorkspaceRegistry(registry_path)
    registry.master_folder = str(workspace_path.parent)
    registry.upsert(
        RosWorkspace(
            name=workspace_name,
            path=str(workspace_path),
            image=image,
        )
    )
    registry.active = workspace_name
    registry.save()
    return registry_path, workspace_path


def _combo_texts(window):
    return [
        window.image_combo.itemText(index)
        for index in range(window.image_combo.count())
    ]


def test_public_root_image_uses_baked_workspace_for_private_workspace(
    tmp_path,
    monkeypatch,
):
    image = 'ozkrelo/x_mobipick_labs:noetic-v1.2'
    registry_path, workspace_path = _write_registry(tmp_path, image)

    app, window = _create_window(
        monkeypatch,
        registry_path,
        ['ozkrelo/mobipick_labs:noetic', image],
    )

    env_args = window._compose_env_args()

    assert 'MOBIPICK_CONTAINER_USER=root' in env_args
    assert 'MOBIPICK_CONTAINER_WORKDIR=/root/catkin_ws' in env_args
    assert 'MOBIPICK_WORKSPACE_ENABLED=0' in env_args
    assert not any(str(workspace_path) in arg for arg in env_args)
    assert not window.build_workspace_button.isEnabled()
    assert 'image default only' in window.image_combo.currentText()

    window.deleteLater()
    app.processEvents()


def test_host_user_image_mounts_active_workspace(tmp_path, monkeypatch):
    _install_private_image_profiles(monkeypatch)
    image = 'ozkrelo/x_mobipick_labs:oscar_user_from_1.2'
    registry_path, workspace_path = _write_registry(tmp_path, image)

    app, window = _create_window(
        monkeypatch,
        registry_path,
        ['ozkrelo/mobipick_labs:noetic', image],
    )

    env_args = window._compose_env_args()
    host_user = CONFIG['process']['compose_run_env']['MOBIPICK_HOST_USER']

    assert f'MOBIPICK_CONTAINER_USER={host_user}' in env_args
    assert 'MOBIPICK_WORKSPACE_ENABLED=1' in env_args
    assert any('MOBIPICK_WORKSPACE_PATH=' in arg for arg in env_args)
    assert str(workspace_path) not in ' '.join(env_args)
    assert window.build_workspace_button.isEnabled()
    assert 'workspace match' in window.image_combo.currentText()

    window.deleteLater()
    app.processEvents()


def test_workspace_match_highlight_uses_explicit_image_profile(
    tmp_path,
    monkeypatch,
):
    _install_private_image_profiles(monkeypatch)
    images = [
        'ozkrelo/mobipick_labs:noetic',
        'ozkrelo/x_mobipick_labs:rae_ws_from_oscar_user',
        'ozkrelo/x_mobipick_labs:oscar_user_from_1.2',
        'ozkrelo/x_mobipick_labs:gpt_ws_from_oscar_user',
    ]
    registry_path, _ = _write_registry(
        tmp_path,
        'ozkrelo/x_mobipick_labs:gpt_ws_from_oscar_user',
        workspace_name='gpt_ws',
    )

    app, window = _create_window(monkeypatch, registry_path, images)

    texts = _combo_texts(window)
    assert (
        'ozkrelo/x_mobipick_labs:gpt_ws_from_oscar_user  [workspace match]'
        in texts
    )
    assert all(
        'workspace match' not in text
        for text in texts
        if 'gpt_ws_from_oscar_user' not in text
    )

    window.deleteLater()
    app.processEvents()


def test_rae_workspace_only_matches_rae_image(tmp_path, monkeypatch):
    _install_private_image_profiles(monkeypatch)
    images = [
        'ozkrelo/mobipick_labs:noetic',
        'ozkrelo/x_mobipick_labs:rae_ws_from_oscar_user',
        'ozkrelo/x_mobipick_labs:oscar_user_from_1.2',
        'ozkrelo/x_mobipick_labs:gpt_ws_from_oscar_user',
    ]
    registry_path, _ = _write_registry(
        tmp_path,
        'ozkrelo/x_mobipick_labs:rae_ws_from_oscar_user',
        workspace_name='rae_upom_mobipick_ws',
    )

    app, window = _create_window(monkeypatch, registry_path, images)

    texts = _combo_texts(window)
    assert (
        'ozkrelo/x_mobipick_labs:rae_ws_from_oscar_user  [workspace match]'
        in texts
    )
    assert all(
        'workspace match' not in text
        for text in texts
        if 'rae_ws_from_oscar_user' not in text
    )

    window.deleteLater()
    app.processEvents()


def test_dependency_workspace_has_no_workspace_match_highlight(
    tmp_path,
    monkeypatch,
):
    _install_private_image_profiles(monkeypatch)
    images = [
        'ozkrelo/mobipick_labs:noetic',
        'ozkrelo/x_mobipick_labs:rae_ws_from_oscar_user',
        'ozkrelo/x_mobipick_labs:oscar_user_from_1.2',
        'ozkrelo/x_mobipick_labs:gpt_ws_from_oscar_user',
    ]
    registry_path, _ = _write_registry(
        tmp_path,
        'ozkrelo/x_mobipick_labs:oscar_user_from_1.2',
        workspace_name='common_tools_ws',
    )

    app, window = _create_window(monkeypatch, registry_path, images)

    assert all('workspace match' not in text for text in _combo_texts(window))

    window.deleteLater()
    app.processEvents()


def test_compose_defaults_to_root_user_for_services():
    compose_path = (
        Path(__file__).parents[2]
        / 'mobipick_gui'
        / 'resources'
        / 'docker-compose.yml'
    )
    compose = yaml.safe_load(compose_path.read_text(encoding='utf-8'))

    for service in ('mobipick', 'mobipick_cmd', 'mobipick_remote_cmd'):
        definition = compose['services'][service]
        assert definition['user'] == '${MOBIPICK_CONTAINER_USER:-root}'
        assert definition['working_dir'] == '${MOBIPICK_CONTAINER_WORKDIR:-/tmp}'
