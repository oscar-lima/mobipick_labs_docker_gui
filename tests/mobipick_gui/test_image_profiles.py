import copy
import os
from pathlib import Path

import yaml

os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')

from PyQt5.QtWidgets import QApplication, QPushButton

from mobipick_gui import main_window as main_window_module
from mobipick_gui.config import CONFIG
from mobipick_gui.main_window import MainWindow, WorkspaceMatchDialog
from mobipick_gui.workspaces import RosWorkspace, WorkspaceRegistry


def _create_window(monkeypatch, registry_path, images):
    monkeypatch.setenv('MOBIPICK_WORKSPACE_CONFIG', str(registry_path))
    monkeypatch.setitem(
        CONFIG,
        'workspace_mismatch_warning',
        {'silenced_exceptions': []},
    )
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


def _button_texts(window):
    return {button.text() for button in window.findChildren(QPushButton)}


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
    assert 'Build Active Workspace' not in _button_texts(window)
    assert 'image default only' in window.image_combo.currentText()

    window.deleteLater()
    app.processEvents()


def test_workspace_mismatch_warning_detects_non_matching_image(
    tmp_path,
    monkeypatch,
):
    image = 'ozkrelo/x_mobipick_labs:noetic-v1.2'
    registry_path, _ = _write_registry(tmp_path, image)

    app, window = _create_window(
        monkeypatch,
        registry_path,
        ['ozkrelo/mobipick_labs:noetic', image],
    )

    reason = window._workspace_mismatch_warning_reason()

    assert 'does not mount host workspaces' in reason
    assert not window._workspace_mismatch_exception_exists()

    window.deleteLater()
    app.processEvents()


def test_workspace_mismatch_warning_detects_docker_image_default(
    tmp_path,
    monkeypatch,
):
    image = 'ozkrelo/x_mobipick_labs:ubuntu_user_from_1.2'
    registry_path = tmp_path / 'workspaces.yaml'

    app, window = _create_window(
        monkeypatch,
        registry_path,
        ['ozkrelo/x_mobipick_labs:noetic-v1.1', image],
    )
    window._select_image(image, log_selection=False)

    reason = window._workspace_mismatch_warning_reason()

    assert window._workspace_registry.active == ''
    assert 'Docker image default' in reason
    assert window._workspace_mismatch_exception_entry() == {
        'image': image,
        'workspace': 'Docker image default',
    }

    window.deleteLater()
    app.processEvents()


def test_workspace_mismatch_exception_is_saved_and_suppresses_warning(
    tmp_path,
    monkeypatch,
):
    image = 'ozkrelo/x_mobipick_labs:noetic-v1.2'
    registry_path, _ = _write_registry(tmp_path, image)
    saved_updates = []
    monkeypatch.setattr(
        main_window_module,
        'save_user_config_update',
        lambda updates: saved_updates.append(updates) or updates,
    )
    monkeypatch.setattr(
        main_window_module.QMessageBox,
        'warning',
        lambda *args, **kwargs: None,
    )

    app, window = _create_window(
        monkeypatch,
        registry_path,
        ['ozkrelo/mobipick_labs:noetic', image],
    )

    window._remember_workspace_mismatch_exception()

    assert window._workspace_mismatch_exception_exists()
    assert saved_updates == [
        {
            'workspace_mismatch_warning': {
                'silenced_exceptions': [
                    {
                        'image': image,
                        'workspace': 'clean_mobipick_labs_ws',
                    },
                ],
            },
        },
    ]

    window.deleteLater()
    app.processEvents()


def test_workspace_mismatch_can_be_marked_as_workspace_match(
    tmp_path,
    monkeypatch,
):
    image = 'ozkrelo/x_mobipick_labs:noetic-v1.2'
    registry_path, _ = _write_registry(tmp_path, image)
    saved_updates = []
    monkeypatch.setattr(
        main_window_module,
        'save_user_config_update',
        lambda updates: saved_updates.append(updates) or updates,
    )

    app, window = _create_window(
        monkeypatch,
        registry_path,
        ['ozkrelo/mobipick_labs:noetic', image],
    )

    assert window._workspace_mismatch_warning_reason()

    window._mark_current_image_workspace_match()

    assert window._image_compatible_with_workspace(
        image,
        'clean_mobipick_labs_ws',
    ) is True
    assert 'workspace match' in window.image_combo.currentText()
    profiles = saved_updates[-1]['images']['profiles']
    assert any(
        profile.get('ref') == image
        and 'clean_mobipick_labs_ws' in profile.get('compatible_workspaces', [])
        for profile in profiles
    )

    window.deleteLater()
    app.processEvents()


def test_build_match_helper_marks_image_as_workspace_match(
    tmp_path,
    monkeypatch,
):
    _install_private_image_profiles(monkeypatch)
    image = 'ozkrelo/x_mobipick_labs:oscar_user_from_1.2'
    registry_path, _ = _write_registry(
        tmp_path,
        image,
        workspace_name='common_tools_ws',
    )
    saved_updates = []
    monkeypatch.setattr(
        main_window_module,
        'save_user_config_update',
        lambda updates: saved_updates.append(updates) or updates,
    )

    app, window = _create_window(
        monkeypatch,
        registry_path,
        ['ozkrelo/mobipick_labs:noetic', image],
    )
    workspace = window._workspace_registry.get('common_tools_ws')

    assert window._image_compatible_with_workspace(image, 'common_tools_ws') is False

    assert window._mark_workspace_build_image_match(workspace)

    assert window._image_compatible_with_workspace(image, 'common_tools_ws') is True
    profiles = saved_updates[-1]['images']['profiles']
    assert any(
        profile.get('ref') == image
        and 'common_tools_ws' in profile.get('compatible_workspaces', [])
        for profile in profiles
    )

    window.deleteLater()
    app.processEvents()


def test_workspace_match_map_can_be_saved_from_editor(
    tmp_path,
    monkeypatch,
):
    image = 'ozkrelo/x_mobipick_labs:noetic-v1.2'
    registry_path, _ = _write_registry(tmp_path, image)
    saved_updates = []
    monkeypatch.setattr(
        main_window_module,
        'save_user_config_update',
        lambda updates: saved_updates.append(updates) or updates,
    )

    app, window = _create_window(
        monkeypatch,
        registry_path,
        ['ozkrelo/mobipick_labs:noetic', image],
    )

    window._save_workspace_match_map({
        image: ['Docker image default', 'clean_mobipick_labs_ws'],
    })

    assert window._image_compatible_with_workspace(
        image,
        'clean_mobipick_labs_ws',
    ) is True
    assert window._image_compatible_with_workspace(image, '') is True
    profiles = saved_updates[-1]['images']['profiles']
    assert any(
        profile.get('ref') == image
        and profile.get('compatible_workspaces') == [
            'Docker image default',
            'clean_mobipick_labs_ws',
        ]
        for profile in profiles
    )

    window.deleteLater()
    app.processEvents()


def test_workspace_match_dialog_edits_image_matches():
    app = QApplication.instance() or QApplication([])
    dialog = WorkspaceMatchDialog(
        ['image:a', 'image:b'],
        ['base_ws', 'demo_ws'],
        {'image:a': ['base_ws']},
    )

    assert dialog._checkboxes['base_ws'].isChecked()
    assert not dialog._checkboxes['demo_ws'].isChecked()

    dialog._checkboxes['demo_ws'].setChecked(True)
    dialog.image_combo.setCurrentIndex(1)
    dialog._checkboxes['Docker image default'].setChecked(True)

    assert dialog.matches() == {
        'image:a': ['base_ws', 'demo_ws'],
        'image:b': ['Docker image default'],
    }

    dialog.deleteLater()
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
    assert 'Build Active Workspace' not in _button_texts(window)
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
