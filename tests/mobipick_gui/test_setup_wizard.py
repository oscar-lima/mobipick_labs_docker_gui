import copy
import os

os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')

from PyQt5.QtWidgets import QApplication

from mobipick_gui.config import CONFIG
import mobipick_gui.main_window as main_window_module
from mobipick_gui.main_window import MainWindow
from mobipick_gui.setup_wizard import ImageSetupWizard, SetupWizardSelection
from mobipick_gui.workspaces import RosWorkspace, WorkspaceRegistry


def test_wizard_parses_newline_and_comma_image_lists():
    images = ImageSetupWizard._parse_image_list(
        'ozkrelo/mobipick_labs:noetic, ozkrelo/x_mobipick_labs:noetic-v1.2\n'
        'ozkrelo/mobipick_labs:noetic'
    )

    assert images == [
        'ozkrelo/mobipick_labs:noetic',
        'ozkrelo/x_mobipick_labs:noetic-v1.2',
    ]


def test_custom_image_dockerfile_creates_host_user_image():
    dockerfile = MainWindow._custom_image_dockerfile(
        'ozkrelo/x_mobipick_labs:noetic-v1.2'
    )

    assert dockerfile.startswith('FROM ozkrelo/x_mobipick_labs:noetic-v1.2')
    assert 'ARG USER' in dockerfile
    assert 'useradd -m -u "${UID}" -g "${GID}"' in dockerfile
    assert 'ENTRYPOINT ["/usr/local/bin/entrypoint_user.sh"]' in dockerfile


def test_setup_wizard_persists_custom_image_profile(
    tmp_path,
    monkeypatch,
):
    registry_path = tmp_path / 'workspaces.yaml'
    workspace_path = tmp_path / 'ros_ws' / 'gpt_ws'
    (workspace_path / 'src').mkdir(parents=True)
    registry = WorkspaceRegistry(registry_path)
    registry.master_folder = str(workspace_path.parent)
    registry.upsert(
        RosWorkspace(
            name='gpt_ws',
            path=str(workspace_path),
            image='ozkrelo/x_mobipick_labs:gpt_ws_from_oscar_user',
        )
    )
    registry.active = 'gpt_ws'
    registry.save()

    monkeypatch.setenv('MOBIPICK_WORKSPACE_CONFIG', str(registry_path))
    monkeypatch.setitem(CONFIG, 'images', copy.deepcopy(CONFIG['images']))
    monkeypatch.setattr(
        MainWindow,
        '_discover_filtered_image_records',
        lambda self: (
            [
                {'ref': 'ozkrelo/mobipick_labs:noetic'},
                {'ref': 'ozkrelo/x_mobipick_labs:gpt_ws_from_oscar_user'},
            ],
            None,
        ),
    )
    monkeypatch.setattr(
        MainWindow,
        'update_sim_status_from_poll',
        lambda self, force=False: None,
    )
    started = {}
    monkeypatch.setattr(
        MainWindow,
        '_start_image_pulls',
        lambda self, images: started.setdefault('pulls', images),
    )
    monkeypatch.setattr(
        MainWindow,
        '_start_custom_image_build',
        lambda self, selection: started.setdefault('build', selection),
    )
    saved = {}
    monkeypatch.setattr(
        main_window_module,
        'save_user_config_update',
        lambda updates: saved.setdefault('updates', updates),
    )

    app = QApplication.instance() or QApplication([])
    window = MainWindow(verbosity=1)
    window.poll_timer.stop()
    window._sigint_timer.stop()

    selection = SetupWizardSelection(
        pull_public_images=True,
        public_images=['ozkrelo/mobipick_labs:noetic'],
        default_image='ozkrelo/mobipick_labs:noetic',
        build_custom_image=True,
        host_user='oscar',
        host_uid='1001',
        host_gid='1001',
        base_image='ozkrelo/x_mobipick_labs:noetic-v1.2',
        target_image='ozkrelo/x_mobipick_labs:gpt_ws_from_oscar_user',
        compatible_workspace='gpt_ws',
        remember_completion=True,
    )

    window._apply_setup_wizard(selection)

    updates = saved['updates']
    assert updates['setup_wizard']['completed'] is True
    assert updates['images']['default'] == 'ozkrelo/mobipick_labs:noetic'
    assert any(
        profile.get('ref') == 'ozkrelo/x_mobipick_labs:gpt_ws_from_oscar_user'
        and profile.get('compatible_workspaces') == ['gpt_ws']
        for profile in updates['images']['profiles']
    )
    assert started['pulls'] == ['ozkrelo/mobipick_labs:noetic']
    assert started['build'] is selection

    window.deleteLater()
    app.processEvents()


def test_setup_wizard_does_not_auto_open_when_images_are_available(
    tmp_path,
    monkeypatch,
):
    registry_path = tmp_path / 'workspaces.yaml'
    monkeypatch.setenv('MOBIPICK_WORKSPACE_CONFIG', str(registry_path))
    monkeypatch.setattr(
        MainWindow,
        '_discover_filtered_image_records',
        lambda self: ([{'ref': 'ozkrelo/mobipick_labs:noetic'}], None),
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
    monkeypatch.setenv('QT_QPA_PLATFORM', 'xcb')

    assert window._image_choices == ['ozkrelo/mobipick_labs:noetic']
    assert not window._should_auto_show_setup_wizard()

    window.deleteLater()
    app.processEvents()
