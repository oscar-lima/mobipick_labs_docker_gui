import copy
import os

os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')

import pytest
from PyQt5.QtWidgets import QApplication, QDialog

from mobipick_gui.config import CONFIG
import mobipick_gui.main_window as main_window_module
from mobipick_gui.main_window import ImageBlacklistDialog, MainWindow
from mobipick_gui.process_tab import ProcessTab
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


def test_wizard_collects_source_workspace_selection(tmp_path):
    app = QApplication.instance() or QApplication([])
    wizard = ImageSetupWizard(
        public_images=['ozkrelo/x_mobipick_labs:noetic-v1.2'],
        default_image='ozkrelo/x_mobipick_labs:noetic-v1.2',
        host_user='oscar',
        host_uid='1001',
        host_gid='1001',
        base_image='ozkrelo/x_mobipick_labs:noetic-v1.2',
        target_image='ozkrelo/x_mobipick_labs:oscar_user_from_1.2',
        workspace_names=['gpt_ws'],
        configuration_paths=[('Workspace registry', str(tmp_path / 'workspaces.yaml'))],
        source_master_folder=str(tmp_path / 'master'),
        source_workspace_name='clean_mobipick_labs_ws',
        source_repository='https://github.com/DFKI-NI/mobipick_labs.git',
        source_branch='noetic',
        source_image='ozkrelo/x_mobipick_labs:oscar_user_from_1.2',
        install_source_default=True,
        image_blacklist=['*n8n*'],
    )

    selection = wizard.selection()

    assert selection.install_source_workspace is True
    assert selection.source_master_folder == str(tmp_path / 'master')
    assert selection.source_workspace_name == 'clean_mobipick_labs_ws'
    assert selection.source_repository.endswith('mobipick_labs.git')
    assert selection.source_branch == 'noetic'
    assert selection.source_image == 'ozkrelo/x_mobipick_labs:oscar_user_from_1.2'
    assert selection.image_blacklist == ['*n8n*']

    wizard._skip_step(wizard.install_source_workspace)

    assert wizard.selection().install_source_workspace is False

    wizard.deleteLater()
    app.processEvents()


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
                {'ref': 'ozkrelo/x_mobipick_labs:noetic-v1.1'},
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
        public_images=['ozkrelo/x_mobipick_labs:noetic-v1.1'],
        default_image='ozkrelo/x_mobipick_labs:noetic-v1.1',
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
    assert updates['images']['default'] == 'ozkrelo/x_mobipick_labs:noetic-v1.1'
    assert updates['images']['blacklist'] == []
    assert any(
        profile.get('ref') == 'ozkrelo/x_mobipick_labs:gpt_ws_from_oscar_user'
        and profile.get('compatible_workspaces') == ['gpt_ws']
        for profile in updates['images']['profiles']
    )
    assert started['pulls'] == ['ozkrelo/x_mobipick_labs:noetic-v1.1']
    assert started['build'] is selection

    window.deleteLater()
    app.processEvents()


def test_discover_filtered_image_records_skips_blacklist(monkeypatch):
    window = MainWindow.__new__(MainWindow)
    window._images_cfg = {
        'discovery_filters': [],
        'include_none_tag': False,
        'blacklist': ['*n8n*'],
    }
    window._console_log = lambda *_args, **_kwargs: None
    window._prepare_run_env = lambda kwargs: kwargs

    monkeypatch.setattr(
        main_window_module.subprocess,
        'run',
        lambda *args, **kwargs: type(
            'Result',
            (),
            {
                'returncode': 0,
                'stdout': (
                    '{"Repository":"docker.n8n.io/n8nio/n8n","Tag":"latest"}\n'
                    '{"Repository":"ozkrelo/x_mobipick_labs","Tag":"gpt"}\n'
                ),
            },
        )(),
    )

    records, error = window._discover_filtered_image_records()

    assert error is None
    assert [record['ref'] for record in records] == [
        'ozkrelo/x_mobipick_labs:gpt'
    ]


def test_image_blacklist_dialog_saves_patterns(monkeypatch):
    window = MainWindow.__new__(MainWindow)
    images_cfg = copy.deepcopy(CONFIG['images'])
    images_cfg['blacklist'] = []
    monkeypatch.setitem(CONFIG, 'images', images_cfg)
    window._images_cfg = images_cfg
    window._image_choices = ['ozkrelo/x_mobipick_labs:gpt']
    window._load_available_images = lambda show_feedback=False: None
    window._log_info = lambda *_args, **_kwargs: None

    def discover_images(blacklist_patterns=None, discovery_filters=None):
        return ([{'ref': 'ozkrelo/x_mobipick_labs:gpt'}], None)

    window._discover_filtered_image_records = discover_images
    saved = {}

    class AcceptedDialog:
        def __init__(self, *args, **kwargs):
            pass

        def exec_(self):
            return QDialog.Accepted

        def patterns(self):
            return ['*n8n*', 'repo/ignore:tag']

        def discovery_filters(self):
            return ['mobipick']

    monkeypatch.setattr(
        main_window_module,
        'ImageBlacklistDialog',
        AcceptedDialog,
    )
    monkeypatch.setattr(
        main_window_module,
        'save_user_config_update',
        lambda updates: saved.setdefault('updates', updates),
    )

    window._open_image_blacklist_dialog()

    assert saved['updates'] == {
        'images': {
            'blacklist': ['*n8n*', 'repo/ignore:tag'],
            'discovery_filters': ['mobipick'],
        }
    }
    assert window._images_cfg['blacklist'] == ['*n8n*', 'repo/ignore:tag']
    assert window._images_cfg['discovery_filters'] == ['mobipick']


def test_image_blacklist_dialog_updates_preview_from_combo():
    app = QApplication.instance() or QApplication([])
    dialog = ImageBlacklistDialog(
        [],
        ['mobipick'],
        [
            'ozkrelo/x_mobipick_labs:noetic-v1.2',
            'docker.n8n.io/n8nio/n8n:latest',
        ],
        MainWindow._image_ref_matches_pattern,
    )

    assert dialog.summary_label.text() == (
        '1 image(s) will be used; 0 image(s) will be ignored; '
        '1 image(s) will be hidden by discovery filters.'
    )
    assert dialog.preview_table.item(1, 0).text() == 'Hidden'

    dialog.image_combo.setCurrentIndex(1)
    dialog._add_selected_image()

    assert dialog.patterns() == ['docker.n8n.io/n8nio/n8n:latest']
    assert dialog.summary_label.text() == (
        '1 image(s) will be used; 1 image(s) will be ignored; '
        '0 image(s) will be hidden by discovery filters.'
    )
    assert dialog.preview_table.item(1, 0).text() == 'Ignored'
    assert dialog.preview_table.item(1, 3).text() == (
        'docker.n8n.io/n8nio/n8n:latest'
    )

    dialog.deleteLater()
    app.processEvents()


def test_source_workspace_install_registers_workspace_and_streams_in_color(
    tmp_path,
    monkeypatch,
):
    registry_path = tmp_path / 'workspaces.yaml'
    source_image = 'ozkrelo/x_mobipick_labs:noetic-v1.2'
    monkeypatch.setenv('MOBIPICK_WORKSPACE_CONFIG', str(registry_path))
    monkeypatch.setattr(
        MainWindow,
        '_discover_filtered_image_records',
        lambda self: ([{'ref': source_image}], None),
    )
    monkeypatch.setattr(
        MainWindow,
        'update_sim_status_from_poll',
        lambda self, force=False: None,
    )
    monkeypatch.setattr(MainWindow, '_ensure_network', lambda self, log_key=None: None)

    captured = {}

    def fake_start_shell(tab, bash_cmd):
        captured['command'] = bash_cmd
        captured['env'] = dict(tab.environment_overrides)
        captured['container_name'] = tab.container_name

    monkeypatch.setattr(ProcessTab, 'start_shell', fake_start_shell)

    app = QApplication.instance() or QApplication([])
    window = MainWindow(verbosity=1)
    window.poll_timer.stop()
    window._sigint_timer.stop()

    selection = SetupWizardSelection(
        pull_public_images=False,
        public_images=[],
        default_image=source_image,
        build_custom_image=False,
        host_user='oscar',
        host_uid='1001',
        host_gid='1001',
        base_image=source_image,
        target_image='ozkrelo/x_mobipick_labs:oscar_user_from_1.2',
        compatible_workspace='',
        remember_completion=True,
        install_source_workspace=True,
        source_master_folder=str(tmp_path / 'master'),
        source_workspace_name='clean_mobipick_labs_ws',
        source_repository='https://github.com/DFKI-NI/mobipick_labs.git',
        source_branch='noetic',
        source_image=source_image,
        activate_source_workspace=False,
    )

    window._start_source_workspace_install(selection)

    workspace = window._workspace_registry.get('clean_mobipick_labs_ws')
    assert workspace is not None
    assert workspace.directory == (
        tmp_path / 'master' / 'clean_mobipick_labs_ws'
    ).resolve()
    assert (workspace.directory / 'src').is_dir()
    assert captured['container_name'].startswith('mobipick-source-')
    assert 'script -qefc' in captured['command']
    assert 'git clone --branch' in captured['command']
    assert 'noetic' in captured['command']
    assert './install-deps.sh' in captured['command']
    assert './build.sh' in captured['command']
    assert 'FORCE_COLOR=1' in captured['command']
    assert captured['env']['MOBIPICK_IMAGE'] == source_image
    assert captured['env']['MOBIPICK_WORKSPACE_NAME'] == 'clean_mobipick_labs_ws'
    assert captured['env']['MOBIPICK_WORKSPACE_MOUNT_SOURCE'] == str(
        (tmp_path / 'master').resolve()
    )

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
        lambda self: ([{'ref': 'ozkrelo/x_mobipick_labs:noetic-v1.1'}], None),
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

    assert window._image_choices == ['ozkrelo/x_mobipick_labs:noetic-v1.1']
    assert not window._should_auto_show_setup_wizard()

    window.deleteLater()
    app.processEvents()


def test_record_screen_prompts_for_output_folder_and_remembers(
    tmp_path,
    monkeypatch,
):
    registry_path = tmp_path / 'workspaces.yaml'
    output_dir = tmp_path / 'captures'
    recording_cfg = copy.deepcopy(CONFIG['recording'])
    recording_cfg.update({
        'enabled_by_default': False,
        'output_dir': str(tmp_path / 'default-captures'),
        'remember_output_dir': False,
    })
    monkeypatch.setenv('MOBIPICK_WORKSPACE_CONFIG', str(registry_path))
    monkeypatch.setitem(CONFIG, 'recording', recording_cfg)
    monkeypatch.setattr(
        MainWindow,
        '_discover_filtered_image_records',
        lambda self: ([{'ref': CONFIG['images']['default']}], None),
    )
    monkeypatch.setattr(
        MainWindow,
        'update_sim_status_from_poll',
        lambda self, force=False: None,
    )
    monkeypatch.setattr(
        MainWindow,
        '_recording_output_dialog',
        lambda self, title, remember_default: (output_dir, True),
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

    window.record_checkbox.setChecked(True)

    assert window.record_checkbox.isChecked()
    assert window._recording_output_root == output_dir
    assert window._recording_remember_output_dir is True
    assert saved['updates'] == {
        'recording': {
            'output_dir': str(output_dir),
            'remember_output_dir': True,
        },
    }

    window.deleteLater()
    app.processEvents()


def test_record_screen_cancel_keeps_checkbox_unchecked(tmp_path, monkeypatch):
    registry_path = tmp_path / 'workspaces.yaml'
    recording_cfg = copy.deepcopy(CONFIG['recording'])
    recording_cfg.update({
        'enabled_by_default': False,
        'remember_output_dir': False,
    })
    monkeypatch.setenv('MOBIPICK_WORKSPACE_CONFIG', str(registry_path))
    monkeypatch.setitem(CONFIG, 'recording', recording_cfg)
    monkeypatch.setattr(
        MainWindow,
        '_discover_filtered_image_records',
        lambda self: ([{'ref': CONFIG['images']['default']}], None),
    )
    monkeypatch.setattr(
        MainWindow,
        'update_sim_status_from_poll',
        lambda self, force=False: None,
    )
    monkeypatch.setattr(
        MainWindow,
        '_recording_output_dialog',
        lambda self, title, remember_default: None,
    )

    app = QApplication.instance() or QApplication([])
    window = MainWindow(verbosity=1)
    window.poll_timer.stop()
    window._sigint_timer.stop()

    window.record_checkbox.setChecked(True)

    assert not window.record_checkbox.isChecked()

    window.deleteLater()
    app.processEvents()


def test_missing_default_image_opens_setup_wizard_when_other_images_exist(
    tmp_path,
    monkeypatch,
):
    registry_path = tmp_path / 'workspaces.yaml'
    monkeypatch.setenv('MOBIPICK_WORKSPACE_CONFIG', str(registry_path))
    images_cfg = copy.deepcopy(CONFIG['images'])
    images_cfg['default'] = 'ozkrelo/x_mobipick_labs:noetic-v1.1'
    monkeypatch.setitem(CONFIG, 'images', images_cfg)
    setup_cfg = copy.deepcopy(CONFIG['setup_wizard'])
    setup_cfg['completed'] = False
    monkeypatch.setitem(CONFIG, 'setup_wizard', setup_cfg)
    monkeypatch.setattr(
        MainWindow,
        '_discover_filtered_image_records',
        lambda self: ([{'ref': 'ozkrelo/x_mobipick_labs:noetic-v1.1'}], None),
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

    scheduled = []
    monkeypatch.setenv('QT_QPA_PLATFORM', 'xcb')
    monkeypatch.setattr(
        main_window_module.QTimer,
        'singleShot',
        lambda _delay, callback: scheduled.append(callback),
    )
    monkeypatch.setattr(
        MainWindow,
        '_show_missing_default_image_dialog',
        lambda self, image_ref: pytest.fail(
            f'unexpected missing default dialog for {image_ref}'
        ),
    )
    window._images_cfg['default'] = 'ozkrelo/mobipick_labs:noetic'

    window._load_available_images(show_feedback=False)

    assert window._image_choices == ['ozkrelo/x_mobipick_labs:noetic-v1.1']
    assert scheduled == [window._open_setup_wizard]

    window.deleteLater()
    app.processEvents()
