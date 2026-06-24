import copy
import os

os.environ['QT_QPA_PLATFORM'] = 'offscreen'

import pytest
from PyQt5.QtWidgets import QApplication, QDialog, QTextEdit, QWizard

from mobipick_gui.config import CONFIG
import mobipick_gui.main_window as main_window_module
from mobipick_gui.main_window import ImageBlacklistDialog, MainWindow
from mobipick_gui.process_tab import ProcessTab
from mobipick_gui.setup_wizard import (
    HostDependency,
    ImageSetupWizard,
    SetupWizardSelection,
)
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
        host_user='testuser',
        host_uid='1001',
        host_gid='1001',
        base_image='ozkrelo/x_mobipick_labs:noetic-v1.2',
        target_image='ozkrelo/x_mobipick_labs:host_user_from_1.2',
        workspace_names=['gpt_ws'],
        configuration_paths=[('Workspace registry', str(tmp_path / 'workspaces.yaml'))],
        source_master_folder=str(tmp_path / 'master'),
        source_workspace_name='clean_mobipick_labs_ws',
        source_repository='https://github.com/DFKI-NI/mobipick_labs.git',
        source_branch='noetic',
        source_image='ozkrelo/x_mobipick_labs:host_user_from_1.2',
        install_source_default=True,
        image_blacklist=['*n8n*'],
    )

    selection = wizard.selection()

    assert selection.install_source_workspace is True
    assert selection.source_master_folder == str(tmp_path / 'master')
    assert selection.source_workspace_name == 'clean_mobipick_labs_ws'
    assert selection.source_repository.endswith('mobipick_labs.git')
    assert selection.source_branch == 'noetic'
    assert selection.source_image == 'ozkrelo/x_mobipick_labs:host_user_from_1.2'
    assert selection.image_blacklist == ['*n8n*']
    assert selection.public_image_pull_mode == 'gui'

    wizard.public_image_pull_mode.setCurrentIndex(
        wizard.public_image_pull_mode.findData('manual')
    )

    assert wizard.selection().public_image_pull_mode == 'manual'

    wizard._skip_step(wizard.install_source_workspace)

    assert wizard.selection().install_source_workspace is False

    wizard.deleteLater()
    app.processEvents()


def test_wizard_page_titles_show_step_position(tmp_path):
    app = QApplication.instance() or QApplication([])
    wizard = ImageSetupWizard(
        public_images=['ozkrelo/x_mobipick_labs:noetic-v1.2'],
        default_image='ozkrelo/x_mobipick_labs:noetic-v1.2',
        host_user='testuser',
        host_uid='1001',
        host_gid='1001',
        base_image='ozkrelo/x_mobipick_labs:noetic-v1.2',
        target_image='ozkrelo/x_mobipick_labs:host_user_from_1.2',
        workspace_names=[],
        configuration_paths=[],
        source_master_folder=str(tmp_path / 'master'),
        source_workspace_name='clean_mobipick_labs_ws',
        source_repository='https://github.com/DFKI-NI/mobipick_labs.git',
        source_branch='noetic',
        source_image='ozkrelo/x_mobipick_labs:host_user_from_1.2',
    )
    expected_titles = [
        'Step 1/7: Host Dependencies',
        'Step 2/7: Setup Guide',
        'Step 3/7: Public Images',
        'Step 4/7: Development Image',
        'Step 5/7: Source Workspace',
    ]

    wizard.show()
    app.processEvents()
    for expected_title in expected_titles:
        assert wizard.currentPage().title() == expected_title
        if expected_title != expected_titles[-1]:
            wizard.next()
            app.processEvents()

    assert wizard.page(wizard._progress_page_id).title() == 'Step 6/7: Run Setup'
    assert (
        wizard.page(wizard._summary_page_id).title()
        == 'Step 7/7: Setup Summary'
    )

    wizard.deleteLater()
    app.processEvents()


def test_wizard_dependency_page_builds_copyable_install_command(tmp_path):
    app = QApplication.instance() or QApplication([])
    refreshed = []

    def refresh_dependencies():
        refreshed.append(True)
        return [
            HostDependency(
                key='docker',
                label='Docker Engine',
                package='docker-ce',
                installed=True,
                reason='Required to run containers.',
                required=True,
            ),
            HostDependency(
                key='wmctrl',
                label='wmctrl',
                package='wmctrl',
                installed=False,
                reason='Optional window layout support.',
            ),
            HostDependency(
                key='ffmpeg',
                label='FFmpeg',
                package='ffmpeg',
                installed=True,
                reason='Optional recording support.',
            ),
        ]

    wizard = ImageSetupWizard(
        public_images=['ozkrelo/x_mobipick_labs:noetic-v1.2'],
        default_image='ozkrelo/x_mobipick_labs:noetic-v1.2',
        host_user='testuser',
        host_uid='1001',
        host_gid='1001',
        base_image='ozkrelo/x_mobipick_labs:noetic-v1.2',
        target_image='ozkrelo/x_mobipick_labs:host_user_from_1.2',
        workspace_names=[],
        configuration_paths=[],
        source_master_folder=str(tmp_path / 'master'),
        source_workspace_name='clean_mobipick_labs_ws',
        source_repository='https://github.com/DFKI-NI/mobipick_labs.git',
        source_branch='noetic',
        source_image='ozkrelo/x_mobipick_labs:host_user_from_1.2',
        host_dependencies=[
            HostDependency(
                key='docker',
                label='Docker Engine',
                package='docker-ce',
                installed=False,
                reason='Required to run containers.',
                required=True,
            ),
            HostDependency(
                key='wmctrl',
                label='wmctrl',
                package='wmctrl',
                installed=False,
                reason='Optional window layout support.',
            ),
            HostDependency(
                key='ffmpeg',
                label='FFmpeg',
                package='ffmpeg',
                installed=True,
                reason='Optional recording support.',
            ),
        ],
        host_dependency_refresher=refresh_dependencies,
    )
    command = wizard.dependency_command_edit.toPlainText()
    assert command.startswith("bash <<'MOBIPICK_DOCKER_INSTALL'")
    assert 'confirm_step()' in command
    assert 'SETUP_STEP_TOTAL=7' in command
    assert 'Run ${step_label}? [y/N]' in command
    assert '==> ${step_label}: ${title}' in command
    assert 'https://download.docker.com/linux/ubuntu' in command
    assert 'sudo apt install -y ca-certificates curl gnupg' in command
    assert 'sudo gpg --dearmor --yes -o /etc/apt/keyrings/docker.gpg' in command
    assert 'signed-by=/etc/apt/keyrings/docker.gpg' in command
    assert '/etc/apt/keyrings/docker.asc' in command
    assert 'apt-cache policy "${docker_packages[@]}"' in command
    assert 'has no apt candidate' in command
    assert 'docker-ce docker-ce-cli containerd.io' in command
    assert 'docker-buildx-plugin docker-compose-plugin' in command
    assert 'support_packages=(wmctrl)' in command
    assert 'sudo apt install -y "${support_packages[@]}"' in command
    assert 'ffmpeg' not in command
    assert 'sudo systemctl restart containerd' in command
    assert 'sudo systemctl restart docker' in command
    assert 'sudo docker images' in command
    assert wizard.dependency_done_button.text() == 'Run Checks'

    wizard._dependency_checkboxes['wmctrl'].setChecked(False)
    command = wizard.dependency_command_edit.toPlainText()
    assert 'docker-ce docker-ce-cli containerd.io' in command
    assert 'wmctrl' not in command

    wizard._mark_selected_dependencies_done()

    assert refreshed == [True]
    assert not wizard._dependency_checkboxes['docker'].isChecked()
    assert wizard._dependency_checkboxes['wmctrl'].isChecked()
    assert 'sudo apt install -y "${host_packages[@]}"' in wizard.dependency_command_edit.toPlainText()
    assert 'SETUP_STEP_TOTAL=2' in wizard.dependency_command_edit.toPlainText()
    assert 'Run ${step_label}? [y/N]' in wizard.dependency_command_edit.toPlainText()
    assert 'download.docker.com' not in wizard.dependency_command_edit.toPlainText()

    wizard.close()
    wizard.deleteLater()
    app.processEvents()


def test_wizard_dependency_check_opens_bug_report_for_failures(tmp_path):
    app = QApplication.instance() or QApplication([])
    reports = []

    def refresh_dependencies():
        return [
            HostDependency(
                key='docker',
                label='Docker Engine',
                package='docker-ce',
                installed=False,
                reason='"docker ps" failed: permission denied',
                required=True,
            ),
            HostDependency(
                key='docker_compose',
                label='Docker Compose plugin',
                package='docker-compose-plugin',
                installed=False,
                reason='"docker compose version" failed: no compose command',
                required=True,
            ),
        ]

    wizard = ImageSetupWizard(
        public_images=['ozkrelo/x_mobipick_labs:noetic-v1.2'],
        default_image='ozkrelo/x_mobipick_labs:noetic-v1.2',
        host_user='testuser',
        host_uid='1001',
        host_gid='1001',
        base_image='ozkrelo/x_mobipick_labs:noetic-v1.2',
        target_image='ozkrelo/x_mobipick_labs:host_user_from_1.2',
        workspace_names=[],
        configuration_paths=[],
        source_master_folder=str(tmp_path / 'master'),
        source_workspace_name='clean_mobipick_labs_ws',
        source_repository='https://github.com/DFKI-NI/mobipick_labs.git',
        source_branch='noetic',
        source_image='ozkrelo/x_mobipick_labs:host_user_from_1.2',
        host_dependencies=refresh_dependencies(),
        host_dependency_refresher=refresh_dependencies,
        host_dependency_report_handler=reports.append,
    )

    wizard._mark_selected_dependencies_done()

    assert wizard.dependency_report_button.isEnabled()
    assert 'Some host dependency checks still failed' in (
        wizard.dependency_result_label.text()
    )

    wizard._open_dependency_report()

    assert reports
    assert 'Docker Engine' in reports[-1]
    assert 'permission denied' in reports[-1]
    assert 'docker-compose-plugin' in reports[-1]

    wizard.close()
    wizard.deleteLater()
    app.processEvents()


def test_wizard_dependency_check_shows_explained_results(tmp_path):
    app = QApplication.instance() or QApplication([])

    def refresh_dependencies():
        return [
            HostDependency(
                key='docker',
                label='Docker Engine',
                package='docker-ce',
                installed=True,
                reason='"docker ps" succeeded',
                required=True,
                check_commands=['command -v docker', 'docker ps'],
            ),
            HostDependency(
                key='wmctrl',
                label='wmctrl',
                package='wmctrl',
                installed=True,
                reason='found at /usr/bin/wmctrl',
                check_commands=['command -v wmctrl'],
            ),
        ]

    wizard = ImageSetupWizard(
        public_images=['ozkrelo/x_mobipick_labs:noetic-v1.2'],
        default_image='ozkrelo/x_mobipick_labs:noetic-v1.2',
        host_user='testuser',
        host_uid='1001',
        host_gid='1001',
        base_image='ozkrelo/x_mobipick_labs:noetic-v1.2',
        target_image='ozkrelo/x_mobipick_labs:host_user_from_1.2',
        workspace_names=[],
        configuration_paths=[],
        source_master_folder=str(tmp_path / 'master'),
        source_workspace_name='clean_mobipick_labs_ws',
        source_repository='https://github.com/DFKI-NI/mobipick_labs.git',
        source_branch='noetic',
        source_image='ozkrelo/x_mobipick_labs:host_user_from_1.2',
        host_dependencies=refresh_dependencies(),
        host_dependency_refresher=refresh_dependencies,
    )

    wizard._mark_selected_dependencies_done()
    app.processEvents()

    assert wizard._dependency_details_dialog is not None
    detail_edits = wizard._dependency_details_dialog.findChildren(QTextEdit)
    assert len(detail_edits) == 2
    details = detail_edits[0].toPlainText()
    commands = detail_edits[1].toPlainText()
    assert 'Everything OK' in details
    assert 'All configured host dependency checks passed.' in details
    assert 'Check: Docker Engine' in details
    assert 'Result: OK' in details
    assert (
        'Why: This is a required host dependency. Apt package: docker-ce.'
        in details
    )
    assert 'Evidence: "docker ps" succeeded' in details
    assert 'Check: wmctrl' in details
    assert 'Evidence: found at /usr/bin/wmctrl' in details
    assert '# Docker Engine (OK)' in commands
    assert 'command -v docker' in commands
    assert 'docker ps' in commands
    assert '# wmctrl (OK)' in commands
    assert 'command -v wmctrl' in commands

    wizard.close()
    wizard.deleteLater()
    app.processEvents()


def test_host_dependency_checks_report_docker_install_details(monkeypatch):
    monkeypatch.setattr(
        main_window_module.shutil,
        'which',
        lambda name: '/usr/bin/docker' if name == 'docker' else None,
    )
    monkeypatch.setattr(
        MainWindow,
        '_docker_apt_candidate_status',
        classmethod(
            lambda cls, packages: (
                False,
                'docker-compose-plugin: (none); missing candidates: docker-compose-plugin',
            )
        ),
    )

    def fake_shell_status(command, timeout=4.0):
        if 'download.docker.com/linux/ubuntu' in command:
            return False, 'no Docker apt source'
        if 'snap list docker' in command:
            return False, 'Name    Version\n'
        if command == 'ps -p 1 -o comm=':
            return True, 'systemd'
        return True, ''

    def fake_command_status(args, timeout=4.0):
        if args == ['docker', 'ps']:
            return False, 'Cannot connect to the Docker daemon'
        if args == ['systemctl', 'is-active', 'docker']:
            return False, 'inactive'
        if args == ['systemctl', 'is-active', 'containerd']:
            return False, 'failed'
        if args == ['id', '-nG']:
            return True, 'rasputin sudo'
        if args == ['ls', '-l', '/var/run/docker.sock']:
            return True, 'srw-rw---- root docker /var/run/docker.sock'
        if args == ['docker', 'compose', 'version']:
            return False, 'docker: compose is not a docker command'
        if args == ['docker', 'compose', 'run', '--help']:
            return False, 'docker: compose is not a docker command'
        raise AssertionError(f'unexpected command: {args}')

    monkeypatch.setattr(
        MainWindow,
        '_host_shell_status',
        classmethod(lambda cls, command, timeout=4.0: fake_shell_status(command, timeout)),
    )
    monkeypatch.setattr(
        MainWindow,
        '_host_command_status',
        staticmethod(fake_command_status),
    )

    window = MainWindow.__new__(MainWindow)
    deps = window._host_dependency_statuses()
    docker = next(dep for dep in deps if dep.key == 'docker')
    compose = next(dep for dep in deps if dep.key == 'docker_compose')

    assert not docker.installed
    assert 'Cannot connect to the Docker daemon' in docker.reason
    assert 'no Docker apt source' in docker.reason
    assert 'docker-compose-plugin: (none)' in docker.reason
    assert 'Snap Docker appears to be installed' in docker.reason
    assert 'docker service is not active: inactive' in docker.reason
    assert 'Current groups: rasputin sudo' in docker.reason
    assert not compose.installed
    assert 'docker compose version' in compose.reason
    assert 'docker-compose-plugin: (none)' in compose.reason


def test_wizard_start_setup_shows_progress_then_summary(tmp_path):
    app = QApplication.instance() or QApplication([])
    wizard = ImageSetupWizard(
        public_images=['ozkrelo/x_mobipick_labs:noetic-v1.2'],
        default_image='ozkrelo/x_mobipick_labs:noetic-v1.2',
        host_user='testuser',
        host_uid='1001',
        host_gid='1001',
        base_image='ozkrelo/x_mobipick_labs:noetic-v1.2',
        target_image='ozkrelo/x_mobipick_labs:host_user_from_1.2',
        workspace_names=[],
        configuration_paths=[],
        source_master_folder=str(tmp_path / 'master'),
        source_workspace_name='clean_mobipick_labs_ws',
        source_repository='https://github.com/DFKI-NI/mobipick_labs.git',
        source_branch='noetic',
        source_image='ozkrelo/x_mobipick_labs:host_user_from_1.2',
    )

    def start_setup(_selection):
        wizard.begin_setup()
        wizard.append_progress_html('<b>Running install</b>')
        wizard.complete_setup(
            success=True,
            summary_lines=['Installed source workspace.'],
        )
        return True

    wizard.set_setup_start_handler(start_setup)
    wizard.show()
    app.processEvents()
    while wizard.currentId() != wizard._source_page_id:
        wizard.next()
        app.processEvents()
    wizard.accept()
    wizard.progress_log._flush()

    assert wizard.currentId() == wizard._progress_page_id
    assert wizard.button(QWizard.NextButton).isEnabled()
    assert 'Running install' in wizard.progress_log.toHtml()

    wizard.next()

    assert wizard.currentId() == wizard._summary_page_id
    assert wizard.buttonText(QWizard.FinishButton) == 'Finish Setup'
    assert wizard.summary_edit.toPlainText() == 'Installed source workspace.'

    wizard.deleteLater()
    app.processEvents()


def test_setup_wizard_progress_labels_selected_step_count(tmp_path, monkeypatch):
    app = QApplication.instance() or QApplication([])
    wizard = ImageSetupWizard(
        public_images=['ozkrelo/x_mobipick_labs:noetic-v1.2'],
        default_image='ozkrelo/x_mobipick_labs:noetic-v1.2',
        host_user='testuser',
        host_uid='1001',
        host_gid='1001',
        base_image='ozkrelo/x_mobipick_labs:noetic-v1.2',
        target_image='ozkrelo/x_mobipick_labs:host_user_from_1.2',
        workspace_names=[],
        configuration_paths=[],
        source_master_folder=str(tmp_path / 'master'),
        source_workspace_name='clean_mobipick_labs_ws',
        source_repository='https://github.com/DFKI-NI/mobipick_labs.git',
        source_branch='noetic',
        source_image='ozkrelo/x_mobipick_labs:host_user_from_1.2',
    )
    window = MainWindow.__new__(MainWindow)
    window._setup_wizard_process_tabs = []

    monkeypatch.setattr(
        window,
        '_setup_wizard_process_tab',
        lambda *_args, **_kwargs: object(),
    )
    monkeypatch.setattr(
        window,
        '_load_available_images',
        lambda show_feedback=False: None,
    )
    monkeypatch.setattr(window, '_log_info', lambda _message: None)

    def finish_immediately(*_args, on_finished=None, **_kwargs):
        assert on_finished is not None
        on_finished(0)
        return True

    monkeypatch.setattr(window, '_start_image_pulls', finish_immediately)
    monkeypatch.setattr(window, '_start_custom_image_build', finish_immediately)
    monkeypatch.setattr(
        window,
        '_start_source_workspace_install',
        finish_immediately,
    )

    selection = SetupWizardSelection(
        pull_public_images=True,
        public_images=['ozkrelo/x_mobipick_labs:noetic-v1.2'],
        default_image='ozkrelo/x_mobipick_labs:noetic-v1.2',
        build_custom_image=True,
        host_user='testuser',
        host_uid='1001',
        host_gid='1001',
        base_image='ozkrelo/x_mobipick_labs:noetic-v1.2',
        target_image='ozkrelo/x_mobipick_labs:host_user_from_1.2',
        compatible_workspace='',
        remember_completion=True,
        install_source_workspace=True,
        source_master_folder=str(tmp_path / 'master'),
        source_workspace_name='clean_mobipick_labs_ws',
        source_repository='https://github.com/DFKI-NI/mobipick_labs.git',
        source_branch='noetic',
        source_image='ozkrelo/x_mobipick_labs:host_user_from_1.2',
    )

    window._run_setup_wizard_sequence(
        wizard,
        selection,
        pull_public_images_automatically=True,
    )
    for _attempt in range(10):
        app.processEvents()
        if wizard._setup_complete:
            break
    wizard.progress_log._flush()

    progress_html = wizard.progress_log.toHtml()
    assert 'Step 1/3:' in progress_html
    assert 'Step 2/3:' in progress_html
    assert 'Step 3/3:' in progress_html
    assert 'Step 1/3 finished with code 0' in progress_html
    assert 'Step 3/3 finished with code 0' in progress_html

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
            image='ozkrelo/x_mobipick_labs:gpt_ws_from_host_user',
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
                {'ref': 'ozkrelo/x_mobipick_labs:gpt_ws_from_host_user'},
            ],
            None,
        ),
    )
    monkeypatch.setattr(
        MainWindow,
        'update_sim_status_from_poll',
        lambda self, force=False: None,
    )
    monkeypatch.setattr(
        MainWindow,
        '_missing_host_dependencies',
        lambda self: [],
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
    monkeypatch.setattr(
        MainWindow,
        '_confirm_host_image_pull',
        lambda self, images: True,
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
        host_user='testuser',
        host_uid='1001',
        host_gid='1001',
        base_image='ozkrelo/x_mobipick_labs:noetic-v1.2',
        target_image='ozkrelo/x_mobipick_labs:gpt_ws_from_host_user',
        compatible_workspace='gpt_ws',
        remember_completion=True,
    )

    window._apply_setup_wizard(selection)

    updates = saved['updates']
    assert updates['setup_wizard']['completed'] is True
    assert updates['images']['default'] == 'ozkrelo/x_mobipick_labs:noetic-v1.1'
    assert updates['images']['blacklist'] == []
    assert any(
        profile.get('ref') == 'ozkrelo/x_mobipick_labs:gpt_ws_from_host_user'
        and profile.get('compatible_workspaces') == ['gpt_ws']
        for profile in updates['images']['profiles']
    )
    assert started['pulls'] == ['ozkrelo/x_mobipick_labs:noetic-v1.1']
    assert started['build'] is selection

    window.deleteLater()
    app.processEvents()


def test_setup_wizard_manual_pull_pauses_before_saving(
    tmp_path,
    monkeypatch,
):
    registry_path = tmp_path / 'workspaces.yaml'
    monkeypatch.setenv('MOBIPICK_WORKSPACE_CONFIG', str(registry_path))
    monkeypatch.setitem(CONFIG, 'images', copy.deepcopy(CONFIG['images']))
    monkeypatch.setattr(
        MainWindow,
        '_discover_filtered_image_records',
        lambda self: (
            [{'ref': 'ozkrelo/x_mobipick_labs:noetic-v1.1'}],
            None,
        ),
    )
    monkeypatch.setattr(
        MainWindow,
        'update_sim_status_from_poll',
        lambda self, force=False: None,
    )

    calls = {'saved': False, 'pulls': False, 'source': False}
    monkeypatch.setattr(
        main_window_module,
        'save_user_config_update',
        lambda updates: calls.__setitem__('saved', True),
    )
    monkeypatch.setattr(
        MainWindow,
        '_start_image_pulls',
        lambda self, images, **kwargs: calls.__setitem__('pulls', True),
    )
    monkeypatch.setattr(
        MainWindow,
        '_start_source_workspace_install',
        lambda self, selection: calls.__setitem__('source', True),
    )
    monkeypatch.setattr(
        MainWindow,
        '_confirm_manual_image_pull',
        lambda self, images: False,
    )

    app = QApplication.instance() or QApplication([])
    window = MainWindow(verbosity=1)
    window.poll_timer.stop()
    window._sigint_timer.stop()

    selection = SetupWizardSelection(
        pull_public_images=True,
        public_images=['ozkrelo/x_mobipick_labs:noetic-v1.1'],
        default_image='ozkrelo/x_mobipick_labs:noetic-v1.1',
        build_custom_image=False,
        host_user='testuser',
        host_uid='1001',
        host_gid='1001',
        base_image='ozkrelo/x_mobipick_labs:noetic-v1.1',
        target_image='ozkrelo/x_mobipick_labs:host_user_from_1.2',
        compatible_workspace='',
        remember_completion=True,
        install_source_workspace=True,
        source_master_folder=str(tmp_path / 'master'),
        source_workspace_name='clean_mobipick_labs_ws',
        source_repository='https://github.com/DFKI-NI/mobipick_labs.git',
        source_branch='noetic',
        source_image='ozkrelo/x_mobipick_labs:noetic-v1.1',
        activate_source_workspace=False,
        public_image_pull_mode='manual',
    )

    window._apply_setup_wizard(selection)

    assert calls == {'saved': False, 'pulls': False, 'source': False}

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
        host_user='testuser',
        host_uid='1001',
        host_gid='1001',
        base_image=source_image,
        target_image='ozkrelo/x_mobipick_labs:host_user_from_1.2',
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
    monkeypatch.setattr(
        MainWindow,
        '_missing_host_dependencies',
        lambda self: [],
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


def test_setup_wizard_auto_opens_when_host_dependency_is_missing(
    tmp_path,
    monkeypatch,
):
    registry_path = tmp_path / 'workspaces.yaml'
    monkeypatch.setenv('MOBIPICK_WORKSPACE_CONFIG', str(registry_path))
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
    monkeypatch.setattr(
        MainWindow,
        '_missing_host_dependencies',
        lambda self: [
            HostDependency(
                key='wmctrl',
                label='wmctrl',
                package='wmctrl',
                installed=False,
                reason='Optional window layout support.',
            )
        ],
    )

    app = QApplication.instance() or QApplication([])
    window = MainWindow(verbosity=1)
    window.poll_timer.stop()
    window._sigint_timer.stop()
    monkeypatch.setenv('QT_QPA_PLATFORM', 'xcb')

    assert window._should_auto_show_setup_wizard()

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
