import os
import subprocess

os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')

from PyQt5.QtCore import Qt
from PyQt5.QtWidgets import QApplication, QDialog

from mobipick_gui import main_window as main_window_module
from mobipick_gui.main_window import DockerCpContainerPathDialog, MainWindow


def _window_for_browser_tests():
    window = MainWindow.__new__(MainWindow)
    window._console_log = lambda *_args, **_kwargs: None
    window._docker_cp_container_command = (
        lambda container_ref, script: ['docker-cp-browser', container_ref, script]
    )
    return window


def test_docker_cp_running_container_path_browser_starts_at_default_path(
    monkeypatch,
):
    window = _window_for_browser_tests()
    seen = {}

    class FakePathDialog:
        def __init__(self, *, container_ref, start_path, list_provider):
            seen['container_ref'] = container_ref
            seen['start_path'] = start_path
            seen['list_provider'] = list_provider

        def exec_(self):
            return QDialog.Rejected

    monkeypatch.setattr(
        main_window_module,
        'DockerCpContainerPathDialog',
        FakePathDialog,
    )

    result = MainWindow._docker_cp_container_path_from_setup(
        window,
        'container-id',
        '/root/catkin_ws/src/demo/config/file.rviz',
    )

    assert result == '/root/catkin_ws/src/demo/config/file.rviz'
    assert seen['container_ref'] == 'container-id'
    assert seen['start_path'] == '/root/catkin_ws/src/demo/config/file.rviz'
    assert seen['list_provider'] == window._docker_cp_list_container_paths


def test_docker_cp_image_path_browser_starts_at_root(monkeypatch):
    window = _window_for_browser_tests()
    seen = {}

    class FakePathDialog:
        def __init__(self, *, container_ref, start_path, list_provider):
            seen['container_ref'] = container_ref
            seen['start_path'] = start_path
            seen['list_provider'] = list_provider

        def exec_(self):
            return QDialog.Rejected

    monkeypatch.setattr(
        main_window_module,
        'DockerCpContainerPathDialog',
        FakePathDialog,
    )

    result = MainWindow._docker_cp_container_path_from_setup(
        window,
        '__image__:example:gpt',
        '/root/catkin_ws/src/demo/config/file.rviz',
    )

    assert result == '/root/catkin_ws/src/demo/config/file.rviz'
    assert seen['container_ref'] == '__image__:example:gpt'
    assert seen['start_path'] == '/'
    assert seen['list_provider'] == window._docker_cp_list_container_paths


def test_docker_cp_container_listing_parses_portable_shell_output(monkeypatch):
    window = _window_for_browser_tests()

    def fake_run(args, **kwargs):
        assert args[0] == 'docker-cp-browser'
        script = args[-1]
        assert 'find "$dir"' not in script
        assert 'for item in "$dir"/*' in script
        return subprocess.CompletedProcess(
            args,
            0,
            stdout='__DIR__\t/root/ws\n'
            'd\tsrc\t/root/ws/src\n'
            'f\tREADME.md\t/root/ws/README.md\n',
            stderr='',
        )

    monkeypatch.setattr(main_window_module.subprocess, 'run', fake_run)

    entries = MainWindow._docker_cp_list_container_paths(
        window,
        'container-id',
        '/root/ws',
    )

    assert entries == [
        {'name': '..', 'path': '/root', 'is_dir': True},
        {'name': 'src', 'path': '/root/ws/src', 'is_dir': True},
        {'name': 'README.md', 'path': '/root/ws/README.md', 'is_dir': False},
    ]


def test_docker_cp_container_listing_returns_visible_error(monkeypatch):
    window = _window_for_browser_tests()

    def fake_run(args, **kwargs):
        return subprocess.CompletedProcess(
            args,
            125,
            stdout='',
            stderr='docker: image requires a newer runtime',
        )

    monkeypatch.setattr(main_window_module.subprocess, 'run', fake_run)

    entries = MainWindow._docker_cp_list_container_paths(
        window,
        '__image__:example:gpt',
        '/',
    )

    assert entries == [
        {
            'error': (
                'Unable to read container path /: '
                'docker: image requires a newer runtime'
            ),
        },
    ]


def test_docker_cp_image_browser_runs_as_root():
    window = MainWindow.__new__(MainWindow)
    window._docker_cp_workspace_browser_env = lambda: {}

    command = MainWindow._docker_cp_container_command(
        window,
        '__image__:example:gpt',
        'ls /',
    )

    assert command[:7] == [
        'docker',
        'run',
        '--rm',
        '--user',
        'root',
        '--entrypoint',
        'bash',
    ]


def test_docker_cp_container_path_dialog_marks_empty_directory():
    app = QApplication.instance() or QApplication([])
    dialog = DockerCpContainerPathDialog(
        container_ref='container-id',
        start_path='/empty',
        list_provider=lambda _container_ref, _path: [],
    )

    assert dialog.entries.count() == 1
    assert dialog.entries.item(0).text() == 'No files found in this directory.'
    assert not dialog.entries.item(0).flags() & Qt.ItemIsEnabled

    dialog.deleteLater()
    app.processEvents()
