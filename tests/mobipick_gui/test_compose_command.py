import os
import subprocess

os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')

from PyQt5.QtWidgets import QApplication

from mobipick_gui.config import CONFIG
import mobipick_gui.main_window as main_window_module
from mobipick_gui.main_window import MainWindow
from mobipick_gui.process_tab import ProcessTab


def _completed(args, returncode=0, stdout='', stderr=''):
    return subprocess.CompletedProcess(args, returncode, stdout, stderr)


def test_compose_command_prefers_plugin_when_run_rm_is_supported(monkeypatch):
    calls = []

    def fake_which(name):
        return f'/usr/bin/{name}' if name == 'docker' else None

    def fake_run(args, **_kwargs):
        calls.append(tuple(args))
        if args == ['docker', 'compose', 'version']:
            return _completed(args, stdout='Docker Compose version v2.27.0\n')
        if args == ['docker', 'compose', 'run', '--help']:
            return _completed(args, stdout='Usage: docker compose run [--rm]\n')
        raise AssertionError(f'unexpected command: {args}')

    monkeypatch.setattr(main_window_module.shutil, 'which', fake_which)
    monkeypatch.setattr(main_window_module.subprocess, 'run', fake_run)

    assert MainWindow._compose_command('run', '--rm', 'mobipick_cmd') == [
        'docker',
        'compose',
        'run',
        '--rm',
        'mobipick_cmd',
    ]
    assert calls == [
        ('docker', 'compose', 'version'),
        ('docker', 'compose', 'run', '--help'),
    ]


def test_compose_command_falls_back_to_standalone_when_plugin_lacks_rm(
    monkeypatch,
):
    def fake_which(name):
        if name in {'docker', 'docker-compose'}:
            return f'/usr/bin/{name}'
        return None

    def fake_run(args, **_kwargs):
        if args == ['docker', 'compose', 'version']:
            return _completed(args, stdout='Docker Compose version v2.0.0\n')
        if args == ['docker', 'compose', 'run', '--help']:
            return _completed(args, stdout='Usage without remove flag\n')
        if args == ['docker-compose', 'version']:
            return _completed(args, stdout='docker-compose version 1.29.2\n')
        if args == ['docker-compose', 'run', '--help']:
            return _completed(args, stdout='Usage: docker-compose run --rm\n')
        raise AssertionError(f'unexpected command: {args}')

    monkeypatch.setattr(main_window_module.shutil, 'which', fake_which)
    monkeypatch.setattr(main_window_module.subprocess, 'run', fake_run)

    assert MainWindow._compose_command('run', '--rm', 'mobipick_cmd') == [
        'docker-compose',
        'run',
        '--rm',
        'mobipick_cmd',
    ]
    assert MainWindow._docker_compose_available()


def test_roscore_launch_uses_resolved_compose_frontend(tmp_path, monkeypatch):
    monkeypatch.setenv(
        'MOBIPICK_WORKSPACE_CONFIG',
        str(tmp_path / 'workspaces.yaml'),
    )
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
    monkeypatch.setattr(MainWindow, '_missing_host_dependencies', lambda self: [])
    monkeypatch.setattr(
        MainWindow,
        '_compose_command',
        classmethod(lambda cls, *args: ['docker-compose', *args]),
    )
    monkeypatch.setattr(
        MainWindow,
        '_ensure_network',
        lambda self, log_key=None: None,
    )
    monkeypatch.setattr(
        MainWindow,
        '_schedule_host_to_container_copy',
        lambda self, tab, attempt=0: None,
    )
    started = []

    def fake_start_program(self, program, args):
        started.append((self.key, program, args))

    monkeypatch.setattr(ProcessTab, 'start_program', fake_start_program)

    app = QApplication.instance() or QApplication([])
    window = MainWindow(verbosity=1)
    window.poll_timer.stop()
    window._sigint_timer.stop()

    window.bring_up_roscore()

    assert started
    key, program, args = started[-1]
    assert key == 'roscore'
    assert program == 'docker-compose'
    assert args[:4] == ['run', '--rm', '--name', 'mobipick-roscore']
    assert '-e' in args
    assert '--env' not in args
    assert 'MOBIPICK_WORKSPACE_ENABLED=0' in args

    window.deleteLater()
    app.processEvents()
