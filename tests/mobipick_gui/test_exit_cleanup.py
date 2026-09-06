from types import MethodType, SimpleNamespace

from PyQt5.QtCore import QProcess, QProcessEnvironment
from PyQt5.QtWidgets import QApplication, QMainWindow

from mobipick_gui.config import SCRIPT_CLEAN
from mobipick_gui.main_window import MainWindow


def test_exit_cleanup_runs_all_button_stop_commands_in_reverse_order():
    harness = SimpleNamespace(
        _config_button_order=[
            'first',
            'without-stop',
            'container-command',
            'last',
        ],
        _config_buttons={
            'first': {
                'kind': 'command',
                'host': True,
                'stop_command': 'docker compose stop first',
            },
            'without-stop': {
                'kind': 'command',
                'host': True,
                'stop_command': '  ',
            },
            'container-command': {
                'kind': 'command',
                'host': False,
                'stop_command': 'stop-container-command',
            },
            'last': {
                'kind': 'command',
                'host': True,
                'stop_command': 'stop-last',
                'pass_ros_master_uri': True,
            },
        },
        _current_master_uri=lambda: 'http://robot:11311',
        _config_runs_on_host=MainWindow._config_runs_on_host,
        _neutralize_compose_ignore=MainWindow._neutralize_compose_ignore,
        _sh_quote=MainWindow._sh_quote,
        _sim_container_name='sim-container',
        _collect_container_commands=lambda *args, **kwargs: [['stop-sim']],
        _stop_all_related=lambda tab: [['stop-related']],
        _cleanup_done=False,
        _cleanup_script_available=lambda: True,
    )
    harness._prepared_config_stop_command = MethodType(
        MainWindow._prepared_config_stop_command,
        harness,
    )
    harness._collect_exit_commands = MethodType(
        MainWindow._collect_exit_commands,
        harness,
    )

    commands = harness._collect_exit_commands()

    assert commands[:3] == [
        [
            'bash',
            '-lc',
            'COMPOSE_IGNORE_ORPHANS= '
            "ROS_MASTER_URI='http://robot:11311' stop-last",
        ],
        ['bash', '-lc', 'stop-container-command'],
        ['bash', '-lc', 'COMPOSE_IGNORE_ORPHANS= docker compose stop first'],
    ]
    assert commands[3:] == [['stop-sim'], ['stop-related'], [SCRIPT_CLEAN]]


def test_exit_cleanup_cancels_background_sequence_before_deleting_process():
    app = QApplication.instance() or QApplication([])
    window = QMainWindow()
    window._bg_procs = []
    window._build_process_environment = (
        lambda _env: QProcessEnvironment.systemEnvironment()
    )
    window._log_cmd = lambda _command: None
    window._append_command_output = lambda _key, _data: None
    window._append_log_html = lambda _message: None
    window._console_log = lambda _level, _message: None
    window._fmt_args = lambda command: ' '.join(command)
    window._is_clean_command = lambda _command: False

    MainWindow._run_command_sequence(
        window,
        [['bash', '-lc', 'sleep 5']],
        log_key='log',
    )
    process = window._bg_procs[0]
    assert process.waitForStarted(1000)

    MainWindow._cancel_background_process(window, process)

    assert process.state() == QProcess.NotRunning
    assert process not in window._bg_procs
    app.processEvents()
    window.deleteLater()
