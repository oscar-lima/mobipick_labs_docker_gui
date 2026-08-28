from types import MethodType, SimpleNamespace

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
