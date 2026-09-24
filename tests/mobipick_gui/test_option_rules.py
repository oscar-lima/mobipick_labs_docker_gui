import os

import pytest
import yaml

os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')

from mobipick_gui.main_window import MainWindow
from mobipick_gui.option_rules import (
    load_option_rules,
    option_rules_path,
    parse_option_rules,
)

from mobipick_gui.remote_adapter import MainWindowRemoteAdapter
from test_remote_ros_master import _create_window

REAL_ROBOT_RULES = {
    'rules': [
        {
            'when': {'remote_master': True},
            'only': {'world': ['cic_tables']},
            'reason': 'the real robot only runs cic_tables',
        }
    ]
}
WORLDS = {'world': ['moelk_tables', 'cic_tables']}
BRINGUP_FIRST = {
    'when': {'remote_master': True, 'running.tables_demo_bringup': False},
    'block_start': 'all',
    'except': ['tables_demo_bringup'],
    'reason': 'start tables_demo_bringup first',
}


SNAPSHOT_REMINDER = {
    'when': {'remote_master': True},
    'remind_start': ['tables_demo_bringup'],
    'notice': 'launch rgbd_snapshot_server.py on the real robot',
    'clipboard': 'rgbd_snapshot_server',
}


def _record_popups(monkeypatch) -> list:
    popups = []
    monkeypatch.setattr(
        MainWindow,
        '_show_rule_popup',
        lambda self, title, message, *_icon: popups.append(
            (title, message)
        ),
    )
    return popups


def test_only_rule_invalidates_other_options_while_condition_holds():
    rules = parse_option_rules(REAL_ROBOT_RULES)

    assert rules.invalid_options({'remote_master': False}, WORLDS) == {}
    assert rules.invalid_options({'remote_master': True}, WORLDS) == {
        'world': {'moelk_tables': 'the real robot only runs cic_tables'}
    }


def test_invalid_rule_matches_any_listed_condition_value():
    rules = parse_option_rules(
        {
            'rules': [
                {
                    'when': {'model_profile': ['o3', 'o3-jev']},
                    'invalid': {'disc_mode': 'cpu'},
                }
            ]
        }
    )
    choices = {'disc_mode': ['mockup', 'cpu']}

    assert rules.invalid_options({'model_profile': 'jev'}, choices) == {}
    invalid = rules.invalid_options({'model_profile': 'o3-jev'}, choices)
    assert list(invalid) == ['disc_mode']
    assert list(invalid['disc_mode']) == ['cpu']
    assert 'model_profile=o3|o3-jev' in invalid['disc_mode']['cpu']


def test_malformed_rules_are_reported_and_skipped():
    rules = parse_option_rules(
        {'rules': [{'when': {'remote_master': True}}, 'bad', REAL_ROBOT_RULES['rules'][0]]}
    )

    assert len(rules.rules) == 1
    assert len(rules.errors) == 2


def test_block_start_spares_excepted_buttons_and_lifts_when_running():
    rules = parse_option_rules({'rules': [BRINGUP_FIRST]})
    waiting = {'remote_master': True, 'running.tables_demo_bringup': False}
    running = {'remote_master': True, 'running.tables_demo_bringup': True}

    assert rules.running_keys() == {'tables_demo_bringup'}
    assert rules.start_blocked(waiting, 'rviz') == (
        'start tables_demo_bringup first'
    )
    assert rules.start_blocked(waiting, 'tables_demo_bringup') is None
    assert rules.start_blocked(running, 'rviz') is None
    assert rules.start_blocked({'remote_master': False}, 'rviz') is None


def test_rules_file_is_found_beside_the_button_profile(tmp_path):
    profile = tmp_path / 'profile.yaml'
    profile.write_text('buttons: []\n')
    assert option_rules_path(profile) is None
    assert load_option_rules(profile).rules == []

    shared = tmp_path / 'option_rules.yaml'
    shared.write_text(yaml.safe_dump(REAL_ROBOT_RULES))
    assert option_rules_path(profile) == shared

    specific = tmp_path / 'profile_rules.yaml'
    specific.write_text(yaml.safe_dump(REAL_ROBOT_RULES))
    assert option_rules_path(profile) == specific
    assert len(load_option_rules(profile).rules) == 1


def test_remote_master_makes_moelk_tables_invalid(monkeypatch, tmp_path):
    popups = _record_popups(monkeypatch)
    app, window = _create_window(monkeypatch, tmp_path)
    try:
        window._option_rules = parse_option_rules(REAL_ROBOT_RULES)
        combo = window.world_combo
        combo.setCurrentIndex(combo.findText('moelk_tables'))
        moelk = combo.model().item(combo.findText('moelk_tables'))

        window.remote_master_checkbox.setChecked(True)

        assert window._current_world() == 'cic_tables'
        assert len(popups) == 1
        assert 'switched to cic_tables' in popups[0][1]
        assert not moelk.isEnabled()
        assert 'cic_tables' in moelk.toolTip()
        world = next(e for e in window.generic_args() if e['name'] == 'world')
        assert list(world['invalid']) == ['moelk_tables']
        with pytest.raises(ValueError, match='invalid'):
            window.set_generic_args({'world': 'moelk_tables'})

        window.remote_master_checkbox.setChecked(False)

        assert moelk.isEnabled()
        window.set_generic_args({'world': 'moelk_tables'})
        assert window._current_world() == 'moelk_tables'
    finally:
        window.close()
        app.processEvents()


def test_buttons_wait_for_bringup_on_the_real_robot(monkeypatch, tmp_path):
    popups = _record_popups(monkeypatch)
    app, window = _create_window(monkeypatch, tmp_path)
    try:
        window._option_rules = parse_option_rules({'rules': [BRINGUP_FIRST]})
        running = {'tables_demo_bringup': False}
        monkeypatch.setattr(
            window, '_is_button_running', lambda key: running.get(key, False)
        )
        started = []
        monkeypatch.setattr(window, 'toggle_rviz', lambda: started.append(1))
        adapter = MainWindowRemoteAdapter(window)

        window._on_config_button_clicked('rviz')
        assert started == [1]

        window.remote_master_checkbox.setChecked(True)
        window._on_config_button_clicked('rviz')
        assert started == [1]
        assert popups[-1][0] == 'Cannot start RViz'
        pressed = adapter.press_button('rviz', 'start')
        assert not pressed['accepted']
        assert 'tables_demo_bringup' in pressed['reason']

        running['tables_demo_bringup'] = True
        window._on_config_button_clicked('rviz')
        assert started == [1, 1]
    finally:
        window.close()
        app.processEvents()


def test_auto_launch_refuses_steps_the_rules_block(monkeypatch, tmp_path):
    popups = _record_popups(monkeypatch)
    app, window = _create_window(monkeypatch, tmp_path)
    try:
        window._option_rules = parse_option_rules({'rules': [BRINGUP_FIRST]})
        monkeypatch.setattr(window, '_is_button_running', lambda key: False)
        monkeypatch.setattr(
            window, '_confirm_workspace_mismatch_warning', lambda _label: True
        )
        launched = []
        monkeypatch.setattr(
            window, '_start_dependency_launch', launched.append
        )
        monkeypatch.setattr(window, '_schedule_recording_after_launch', lambda: None)
        window.remote_master_checkbox.setChecked(True)
        popups.clear()
        window._launch_plan = {
            'mode': 'advanced',
            'processes': [{'button': 'rviz'}],
        }

        # Repeated clicks keep refusing instead of turning the button green.
        for _attempt in range(2):
            window._on_auto_launch_toggle_clicked()
            assert launched == []
            assert not window._auto_launch_running
            assert popups[-1][0].startswith('Cannot start')
            assert 'RViz: start tables_demo_bringup first' in popups[-1][1]
        assert len(popups) == 2

        # A run that starts the bringup itself satisfies the rule.
        window._launch_plan = {
            'mode': 'advanced',
            'processes': [
                {'button': 'tables_demo_bringup'},
                {'button': 'rviz', 'depends_on': 'tables_demo_bringup'},
            ],
        }
        window._on_auto_launch_toggle_clicked()
        assert len(launched) == 1
        assert window._auto_launch_running
        assert len(popups) == 2
    finally:
        window._auto_launch_running = False
        window.close()
        app.processEvents()


def test_blocked_auto_launch_step_is_never_marked_ready(monkeypatch, tmp_path):
    app, window = _create_window(monkeypatch, tmp_path)
    try:
        window._option_rules = parse_option_rules({'rules': [BRINGUP_FIRST]})
        monkeypatch.setattr(window, '_is_button_running', lambda key: False)
        started = []
        monkeypatch.setattr(window, 'toggle_rviz', lambda: started.append(1))
        window.remote_master_checkbox.setChecked(True)
        window._auto_launch_running = True
        window._auto_launch_blocked_keys = set()

        assert window._dispatch_auto_launch_toggle('rviz') is False
        assert started == []
        assert window._auto_launch_blocked_keys == {'rviz'}

        window._schedule_auto_launch_ready('rviz', 0)
        for _ in range(5):
            app.processEvents()
        assert 'rviz' not in window._auto_launch_ready_keys
    finally:
        window._auto_launch_running = False
        window.close()
        app.processEvents()


def test_remind_start_needs_a_notice():
    rules = parse_option_rules(
        {'rules': [{'remind_start': 'all'}, SNAPSHOT_REMINDER]}
    )

    assert len(rules.rules) == 1
    assert 'notice' in rules.errors[0]
    assert rules.start_reminders({'remote_master': True}, 'rviz') == []
    assert rules.start_reminders(
        {'remote_master': True}, 'tables_demo_bringup'
    ) == [(SNAPSHOT_REMINDER['notice'], 'rgbd_snapshot_server')]
    assert rules.start_reminders(
        {'remote_master': False}, 'tables_demo_bringup'
    ) == []


def test_bringup_start_reminds_and_copies_to_clipboard(
    monkeypatch, tmp_path
):
    popups = _record_popups(monkeypatch)
    app, window = _create_window(monkeypatch, tmp_path)
    try:
        window._option_rules = parse_option_rules(
            {'rules': [BRINGUP_FIRST, SNAPSHOT_REMINDER]}
        )
        window._config_buttons['tables_demo_bringup'] = {
            'key': 'tables_demo_bringup',
            'label': 'Tables Demo Bringup',
            'kind': 'command',
        }
        running = {'tables_demo_bringup': False}
        monkeypatch.setattr(
            window, '_is_button_running', lambda key: running.get(key, False)
        )
        started = []
        monkeypatch.setattr(
            window, '_run_config_command', lambda config: started.append(1)
        )
        app.clipboard().setText('before')

        window._on_config_button_clicked('tables_demo_bringup')
        assert started == [1]
        assert popups == []
        assert app.clipboard().text() == 'before'

        window.remote_master_checkbox.setChecked(True)
        popups.clear()
        window._on_config_button_clicked('tables_demo_bringup')
        assert started == [1, 1]
        assert popups[-1][0] == 'Starting Tables Demo Bringup'
        assert 'rgbd_snapshot_server.py' in popups[-1][1]
        assert 'copied to the clipboard' in popups[-1][1]
        assert app.clipboard().text() == 'rgbd_snapshot_server'

        # Stopping a running bringup shows no reminder.
        running['tables_demo_bringup'] = True
        popups.clear()
        window._on_config_button_clicked('tables_demo_bringup')
        assert popups == []
    finally:
        window.close()
        app.processEvents()


KEYFRAME_ON_ROBOT = {
    'when': {'remote_master': True},
    'start_args': {'disc_ros': {'start_keyframe_node': False}},
    'reason': 'the keyframe node runs on the robot PC',
}


def test_start_args_apply_only_while_the_rule_matches():
    rules = parse_option_rules(
        {'rules': [KEYFRAME_ON_ROBOT, {'start_args': {'disc_ros': 'oops'}}]}
    )

    assert len(rules.rules) == 1
    assert 'start_args.disc_ros' in rules.errors[0]
    assert rules.start_args({'remote_master': True}, 'disc_ros') == [
        ('start_keyframe_node', 'false')
    ]
    assert rules.start_args({'remote_master': False}, 'disc_ros') == []
    assert rules.start_args({'remote_master': True}, 'rviz') == []


def test_start_args_reach_the_command_and_the_remote_api(monkeypatch, tmp_path):
    app, window = _create_window(monkeypatch, tmp_path)
    try:
        window._option_rules = parse_option_rules({'rules': [KEYFRAME_ON_ROBOT]})
        config = {
            'key': 'disc_ros',
            'label': 'DISC ROS',
            'kind': 'command',
            'command': 'roslaunch disc_mapping_ros disc_mapping_live.launch',
        }
        window._config_buttons['disc_ros'] = config

        assert MainWindow._command_with_generic_args(
            window, config['command'], config
        ) == config['command']

        window.remote_master_checkbox.setChecked(True)
        expected = f"{config['command']} start_keyframe_node:='false'"
        assert MainWindow._command_with_generic_args(
            window, config['command'], config
        ) == expected
        described = MainWindowRemoteAdapter(window)._describe_button('disc_ros')
        assert described['full_command'] == expected
    finally:
        window.close()
        app.processEvents()
