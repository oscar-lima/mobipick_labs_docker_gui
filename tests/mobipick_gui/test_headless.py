"""Headless mode: no windows from Auto Launch and headless button args."""

from types import SimpleNamespace

import yaml

from mobipick_gui.config import load_button_layout, save_button_layout
from mobipick_gui.headless import (
    button_opens_window,
    headless_launch_entries,
    normalize_headless_args,
)
from mobipick_gui.main_window import MainWindow
from mobipick_gui.remote_client import _build_parser
from mobipick_gui.remote_control import (
    DirectInvoker,
    GuiAdapter,
    RemoteControlServer,
)


def test_builtin_rviz_and_rqt_open_windows_unless_the_profile_says_otherwise():
    assert button_opens_window({'action': 'rviz'})
    assert button_opens_window({'action': 'rqt_tables'})
    assert not button_opens_window({'action': 'sim'})
    assert not button_opens_window({'kind': 'command'})
    assert button_opens_window({'kind': 'command', 'opens_window': True})
    assert not button_opens_window({'action': 'rviz', 'opens_window': False})


def test_headless_args_accept_yaml_booleans():
    assert normalize_headless_args({'gui': False, 'rate': 5, '': 'x', 'n': None}) == {
        'gui': 'false',
        'rate': '5',
    }
    assert normalize_headless_args(['gui:=false']) == {}


def test_headless_launch_plan_skips_windows_and_relinks_dependencies():
    plan = [
        {'button': 'roscore', 'depends_on': ''},
        {'button': 'sim', 'depends_on': 'roscore'},
        {'button': 'rviz', 'depends_on': 'sim'},
        {'button': 'viewer', 'depends_on': 'rviz'},
        {'button': 'tool', 'depends_on': 'viewer'},
        {'button': 'gpt', 'depends_on': 'sim'},
    ]
    kept, skipped = headless_launch_entries(plan, {'rviz', 'viewer'})

    assert skipped == ['rviz', 'viewer']
    assert [(e['button'], e['depends_on']) for e in kept] == [
        ('roscore', ''),
        ('sim', 'roscore'),
        ('tool', 'sim'),
        ('gpt', 'sim'),
    ]
    # the caller's plan is not modified
    assert plan[4]['depends_on'] == 'viewer'


def test_timeline_entries_without_dependencies_are_filtered():
    kept, skipped = headless_launch_entries(
        [{'button': 'sim', 'at_seconds': 0}, {'button': 'rviz', 'at_seconds': 5}],
        {'rviz'},
    )
    assert kept == [{'button': 'sim', 'at_seconds': 0}]
    assert skipped == ['rviz']


def test_profile_keeps_opens_window_and_headless_args(tmp_path):
    source = tmp_path / 'buttons.yaml'
    source.write_text(yaml.safe_dump({'buttons': [
        {'key': 'sim', 'kind': 'builtin', 'action': 'sim', 'headless_args': {'gui': False}},
        {'key': 'rviz', 'kind': 'builtin', 'action': 'rviz'},
        {'key': 'viewer', 'kind': 'command', 'command': 'viewer', 'opens_window': True},
    ]}))
    entries = {entry['key']: entry for entry in load_button_layout(source)}
    assert entries['sim']['headless_args'] == {'gui': 'false'}
    assert entries['viewer']['opens_window'] is True
    assert entries['rviz']['opens_window'] is None

    saved_path = tmp_path / 'saved.yaml'
    save_button_layout(saved_path, list(entries.values()))
    saved = {entry['key']: entry for entry in yaml.safe_load(saved_path.read_text())['buttons']}
    assert saved['sim']['headless_args'] == {'gui': 'false'}
    assert saved['viewer']['opens_window'] is True
    assert 'opens_window' not in saved['rviz']
    assert 'headless_args' not in saved['rviz']


class _Combo:
    def __init__(self, text):
        self._text = text

    def currentText(self):
        return self._text


def _command_harness(headless: bool):
    return SimpleNamespace(
        _generic_arg_inputs={7: _Combo('true')},
        _headless_mode=headless,
        _option_rules=None,
        _sh_quote=MainWindow._sh_quote,
    )


def test_headless_args_replace_the_dropdown_value_and_add_the_rest():
    config = {
        'key': 'sim',
        'arg_7_name': 'gui',
        'arg_7_applies': True,
        'headless_args': {'gui': 'false', 'rviz': 'false'},
    }
    headless = MainWindow._command_with_generic_args(
        _command_harness(True), 'roslaunch demo sim.launch', config
    )
    assert headless == "roslaunch demo sim.launch gui:='false' rviz:='false'"
    normal = MainWindow._command_with_generic_args(
        _command_harness(False), 'roslaunch demo sim.launch', config
    )
    assert normal == "roslaunch demo sim.launch gui:='true'"


def test_set_headless_reports_skipped_buttons_and_args():
    logged = []
    harness = SimpleNamespace(
        _headless_mode=False,
        _log_info=logged.append,
        _config_buttons={
            'sim': {'action': 'sim', 'headless_args': {'gui': 'false'}},
            'rviz': {'action': 'rviz'},
            'viewer': {'kind': 'command', 'opens_window': True},
        },
    )
    harness._window_button_keys = lambda: MainWindow._window_button_keys(harness)

    status = MainWindow.set_headless(harness, True)

    assert harness._headless_mode is True
    assert status == {
        'enabled': True,
        'skipped_by_auto_launch': ['rviz', 'viewer'],
        'headless_args': {'sim': {'gui': 'false'}},
    }
    assert logged == ['headless mode on: no windows from the next launches']


class _HeadlessAdapter(GuiAdapter):
    def __init__(self):
        self.enabled = False

    def headless(self):
        return {'enabled': self.enabled, 'skipped_by_auto_launch': ['rviz'], 'headless_args': {}}

    def set_headless(self, enabled):
        self.enabled = enabled
        return self.headless()


def test_headless_endpoint_switches_mode_and_emits_event():
    adapter = _HeadlessAdapter()
    server = RemoteControlServer(adapter, host='127.0.0.1', port=0, invoker=DirectInvoker())

    status, payload = server.handle('GET', '/headless', {}, {})
    assert status == 200 and payload['headless']['enabled'] is False

    status, payload = server.handle('POST', '/headless', {}, {'enabled': True})
    assert status == 200 and payload['headless']['enabled'] is True
    assert adapter.enabled is True
    assert server.events.since(0, ['headless_changed'])[-1]['data']['enabled'] is True

    status, payload = server.handle('POST', '/headless', {}, {})
    assert status == 400


def test_headless_endpoint_is_listed_and_in_the_cli():
    _status, payload = RemoteControlServer(
        _HeadlessAdapter(), host='127.0.0.1', port=0
    ).handle('GET', '/', {}, {})
    paths = {(entry['method'], entry['path']) for entry in payload['endpoints']}
    assert {('GET', '/headless'), ('POST', '/headless')} <= paths
    assert 'headless_changed' in payload['events']
    assert _build_parser().parse_args(['headless', 'on']).state == 'on'
    assert _build_parser().parse_args(['headless']).state is None
