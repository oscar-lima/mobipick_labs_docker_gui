'''Headless mode: a remote press that would open a desktop window is refused unless allow_window.'''
import os

os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')

from PyQt5.QtWidgets import QApplication, QPushButton

import pytest

from mobipick_gui import remote_adapter
from mobipick_gui.headless import headless_window_refusal, remote_press_opens_window
from mobipick_gui.remote_adapter import MainWindowRemoteAdapter
from mobipick_gui.remote_client import _build_parser
from mobipick_gui.remote_control import DirectInvoker, RemoteControlServer


@pytest.fixture(autouse=True)
def no_docker(monkeypatch):
    monkeypatch.setattr(remote_adapter, '_running_containers_with_tab', lambda key: [])
    monkeypatch.setattr(remote_adapter, '_stop_containers', lambda names: [])


class FakeTab:
    running, exec_id, container_name = False, None, None

    def is_running(self):
        return self.running


class FakeWindow:
    '''what press_button and the window guard read; a click only records itself'''

    def __init__(self, headless, states=None):
        self._headless_mode = headless
        self.clicks = []
        self.logged = []
        self.set_args_calls = []
        self._config_buttons = {
            'sim': {'label': 'Sim', 'action': 'sim', 'headless_args': {'gui': 'false'}},
            'rqt': {'label': 'RQt (tables demo)', 'action': 'rqt_tables'},
            'viewer': {'label': 'Viewer', 'kind': 'command', 'command': 'x', 'opens_window': True},
        }
        self._config_button_order = list(self._config_buttons)
        self._toggle_states = {key: 'red' for key in [*self._config_buttons, 'terminal']}
        self._toggle_states.update(states or {})
        self._button_widgets = {}
        for key in [*self._config_buttons, 'terminal']:
            button = QPushButton(key)
            button.clicked.connect(lambda _=False, key=key: self.clicks.append(key))
            self._button_widgets[key] = button
        self.tasks = {}

    def _config_runs_on_host(self, config):
        return False

    def button_args(self, key):
        return {}

    def _command_with_generic_args(self, command, config):
        return command

    def button_readiness(self, key):
        return {'ready': False}

    def _start_blocked_reason(self, key):
        return None

    def _log_info(self, text):
        self.logged.append(text)

    def set_generic_args(self, values):
        self.set_args_calls.append(values)
        return []


_APP = None


def _app():
    global _APP  # keep the application alive: a collected QApplication aborts Qt
    _APP = QApplication.instance() or _APP or QApplication([])
    return _APP


def _server(headless, states=None):
    _app()
    window = FakeWindow(headless, states)
    adapter = MainWindowRemoteAdapter.__new__(MainWindowRemoteAdapter)  # no snapshot of a real window
    adapter.window = window
    server = RemoteControlServer(adapter, host='127.0.0.1', port=0, invoker=DirectInvoker())
    return server, window


def test_window_buttons_include_profile_windows_and_the_terminal():
    assert remote_press_opens_window('rqt', {'action': 'rqt_tables'})
    assert remote_press_opens_window('rviz', {'action': 'rviz'})
    assert remote_press_opens_window('viewer', {'kind': 'command', 'opens_window': True})
    assert remote_press_opens_window('terminal', {})
    assert not remote_press_opens_window('sim', {'action': 'sim'})
    assert not remote_press_opens_window('roscore', {})


def test_refusal_says_why_and_what_to_do():
    text = headless_window_refusal('rqt', 'RQt (tables demo)')
    assert 'headless mode is on' in text and 'RQt (tables demo) (rqt)' in text
    assert 'virtual display' in text and '"allow_window": true' in text


@pytest.mark.parametrize('key', ['rqt', 'viewer', 'terminal'])
@pytest.mark.parametrize('action', ['click', 'start'])
def test_headless_refuses_remote_window_press(key, action):
    server, window = _server(headless=True)
    status, payload = server.handle(
        'POST', f'/buttons/{key}/{action}', {}, {'args': {'world': 'x'}}
    )
    assert status == 409
    assert payload['ok'] is False and payload['accepted'] is False
    assert payload['refused'] == 'headless_window'
    assert 'allow_window' in payload['error'] and 'virtual display' in payload['error']
    assert payload['button']['key'] == key
    _app().processEvents()
    assert window.clicks == []
    assert window.set_args_calls == []  # a refused press changes nothing


def test_allow_window_presses_as_before():
    server, window = _server(headless=True)
    status, payload = server.handle('POST', '/buttons/rqt/start', {}, {'allow_window': True})
    assert status == 200 and payload['accepted'] is True
    assert window.clicks == ['rqt']


def test_non_window_buttons_and_headless_off_are_not_refused():
    server, window = _server(headless=True)
    status, payload = server.handle('POST', '/buttons/sim/start', {}, {})
    assert status == 200 and payload['accepted'] is True

    server, window = _server(headless=False)
    status, payload = server.handle('POST', '/buttons/rqt/click', {}, {})
    assert status == 200 and payload['accepted'] is True
    assert window.clicks == ['rqt']


def test_stopping_a_running_window_is_never_refused():
    server, window = _server(headless=True, states={'rqt': 'green', 'viewer': 'green'})
    status, payload = server.handle('POST', '/buttons/rqt/stop', {}, {})
    assert status == 200 and payload['accepted'] is True
    status, payload = server.handle('POST', '/buttons/viewer/click', {}, {})  # click on running = stop
    assert status == 200 and payload['accepted'] is True
    assert window.clicks == ['rqt', 'viewer']


def test_a_mouse_click_is_never_blocked():
    _server_, window = _server(headless=True)
    window._button_widgets['rqt'].click()
    assert window.clicks == ['rqt']


def test_cli_passes_allow_window_only_when_asked():
    parser = _build_parser()
    assert parser.parse_args(['click', 'rqt', '--allow-window']).allow_window is True
    assert parser.parse_args(['start', 'rqt']).allow_window is False
    assert not hasattr(parser.parse_args(['stop', 'rqt']), 'allow_window')


def test_endpoint_help_documents_allow_window():
    server, _window = _server(headless=False)
    _status, payload = server.handle('GET', '/', {}, {})
    click = next(
        entry for entry in payload['endpoints']
        if entry['method'] == 'POST' and entry['path'] == '/buttons/{key}/click'
    )
    text = str(click)
    assert 'allow_window' in text and 'headless' in text
