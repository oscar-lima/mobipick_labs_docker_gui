'''#79/#61: a remote "start" must never stop a half-alive run, and "stop" must still reach it (no docker, no port).'''
import os

os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')

from PyQt5.QtWidgets import QApplication, QPushButton

from mobipick_gui.remote_adapter import MainWindowRemoteAdapter


class FakeTab:
    def __init__(self, running=False, exec_id=None, container_name=None):
        self.running, self.exec_id, self.container_name = running, exec_id, container_name

    def is_running(self):
        return self.running


class FakeWindow:
    '''just what press_button reads; a click only records itself (the real handler toggles on the process)'''

    def __init__(self, state, tab):
        self.clicks = []
        self._config_buttons = {'gpt_robot_demo': {'label': 'GPT Robot Demo', 'kind': 'command', 'command': 'x'}}
        self._config_button_order = ['gpt_robot_demo']
        self._toggle_states = {'gpt_robot_demo': state}
        button = QPushButton('Start GPT Robot Demo')
        button.clicked.connect(lambda: self.clicks.append('click'))
        self._button_widgets = {'gpt_robot_demo': button}
        self.tasks = {'gpt_robot_demo': tab}

    def _config_runs_on_host(self, config):
        return False

    def button_args(self, key):
        return {}

    def _command_with_generic_args(self, command, config):
        return command

    def button_readiness(self, key):
        return {'ready': self._toggle_states.get(key) == 'green'}

    def _start_blocked_reason(self, key):
        return None

    def _log_info(self, text):
        pass


def _press(state, tab, action):
    app = QApplication.instance() or QApplication([])
    window = FakeWindow(state, tab)
    adapter = MainWindowRemoteAdapter.__new__(MainWindowRemoteAdapter)   # no status snapshot of a real window
    adapter.window = window
    result = adapter.press_button('gpt_robot_demo', action)
    app.processEvents()
    return result, window.clicks


def test_start_refused_while_the_old_client_still_runs():
    result, clicks = _press('red', FakeTab(running=True), 'start')
    assert not result['accepted'] and result['reason'].startswith('busy:')
    assert clicks == []                                   # the click would have stopped it


def test_start_refused_while_a_container_is_still_attached():
    result, clicks = _press('red', FakeTab(exec_id='abc123', container_name='mpcmd-abc123'), 'start')
    assert not result['accepted'] and 'mpcmd-abc123' in result['reason']
    assert clicks == []


def test_stop_reaches_a_half_alive_run():
    result, clicks = _press('red', FakeTab(running=True), 'stop')
    assert result['accepted'] and clicks == ['click']


def test_clean_red_button_starts_and_refuses_stop():
    result, clicks = _press('red', FakeTab(), 'start')
    assert result['accepted'] and clicks == ['click']
    result, clicks = _press('red', FakeTab(), 'stop')
    assert not result['accepted'] and result['reason'] == 'not running' and clicks == []


def test_green_button_unchanged():
    result, clicks = _press('green', FakeTab(running=True), 'start')
    assert not result['accepted'] and result['reason'] == 'already running'
    result, clicks = _press('green', FakeTab(running=True), 'stop')
    assert result['accepted'] and clicks == ['click']
