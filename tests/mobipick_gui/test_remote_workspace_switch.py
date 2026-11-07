"""POST /workspace: remote workspace switch, behind remote_control.allow_workspace_switch."""
import os
from unittest.mock import MagicMock

os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')

import pytest

from mobipick_gui import remote_adapter
from mobipick_gui.main_window import MainWindow
from mobipick_gui.remote_adapter import MainWindowRemoteAdapter
from mobipick_gui.remote_control import RemoteControlError
from mobipick_gui.workspaces import RosWorkspace, WorkspaceRegistry

from test_remote_control import FakeAdapter, api_server, _Api  # noqa: F401


def _window(active='', running=False, images=('img:1',), known=('gpt_ws', 'amenable_ws')):
    window = MagicMock()
    registry = MagicMock()
    registry.active = active
    registry.get.side_effect = lambda n: MagicMock() if n in known else None
    registry.workspaces = []
    window._workspace_registry = registry
    window._workspace_processes_running.return_value = running
    window._workspace_match_image.return_value = 'img:1'
    window._workspace_image.return_value = ''
    window._image_choices = list(images)
    window._activate_workspace.return_value = True
    window.workspaces_status.return_value = {'active': 'gpt_ws', 'workspaces': []}
    return window


def test_switch_activates_without_asking():
    window = _window()
    result = MainWindow.switch_workspace_remote(window, 'gpt_ws')
    window._activate_workspace.assert_called_once_with('gpt_ws', ask=False)
    assert result['accepted'] and result['changed']


def test_switch_to_active_workspace_is_a_no_op():
    window = _window(active='gpt_ws')
    result = MainWindow.switch_workspace_remote(window, 'gpt_ws')
    window._activate_workspace.assert_not_called()
    assert result['accepted'] and not result['changed']


def test_switch_refuses_unknown_name_running_processes_and_missing_image():
    unknown = MainWindow.switch_workspace_remote(_window(), 'nope')
    assert not unknown['accepted'] and 'not configured' in unknown['reason']
    busy_window = _window(running=True)
    busy = MainWindow.switch_workspace_remote(busy_window, 'gpt_ws')
    assert not busy['accepted'] and 'stop running' in busy['reason']
    busy_window._activate_workspace.assert_not_called()
    no_image = MainWindow.switch_workspace_remote(_window(images=()), 'gpt_ws')
    assert not no_image['accepted'] and 'not installed' in no_image['reason']


def test_activate_workspace_ask_false_skips_the_question(monkeypatch):
    from mobipick_gui import main_window as mw
    question = MagicMock()
    monkeypatch.setattr(mw.popups, 'question', question)
    window = _window()
    window._image_choices = ['img:1']
    window._workspace_registry.get.side_effect = lambda n: MagicMock(directory='/x', name=n)
    window._reset_workspace_tabs = MagicMock()
    MainWindow._activate_workspace(window, 'gpt_ws', ask=False)
    question.assert_not_called()
    assert window._workspace_registry.active == 'gpt_ws'
    window._workspace_registry.save.assert_called_once_with()


def test_flag_is_off_in_code():
    from mobipick_gui.config import CONFIG_DEFAULTS
    assert CONFIG_DEFAULTS['remote_control']['allow_workspace_switch'] is False


def test_adapter_refuses_with_403_while_the_flag_is_off(monkeypatch):
    from mobipick_gui import config
    monkeypatch.setitem(config.CONFIG, 'remote_control', {'allow_workspace_switch': False})
    window = _window()
    adapter = MainWindowRemoteAdapter.__new__(MainWindowRemoteAdapter)
    adapter.window = window
    with pytest.raises(RemoteControlError) as err:
        adapter.set_workspace('gpt_ws')
    assert err.value.status == 403
    window.switch_workspace_remote.assert_not_called()


def test_adapter_switches_when_the_flag_is_on(monkeypatch):
    from mobipick_gui import config
    monkeypatch.setitem(config.CONFIG, 'remote_control', {'allow_workspace_switch': True})
    window = MagicMock()
    window.switch_workspace_remote.return_value = {'accepted': True}
    adapter = MainWindowRemoteAdapter.__new__(MainWindowRemoteAdapter)
    adapter.window = window
    assert adapter.set_workspace('gpt_ws') == {'accepted': True}
    window.switch_workspace_remote.assert_called_once_with('gpt_ws')


def test_http_endpoints(api_server):
    server, adapter, api = api_server
    adapter.workspaces = lambda: {'active': 'a', 'workspaces': [], 'processes_running': False}
    adapter.set_workspace = lambda n: {'accepted': True, 'active': n}
    status, body = api('GET', '/workspaces')
    assert status == 200 and body['workspaces']['active'] == 'a'
    status, body = api('POST', '/workspace', {'name': 'gpt_ws'})
    assert status == 200 and body['workspace']['active'] == 'gpt_ws'
    status, _ = api('POST', '/workspace', {})
    assert status == 400
    adapter.set_workspace = lambda n: {'accepted': False, 'reason': 'busy'}
    status, body = api('POST', '/workspace', {'name': 'gpt_ws'})
    assert status == 409 and 'busy' in str(body)


def test_save_failure_without_ask_returns_a_refusal_and_no_popup(monkeypatch):
    from mobipick_gui import main_window as mw
    critical = MagicMock()
    monkeypatch.setattr(mw.popups, 'critical', critical)
    window = _window()
    window._workspace_registry.save.side_effect = OSError('disk full')
    window._workspace_switch_error = ''
    window._activate_workspace = lambda n, ask=True: MainWindow._activate_workspace(window, n, ask=ask)
    result = MainWindow.switch_workspace_remote(window, 'gpt_ws')
    critical.assert_not_called()
    assert not result['accepted'] and 'disk full' in result['reason']


def test_event_only_when_changed(api_server):
    server, adapter, api = api_server
    adapter.set_workspace = lambda n: {'accepted': True, 'changed': False, 'active': n}
    api('POST', '/workspace', {'name': 'a'})
    assert not [e for e in server.events.since(0) if e['name'] == 'workspace_changed']
    adapter.set_workspace = lambda n: {'accepted': True, 'changed': True, 'active': n}
    api('POST', '/workspace', {'name': 'b'})
    assert [e['data']['workspace'] for e in server.events.since(0) if e['name'] == 'workspace_changed'] == ['b']
