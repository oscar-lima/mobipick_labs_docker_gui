import json
import os
import shutil
import threading
import time
from urllib import error as urlerror
from urllib import request as urlrequest

import pytest

os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')

from mobipick_gui.cli import _build_parser, remote_control_overrides
from mobipick_gui.remote_control import (
    Conflict,
    DirectInvoker,
    EventBus,
    GuiAdapter,
    RemoteControlServer,
    RemoteShellSession,
    build_exec_line,
    clean_output_line,
    remote_control_settings,
)

BASH = shutil.which('bash')
pytestmark_bash = pytest.mark.skipif(BASH is None, reason='bash is required')


# ---------------------------------------------------------------------------
# helpers
# ---------------------------------------------------------------------------


def _local_shell(**kwargs) -> RemoteShellSession:
    return RemoteShellSession(
        kwargs.pop('session_id', 1),
        kwargs.pop('name', 'test'),
        [BASH, '--noprofile', '--norc'],
        **kwargs,
    )


def _run(session: RemoteShellSession, command: str, timeout: float = 10.0):
    record = session.run(command)
    assert session.wait(record, timeout), f'command timed out: {command}'
    lines, _, _ = session.lines(since=record.first_seq, until=record.last_seq)
    return record, [entry['text'] for entry in lines]


class FakeAdapter(GuiAdapter):
    """Stand-in for the MainWindow adapter used by HTTP tests."""

    def __init__(self):
        self.clicks = []
        self.mirrored = []
        self.exited = []
        self.states = {'roscore': 'red', 'sim': 'red', 'busy': 'yellow'}
        self.dialog = None
        self.quit_called = False
        self.server = None

    def status(self):
        return {'workspace': 'ws', 'buttons': self.buttons(), 'tabs': self.tabs()}

    def buttons(self):
        return [
            {
                'key': key,
                'state': state,
                'running': state == 'green',
                'busy': state == 'yellow',
                'enabled': state != 'yellow',
                'text': key,
            }
            for key, state in self.states.items()
        ]

    def press_button(self, key, action):
        from mobipick_gui.remote_control import NotFound

        if key not in self.states:
            raise NotFound(f'unknown button {key}')
        if self.states[key] == 'yellow':
            return {'accepted': False, 'reason': 'busy'}
        if action == 'start' and self.states[key] == 'green':
            return {'accepted': False, 'reason': 'already running'}
        self.clicks.append((key, action))
        self.states[key] = 'red' if self.states[key] == 'green' else 'green'
        if self.server is not None:
            self.server.emit('button_state', key=key, state=self.states[key])
        return {'accepted': True, 'action': action}

    def tabs(self):
        return [{'key': 'log', 'running': False}]

    def tab_text(self, key):
        from mobipick_gui.remote_control import NotFound

        if key != 'log':
            raise NotFound('unknown tab')
        return 'first line\nsecond ERROR line\nthird line\n'

    def active_dialog(self):
        return self.dialog

    def dismiss_dialog(self, button):
        dismissed = self.dialog is not None
        self.dialog = None
        return {'dismissed': dismissed, 'button': button}

    def run_gui_command(self, command):
        return {'accepted': True, 'tab': 'custom1', 'command': command}

    def shell_spec(self, session_id, label, *, root):
        return {
            'argv': [BASH, '--noprofile', '--norc'],
            'env': dict(os.environ),
            'cwd': None,
            'container_name': None,
            'tab_key': f'terminal-remote{session_id}',
            'init_command': 'export MPRC_TEST=1',
        }

    def mirror_shell_output(self, session, text):
        self.mirrored.append(text)

    def shell_session_exited(self, session):
        self.exited.append(session.id)

    def quit(self):
        self.quit_called = True


class _Api:
    def __init__(self, base_url, token=''):
        self.base_url = base_url
        self.token = token

    def __call__(self, method, path, body=None, token=None, timeout=30):
        data = json.dumps(body).encode() if body is not None else None
        headers = {'Content-Type': 'application/json'}
        use_token = self.token if token is None else token
        if use_token:
            headers['Authorization'] = f'Bearer {use_token}'
        req = urlrequest.Request(self.base_url + path, data=data, method=method, headers=headers)
        try:
            with urlrequest.urlopen(req, timeout=timeout) as response:
                return response.status, json.loads(response.read().decode())
        except urlerror.HTTPError as exc:
            return exc.code, json.loads(exc.read().decode())

    def stream(self, path, timeout=30):
        req = urlrequest.Request(self.base_url + path)
        if self.token:
            req.add_header('Authorization', f'Bearer {self.token}')
        with urlrequest.urlopen(req, timeout=timeout) as response:
            return [json.loads(line) for line in response if line.strip()]


@pytest.fixture
def api_server():
    adapter = FakeAdapter()
    server = RemoteControlServer(
        adapter,
        host='127.0.0.1',
        port=0,
        invoker=DirectInvoker(),
        default_exec_timeout=10.0,
        max_output_lines=50,
    )
    adapter.server = server
    host, port = server.start()
    try:
        yield server, adapter, _Api(f'http://{host}:{port}')
    finally:
        server.stop()


# ---------------------------------------------------------------------------
# event bus
# ---------------------------------------------------------------------------


def test_event_bus_sequences_and_wait():
    bus = EventBus()
    assert bus.last_seq == 0
    first = bus.emit('a', value=1)
    second = bus.emit('b', value=2)
    assert first['seq'] == 1 and second['seq'] == 2
    assert [e['name'] for e in bus.since(0)] == ['a', 'b']
    assert [e['name'] for e in bus.since(1)] == ['b']
    assert [e['name'] for e in bus.since(0, names=['a'])] == ['a']
    assert bus.wait(['b'], since=1, timeout=0.1)['name'] == 'b'
    assert bus.wait(['b'], since=2, timeout=0.05) is None

    def emit_later():
        time.sleep(0.05)
        bus.emit('late', done=True)

    threading.Thread(target=emit_later, daemon=True).start()
    event = bus.wait(['late'], since=2, timeout=2.0)
    assert event is not None and event['data'] == {'done': True}


# ---------------------------------------------------------------------------
# shell protocol
# ---------------------------------------------------------------------------


def test_exec_line_encodes_command_and_marker():
    line = build_exec_line("echo 'it''s' \"quoted\"\nprintf second", 'abc123')
    assert line.endswith('\n')
    assert 'base64 -d' in line
    assert '__MPRC_END_%s__ %d' in line
    assert ' abc123 ' in line
    assert '</dev/null' in line


def test_clean_output_line_strips_escapes_and_overwrites():
    assert clean_output_line('\x1b[31mred\x1b[0m') == 'red'
    assert clean_output_line('progress 10%\rprogress 100%') == 'progress 100%'
    assert clean_output_line('abc\b\bX') == 'aX'


@pytestmark_bash
def test_shell_session_runs_stateful_commands_and_reports_exit_codes():
    outputs = []
    session = _local_shell(on_output=lambda _s, lines: outputs.extend(lines))
    try:
        record, lines = _run(session, 'echo "__MPRC_PID__ $$"; echo hello; echo world')
        assert record.exit_code == 0
        assert lines == ['hello', 'world']
        assert session.shell_pid == session.proc.pid

        record, lines = _run(session, 'cd /tmp; export MPRC_VAR=stateful')
        assert record.exit_code == 0 and lines == []

        record, lines = _run(session, 'pwd; echo "$MPRC_VAR"')
        assert lines == ['/tmp', 'stateful']

        record, lines = _run(session, 'echo "line with \'quotes\' and \\"doubles\\""; exit_code_test() { return 7; }; exit_code_test')
        assert record.exit_code == 7
        assert lines == ['line with \'quotes\' and "doubles"']

        record, lines = _run(session, 'printf "no newline at end"')
        assert lines == ['no newline at end']

        record, lines = _run(session, 'echo stderr-line >&2; false')
        assert record.exit_code == 1
        assert lines == ['stderr-line']

        record, lines = _run(session, 'cat')
        assert record.exit_code == 0 and lines == []

        assert 'hello' in outputs
        assert not any('__MPRC_END_' in line for line in outputs)
    finally:
        session.close()
    assert session.closed


@pytestmark_bash
def test_shell_session_rejects_concurrent_commands_and_supports_background_polling():
    session = _local_shell()
    try:
        record = session.run('for i in 1 2 3; do echo tick$i; sleep 0.2; done')
        with pytest.raises(Conflict):
            session.run('echo second')
        assert record.running
        assert session.wait(record, 0.05) is False
        assert record.timed_out
        assert session.wait(record, 10.0)
        lines, total, truncated = session.lines(since=record.first_seq, until=record.last_seq, tail=2)
        assert total == 3 and truncated
        assert [entry['text'] for entry in lines] == ['tick2', 'tick3']
        lines, total, _ = session.lines(since=0, grep='tick[13]')
        assert [entry['text'] for entry in lines] == ['tick1', 'tick3'] and total == 2
    finally:
        session.close()


@pytestmark_bash
@pytest.mark.skipif(not (shutil.which('pkill') or shutil.which('pgrep')), reason='pkill/pgrep required')
def test_shell_session_interrupt_stops_foreground_command():
    session = _local_shell()
    try:
        _run(session, 'echo "__MPRC_PID__ $$"')
        record = session.run('sleep 30 && echo unreachable')
        time.sleep(0.3)
        result = session.interrupt('INT')
        assert result['signal'] == 'INT'
        assert session.wait(record, 10.0)
        assert record.interrupted
        assert record.exit_code != 0
        _, lines = _run(session, 'echo still alive')
        assert lines == ['still alive']
    finally:
        session.close()


@pytestmark_bash
def test_shell_session_exit_marks_session_closed():
    exited = []
    session = _local_shell(on_exit=lambda s: exited.append(s.id))
    record = session.run('exit 3')
    assert session.wait(record, 10.0)
    assert record.session_closed and record.exit_code == 3
    assert session.closed and exited == [1]
    with pytest.raises(Conflict):
        session.run('echo nope')


# ---------------------------------------------------------------------------
# HTTP API
# ---------------------------------------------------------------------------


def test_api_index_status_buttons_and_events(api_server):
    server, adapter, api = api_server
    status, payload = api('GET', '/')
    assert status == 200 and payload['ok']
    assert any(entry['path'].startswith('/shell') for entry in payload['endpoints'])

    status, payload = api('GET', '/status')
    assert status == 200 and payload['workspace'] == 'ws'
    assert payload['server']['port'] == server.address[1]

    status, payload = api('GET', '/buttons')
    assert {b['key'] for b in payload['buttons']} == {'roscore', 'sim', 'busy'}

    status, payload = api('POST', '/buttons/nope/click', {})
    assert status == 404 and not payload['ok']

    status, payload = api('POST', '/buttons/busy/click', {})
    assert status == 200 and payload['accepted'] is False

    status, payload = api('POST', '/buttons/roscore/click', {'wait_for': ['button_state'], 'timeout': 5})
    assert status == 200 and payload['accepted']
    assert payload['wait']['timed_out'] is False
    assert payload['wait']['event']['data'] == {'key': 'roscore', 'state': 'green'}
    assert adapter.clicks == [('roscore', 'click')]

    status, payload = api('POST', '/buttons/roscore/start', {})
    assert payload['accepted'] is False and payload['reason'] == 'already running'

    status, payload = api('GET', '/events?since=0&names=button_state')
    assert [e['name'] for e in payload['events']] == ['button_state']

    status, payload = api('POST', '/wait', {'events': ['never'], 'timeout': 0.1})
    assert status == 200 and payload['timed_out'] is True


def test_api_tabs_dialogs_command_and_quit(api_server):
    server, adapter, api = api_server
    status, payload = api('GET', '/tabs')
    assert payload['tabs'][0]['key'] == 'log'

    status, payload = api('GET', '/tabs/log?tail=1')
    assert payload['lines'] == ['third line'] and payload['truncated'] and payload['total_lines'] == 3

    status, payload = api('GET', '/tabs/log?grep=ERROR')
    assert payload['lines'] == ['second ERROR line']

    status, payload = api('GET', '/tabs/missing')
    assert status == 404

    adapter.dialog = {'title': 'Please Wait', 'buttons': ['OK']}
    status, payload = api('GET', '/dialogs')
    assert payload['dialog']['title'] == 'Please Wait'
    status, payload = api('POST', '/dialogs/dismiss', {'button': 'OK'})
    assert payload['dismissed'] is True and adapter.dialog is None

    status, payload = api('POST', '/command', {'command': 'rostopic list'})
    assert payload['tab'] == 'custom1'

    status, payload = api('POST', '/quit', {})
    assert payload['quitting'] and adapter.quit_called


def test_api_requires_token_when_configured():
    adapter = FakeAdapter()
    server = RemoteControlServer(adapter, host='127.0.0.1', port=0, token='s3cret', invoker=DirectInvoker())
    host, port = server.start()
    try:
        api = _Api(f'http://{host}:{port}')
        status, payload = api('GET', '/status')
        assert status == 401 and not payload['ok']
        status, payload = api('GET', '/status', token='wrong')
        assert status == 401
        status, payload = api('GET', '/status', token='s3cret')
        assert status == 200
        status, payload = api('GET', '/status?token=s3cret')
        assert status == 200
    finally:
        server.stop()


@pytestmark_bash
def test_api_shell_lifecycle_with_stream_flag(api_server):
    server, adapter, api = api_server
    status, payload = api('POST', '/shell', {'name': 'agent', 'stream': True})
    assert status == 200 and payload['ready'], payload
    session_id = payload['session']['id']
    assert payload['session']['shell_pid']

    status, payload = api('POST', f'/shell/{session_id}/exec', {'command': 'echo "$MPRC_TEST"; echo two'})
    assert status == 200 and payload['command']['exit_code'] == 0
    assert payload['output'] == ['1', 'two'] and payload['line_count'] == 2
    assert adapter.mirrored and any('two' in chunk for chunk in adapter.mirrored)

    status, payload = api('POST', f'/shell/{session_id}/exec', {'command': 'seq 1 5', 'stream': False})
    assert 'output' not in payload and payload['line_count'] == 5
    assert 'output_hint' in payload
    command_id = payload['command']['id']

    status, payload = api('GET', f'/shell/{session_id}/output?command={command_id}&tail=2')
    assert [entry['text'] for entry in payload['lines']] == ['4', '5']
    assert payload['matched_lines'] == 5 and payload['truncated']

    status, payload = api('POST', f'/shell/{session_id}/exec', {'command': 'seq 1 5', 'tail': 1, 'grep': '[24]'})
    assert payload['output'] == ['4'] and payload['line_count'] == 2

    status, payload = api('POST', f'/shell/{session_id}/settings', {'stream': False})
    assert payload['session']['stream_default'] is False
    status, payload = api('POST', f'/shell/{session_id}/exec', {'command': 'echo hidden'})
    assert 'output' not in payload

    status, payload = api('POST', f'/shell/{session_id}/exec', {'command': 'sleep 0.3; echo late', 'wait': False})
    assert payload['waited'] is False and payload['command']['running']
    status, conflict = api('POST', f'/shell/{session_id}/exec', {'command': 'echo blocked'})
    assert status == 409 and 'busy' in conflict['error']

    events = api.stream(f'/shell/{session_id}/output?follow=1&timeout=10&command={payload["command"]["id"]}')
    texts = [item['text'] for item in events if 'text' in item]
    assert texts == ['late']
    assert events[-1]['done'] is True

    status, payload = api('POST', f'/shell/{session_id}/exec', {'command': 'sleep 5', 'timeout': 0.2})
    assert payload['timed_out'] is True and payload['command']['running']
    status, payload = api('POST', f'/shell/{session_id}/interrupt', {})
    assert status == 200 and payload['signal'] == 'INT'

    deadline = time.time() + 10
    while time.time() < deadline:
        status, payload = api('GET', f'/shell/{session_id}')
        if not payload['session']['busy']:
            break
        time.sleep(0.05)
    assert not payload['session']['busy']

    status, payload = api('DELETE', f'/shell/{session_id}')
    assert status == 200 and payload['session']['closed']
    assert session_id in adapter.exited
    status, payload = api('GET', '/shell')
    assert payload['shells'] == []
    names = [e['name'] for e in server.events.since(0)]
    assert 'shell_opened' in names and 'shell_closed' in names


def test_api_events_follow_streams_until_timeout(api_server):
    server, adapter, api = api_server
    threading.Thread(target=lambda: (time.sleep(0.1), server.emit('custom', n=1)), daemon=True).start()
    items = api.stream('/events?follow=1&timeout=0.6&names=custom')
    assert items[0]['name'] == 'custom' and items[0]['data'] == {'n': 1}
    assert items[-1]['name'] == 'stream_timeout'


# ---------------------------------------------------------------------------
# settings and CLI parsing
# ---------------------------------------------------------------------------


def test_remote_control_settings_merges_sources():
    merged = remote_control_settings({'enabled': 'yes', 'port': '9000'}, {'token': 'abc', 'host': None})
    assert merged['enabled'] is True and merged['port'] == 9000
    assert merged['token'] == 'abc' and merged['host'] == '0.0.0.0'
    assert remote_control_settings(None)['enabled'] is False


def test_cli_remote_control_overrides(monkeypatch):
    parser = _build_parser()
    for name in ('MOBIPICK_GUI_REMOTE_CONTROL', 'MOBIPICK_GUI_REMOTE_HOST', 'MOBIPICK_GUI_REMOTE_PORT', 'MOBIPICK_GUI_REMOTE_TOKEN'):
        monkeypatch.delenv(name, raising=False)
    args, _ = parser.parse_known_args([])
    assert remote_control_overrides(args) == {}

    args, _ = parser.parse_known_args(['--remote-control', '--remote-port', '9999'])
    assert remote_control_overrides(args) == {'enabled': True, 'port': 9999}

    args, _ = parser.parse_known_args(['--remote-token', 'x'])
    assert remote_control_overrides(args) == {'token': 'x', 'enabled': True}

    monkeypatch.setenv('MOBIPICK_GUI_REMOTE_CONTROL', '1')
    monkeypatch.setenv('MOBIPICK_GUI_REMOTE_HOST', '127.0.0.1')
    args, _ = parser.parse_known_args(['--no-remote-control'])
    assert remote_control_overrides(args) == {'enabled': False, 'host': '127.0.0.1'}


# ---------------------------------------------------------------------------
# MainWindow integration
# ---------------------------------------------------------------------------


def _make_window(tmp_path, monkeypatch, **remote):
    from PyQt5.QtWidgets import QApplication

    from mobipick_gui.config import CONFIG
    from mobipick_gui.main_window import MainWindow

    monkeypatch.setenv('MOBIPICK_WORKSPACE_CONFIG', str(tmp_path / 'workspaces.yaml'))
    monkeypatch.setattr(
        MainWindow,
        '_discover_filtered_image_records',
        lambda self: ([{'ref': CONFIG['images']['default']}], None),
    )
    monkeypatch.setattr(MainWindow, 'update_sim_status_from_poll', lambda self, force=False: None)
    app = QApplication.instance() or QApplication([])
    window = MainWindow(
        verbosity=1,
        remote_control={'enabled': True, 'host': '127.0.0.1', 'port': 0, **remote},
    )
    window.poll_timer.stop()
    window._sigint_timer.stop()
    return app, window


def test_main_window_starts_remote_control_and_serves_buttons(tmp_path, monkeypatch):
    app, window = _make_window(tmp_path, monkeypatch)
    try:
        server = window.remote_control
        assert server is not None and server.running
        host, port = server.address
        assert host == '127.0.0.1' and port > 0
        assert window._remote_control_action.isChecked()

        api = _Api(f'http://{host}:{port}')
        result = {}

        def query():
            result['status'] = api('GET', '/status')
            result['buttons'] = api('GET', '/buttons')
            result['tab'] = api('GET', '/tabs/log?tail=5')
            result['click'] = api('POST', '/buttons/roscore/click', {})

        toggled = []
        monkeypatch.setattr(window, 'toggle_roscore', lambda: toggled.append(True))
        worker = threading.Thread(target=query, daemon=True)
        worker.start()
        deadline = time.time() + 10
        while worker.is_alive() and time.time() < deadline:
            app.processEvents()
            time.sleep(0.01)
        assert not worker.is_alive(), 'HTTP requests did not complete'

        status, payload = result['status']
        assert status == 200 and payload['ok']
        assert payload['roscore_running'] is False
        assert 'server' in payload

        status, payload = result['buttons']
        keys = {button['key'] for button in payload['buttons']}
        assert {'roscore', 'terminal', 'auto_launch', 'sim'} <= keys
        roscore = next(b for b in payload['buttons'] if b['key'] == 'roscore')
        assert roscore['state'] == 'red' and roscore['text'] == 'Start Roscore'
        assert roscore['runs_on'] == 'container' and roscore['tab'] == 'roscore'
        sim = next(b for b in payload['buttons'] if b['key'] == 'sim')
        assert 'simulator' in sim['tooltip'].lower()

        status, payload = result['tab']
        assert status == 200 and payload['tab'] == 'log'

        status, payload = result['click']
        assert status == 200 and payload['accepted'] is True
        assert toggled == [True]

        before = server.events.last_seq
        window.set_roscore_visual('green', 'Stop Roscore', True)
        event = server.events.since(before, names=['button_state'])[-1]
        assert event['data']['key'] == 'roscore' and event['data']['state'] == 'green'

        window._stop_remote_control()
        assert window.remote_control is None
        assert not window._remote_control_action.isChecked()
    finally:
        if window.remote_control is not None:
            window._stop_remote_control()
        window.deleteLater()
        app.processEvents()


def test_main_window_layout_apply_and_ready_emit_remote_events(tmp_path, monkeypatch):
    app, window = _make_window(tmp_path, monkeypatch)
    try:
        server = window.remote_control
        window._auto_launch_running = True
        window._auto_launch_active_keys = ['roscore', 'sim']
        window._auto_launch_ready_keys = set()
        before = server.events.last_seq
        window._mark_auto_launch_ready('roscore')
        window._mark_auto_launch_ready('sim')
        names = [e['name'] for e in server.events.since(before)]
        assert names == ['auto_launch_ready', 'auto_launch_ready', 'auto_launch_complete']

        before = server.events.last_seq
        window._window_layout_manager._on_applied(2)
        event = server.events.since(before)[-1]
        assert event['name'] == 'window_layout_applied' and event['data'] == {'windows': 2}
    finally:
        window._stop_remote_control()
        window.deleteLater()
        app.processEvents()


# ---------------------------------------------------------------------------
# bundled skill
# ---------------------------------------------------------------------------


def test_bundled_skill_matches_repo_skill_and_installs(tmp_path):
    from pathlib import Path

    from mobipick_gui.remote_client import bundled_skill_path, install_skill, main as client_main

    bundled = bundled_skill_path()
    assert bundled.is_file()
    assert bundled.read_text().startswith('---\nname: mobipick-gui-remote\n')
    repo_copy = Path(__file__).resolve().parents[2] / '.claude' / 'skills' / 'mobipick-gui-remote' / 'SKILL.md'
    if repo_copy.is_file():
        assert repo_copy.read_text() == bundled.read_text(), 'keep .claude/skills and resources/skills in sync'

    installed = install_skill(tmp_path / 'skills')
    assert installed == tmp_path / 'skills' / 'mobipick-gui-remote' / 'SKILL.md'
    assert installed.read_text() == bundled.read_text()
    assert client_main(['skill', '--install', str(tmp_path / 'again')]) == 0
    assert (tmp_path / 'again' / 'mobipick-gui-remote' / 'SKILL.md').is_file()
