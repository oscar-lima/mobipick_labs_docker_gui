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
        self.stopped_tabs = []
        self.cleaned = []
        self.reloads = []
        self.arg_values = {'anygrasp_mode': 'mockup'}
        self.ready = {}

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
                'ready': bool(self.ready.get(key, state == 'green')),
                'args': dict(self.arg_values) if key == 'sim' else {},
            }
            for key, state in self.states.items()
        ]

    def args(self):
        return [
            {
                'slot': 3,
                'name': name,
                'value': value,
                'options': ['real', 'mockup'],
                'buttons': ['sim'],
            }
            for name, value in self.arg_values.items()
        ]

    def set_args(self, values):
        from mobipick_gui.remote_control import RemoteControlError

        for name, value in values.items():
            if name not in self.arg_values:
                raise RemoteControlError(f'unknown argument {name!r}')
            if value not in {'real', 'mockup'}:
                raise RemoteControlError(f'{name!r} accepts [real, mockup], not {value!r}')
            self.arg_values[name] = value
        return self.args()

    def press_button(self, key, action):
        from mobipick_gui.remote_control import NotFound

        if key not in self.states:
            raise NotFound(f'unknown button {key}')
        button = next(b for b in self.buttons() if b['key'] == key)
        if self.states[key] == 'yellow':
            return {'accepted': False, 'reason': 'busy', 'button': button}
        if action == 'start' and self.states[key] == 'green':
            return {'accepted': False, 'reason': 'already running', 'button': button}
        self.clicks.append((key, action))
        was_running = self.states[key] == 'green'
        self.states[key] = 'red' if was_running else 'green'
        if self.server is not None:
            self.server.emit('button_state', key=key, state=self.states[key])
        button = next(b for b in self.buttons() if b['key'] == key)
        return {'accepted': True, 'action': action, 'was_running': was_running, 'button': button}

    def reload_configuration(self):
        self.reloads.append(True)
        return {'reloaded': True, 'buttons': ['sim', 'rviz'], 'profile': 'profile.yaml'}

    def recording(self):
        return dict(getattr(self, 'recording_state', {'active': False, 'paused': False, 'armed': False}))

    def recording_action(self, action):
        state = getattr(self, 'recording_state', {'active': False, 'paused': False, 'armed': False})
        self.recording_state = state
        if action == 'start':
            if state['active']:
                return {'accepted': False, 'reason': 'recording already active', **state}
            state.update(active=True, paused=False, segments=1, video_path='/tmp/rec/rec.mp4')
        elif not state['active']:
            return {'accepted': False, 'reason': 'no recording active', **state}
        elif action == 'pause':
            if state['paused']:
                return {'accepted': False, 'reason': 'already paused', **state}
            state['paused'] = True
        elif action == 'resume':
            if not state['paused']:
                return {'accepted': False, 'reason': 'not paused', **state}
            state['paused'] = False
            state['segments'] = state.get('segments', 1) + 1
        elif action == 'stop':
            state.update(active=False, paused=False)
        return {'accepted': True, 'reason': None, **state}

    def stop_tab(self, key):
        self.stopped_tabs.append(key)
        return {'tab': key, 'stopped': True, 'kind': 'process'}

    def stop_owned(self, name, entries):
        self.cleaned.append((name, entries))
        notes = []
        for entry in entries:
            if entry['kind'] == 'button' and self.states.get(entry['key']) == 'green':
                notes.append(f"stopped button {entry['key']}")
                self.press_button(entry['key'], 'stop')
            elif entry['kind'] == 'tab':
                notes.append(f"stopped tab {entry['key']}")
                self.stop_tab(entry['key'])
        return notes

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


@pytestmark_bash
def test_shell_session_close_does_not_block_the_caller():
    # A shell still running a foreground command ignores the "exit" and has
    # to be terminated after the timeout; the caller (the GUI thread when a
    # client lapsed) must not wait for that.
    session = _local_shell()
    session.run('sleep 30')
    started = time.monotonic()
    thread = session.close(timeout=2.0, wait=False)
    assert time.monotonic() - started < 1.0
    thread.join(15)
    assert not thread.is_alive() and session.closed
    assert session.close(wait=False) is thread  # a second close reuses the teardown


def test_close_all_sessions_runs_teardowns_in_parallel(monkeypatch, api_server):
    server, _adapter, _api = api_server
    teardowns = []

    def slow_teardown(self, timeout):
        teardowns.append(self.id)
        time.sleep(0.5)
        self._closed = True

    monkeypatch.setattr(RemoteShellSession, '_teardown', slow_teardown)
    for session_id in (1, 2, 3):
        with server._sessions_lock:
            server._sessions[session_id] = _local_shell(session_id=session_id)
    started = time.monotonic()
    server.close_all_sessions(wait=False)
    assert time.monotonic() - started < 0.3 and server.sessions() == []
    server.close_all_sessions(wait=True)  # nothing left: returns at once
    deadline = time.monotonic() + 5
    while sorted(teardowns) != [1, 2, 3] and time.monotonic() < deadline:
        time.sleep(0.01)
    assert sorted(teardowns) == [1, 2, 3]


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


def test_api_args_endpoint_and_button_args(api_server):
    server, adapter, api = api_server
    status, payload = api('GET', '/args')
    assert status == 200 and payload['args'][0]['name'] == 'anygrasp_mode'
    assert payload['args'][0]['value'] == 'mockup' and payload['args'][0]['buttons'] == ['sim']

    status, payload = api('POST', '/args', {'anygrasp_mode': 'real'})
    assert status == 200 and payload['args'][0]['value'] == 'real'
    status, payload = api('POST', '/args', {'anygrasp_mode': 'bogus'})
    assert status == 400 and 'accepts' in payload['error']
    status, payload = api('POST', '/args', {'nope': 'x'})
    assert status == 400 and 'unknown argument' in payload['error']
    status, payload = api('POST', '/args', {})
    assert status == 400
    assert adapter.arg_values == {'anygrasp_mode': 'real'}

    # args in the press body are applied before the click, and a bad value blocks it
    status, payload = api('POST', '/buttons/sim/start', {'args': {'anygrasp_mode': 'bogus'}})
    assert status == 400 and adapter.clicks == []
    status, payload = api('POST', '/buttons/sim/start', {'args': 'real'})
    assert status == 400 and adapter.clicks == []
    status, payload = api('POST', '/buttons/sim/start', {'args': {'anygrasp_mode': 'mockup'}})
    assert status == 200 and payload['accepted'] is True
    assert adapter.clicks == [('sim', 'start')]
    assert payload['button']['args'] == {'anygrasp_mode': 'mockup'}
    status, payload = api('GET', '/buttons')
    sim = next(b for b in payload['buttons'] if b['key'] == 'sim')
    assert sim['args'] == {'anygrasp_mode': 'mockup'} and sim['ready'] is True


def test_api_button_wait_is_keyed_and_ready_short_circuits(api_server):
    server, adapter, api = api_server

    # button_state from another button must not satisfy a keyed wait
    def emit_other_then_mine():
        time.sleep(0.05)
        server.emit('button_state', key='roscore', state='green')
        server.emit('button_ready', key='roscore', startup_seconds=3.0)
        time.sleep(0.05)
        server.emit('button_ready', key='sim', startup_seconds=30.0)

    threading.Thread(target=emit_other_then_mine, daemon=True).start()
    status, payload = api('POST', '/buttons/sim/start', {'wait_for': ['button_ready'], 'timeout': 5})
    assert status == 200 and payload['accepted'] is True
    assert payload['wait']['timed_out'] is False
    assert payload['wait']['event']['name'] == 'button_ready'
    assert payload['wait']['event']['data']['key'] == 'sim'

    # already running and ready: start is refused but the wait resolves at once
    status, payload = api('POST', '/buttons/sim/start', {'wait_for': ['button_ready'], 'timeout': 5})
    assert payload['accepted'] is False and payload['reason'] == 'already running'
    assert payload['wait'] == {'timed_out': False, 'since': payload['wait']['since'], 'event': None, 'already_ready': True}

    # running but not yet past the estimate: no shortcut, and the request does not wait
    adapter.ready['sim'] = False
    status, payload = api('POST', '/buttons/sim/start', {'wait_for': ['button_ready'], 'timeout': 0.1})
    assert payload['accepted'] is False and 'wait' not in payload

    # /wait accepts a key too; events without a key still match
    seq = server.events.last_seq

    def emit_later():
        time.sleep(0.05)
        server.emit('button_ready', key='roscore')
        server.emit('window_layout_applied', windows=1)

    threading.Thread(target=emit_later, daemon=True).start()
    status, payload = api('POST', '/wait', {'events': ['button_ready', 'window_layout_applied'], 'since': seq, 'key': 'sim', 'timeout': 5})
    assert status == 200 and payload['event']['name'] == 'window_layout_applied'


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
    assert remote_control_overrides(args) == {'enabled': False}
    assert remote_control_settings(
        {'enabled': True},
        remote_control_overrides(args),
    )['enabled'] is False

    args, _ = parser.parse_known_args(['--remote-control', '--remote-port', '9999'])
    assert remote_control_overrides(args) == {
        'enabled': True,
        '_enabled_source': '--remote-control',
        'port': 9999,
    }

    args, _ = parser.parse_known_args(['--remote-token', 'x'])
    assert remote_control_overrides(args) == {
        'token': 'x',
        'enabled': True,
        '_enabled_source': 'remote-control CLI option',
    }

    monkeypatch.setenv('MOBIPICK_GUI_REMOTE_CONTROL', '1')
    monkeypatch.setenv('MOBIPICK_GUI_REMOTE_HOST', '127.0.0.1')
    args, _ = parser.parse_known_args(['--no-remote-control'])
    assert remote_control_overrides(args) == {
        'enabled': False,
        '_enabled_source': '--no-remote-control',
        'host': '127.0.0.1',
    }


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


def test_main_window_button_readiness_uses_launch_plan_estimates(tmp_path, monkeypatch):
    app, window = _make_window(tmp_path, monkeypatch)
    try:
        server = window.remote_control
        window._launch_plan = dict(
            window._launch_plan,
            processes=[{'button': 'roscore', 'duration_seconds': 0.2, 'depends_on': ''}],
        )
        assert window.button_startup_seconds('roscore') == 0.2
        assert window.button_startup_seconds('sim') is None
        adapter = window.remote_control.adapter
        roscore = next(b for b in adapter.buttons() if b['key'] == 'roscore')
        assert roscore['ready'] is False and roscore['startup_seconds'] == 0.2
        assert roscore['ready_in_s'] is None

        before = server.events.last_seq
        window.set_roscore_visual('yellow', 'Starting Roscore...', False)
        window.set_roscore_visual('green', 'Stop Roscore', True)
        roscore = next(b for b in adapter.buttons() if b['key'] == 'roscore')
        assert roscore['running'] and roscore['ready'] is False
        assert 0.0 < roscore['ready_in_s'] <= 0.2
        assert [e['name'] for e in server.events.since(before)] == ['button_state', 'button_state']

        deadline = time.time() + 3
        while time.time() < deadline and not server.events.since(before, names=['button_ready']):
            app.processEvents()
            time.sleep(0.01)
        ready = server.events.since(before, names=['button_ready'])
        assert ready and ready[-1]['data'] == {'key': 'roscore', 'startup_seconds': 0.2, 'estimated': True}
        roscore = next(b for b in adapter.buttons() if b['key'] == 'roscore')
        assert roscore['ready'] is True and roscore['ready_in_s'] == 0.0

        # no estimate: ready as soon as the button turns green
        before = server.events.last_seq
        window._set_toggle_state('sim', None, 'green', 'Stop Sim', True)
        app.processEvents()
        names = [e['name'] for e in server.events.since(before)]
        assert names == ['button_state', 'button_ready']
        assert server.events.since(before)[-1]['data']['estimated'] is False

        # stopping forgets the readiness bookkeeping
        window.set_roscore_visual('red', 'Start Roscore', True)
        roscore = next(b for b in adapter.buttons() if b['key'] == 'roscore')
        assert roscore['ready'] is False and roscore['ready_at'] is None
        assert 'roscore' not in window._button_ready_timers
    finally:
        window._stop_remote_control()
        window.deleteLater()
        app.processEvents()


def test_main_window_generic_args_over_remote_api(tmp_path, monkeypatch):
    from mobipick_gui import main_window as mw

    original = mw.load_button_layout

    def layout_with_arg(*a, **k):
        entries = list(original(*a, **k))
        entries.append(
            {
                'key': 'anygrasp',
                'label': 'Anygrasp',
                'kind': 'command',
                'command': 'launch_anygrasp.sh',
                'host': True,
                'arg_3_name': 'anygrasp_mode',
                'arg_3_options': ['real', 'mockup'],
                'arg_3_applies': True,
            }
        )
        return entries

    monkeypatch.setattr(mw, 'load_button_layout', layout_with_arg)
    app, window = _make_window(tmp_path, monkeypatch)
    try:
        adapter = window.remote_control.adapter
        args = {entry['name']: entry for entry in adapter.args()}
        assert args['anygrasp_mode']['options'] == ['real', 'mockup']
        assert args['anygrasp_mode']['value'] == 'real'
        assert args['anygrasp_mode']['buttons'] == ['anygrasp']
        assert 'world' in args and window._current_world() in args['world']['options']

        anygrasp = next(b for b in adapter.buttons() if b['key'] == 'anygrasp')
        assert anygrasp['args'] == {'anygrasp_mode': 'real'}
        assert anygrasp['full_command'] == "launch_anygrasp.sh anygrasp_mode:='real'"
        sim = next(b for b in adapter.buttons() if b['key'] == 'sim')
        assert sim['args'] == {}

        adapter.set_args({'anygrasp_mode': 'mockup'})
        assert window._generic_arg_inputs[3].currentText() == 'mockup'
        anygrasp = next(b for b in adapter.buttons() if b['key'] == 'anygrasp')
        assert anygrasp['full_command'] == "launch_anygrasp.sh anygrasp_mode:='mockup'"

        from mobipick_gui.remote_control import RemoteControlError

        with pytest.raises(RemoteControlError):
            adapter.set_args({'anygrasp_mode': 'gpu'})
        with pytest.raises(RemoteControlError):
            adapter.set_args({'unknown': 'x'})

        world = args['world']['options'][-1]
        adapter.set_args({'world': world})
        assert window._current_world() == world
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


def test_remote_control_tracks_active_requests():
    class _BlockingAdapter(FakeAdapter):
        release = threading.Event()
        entered = threading.Event()

        def status(self):
            self.entered.set()
            self.release.wait(5)
            return super().status()

    adapter = _BlockingAdapter()
    server = RemoteControlServer(adapter, host='127.0.0.1', port=0)
    host, port = server.start()
    try:
        assert server.active_requests == 0
        api = _Api(f'http://{host}:{port}')
        worker = threading.Thread(target=lambda: api('GET', '/status'), daemon=True)
        worker.start()
        assert adapter.entered.wait(5)
        assert server.active_requests == 1
        adapter.release.set()
        worker.join(5)
        assert not worker.is_alive()
        deadline = time.time() + 2
        while server.active_requests and time.time() < deadline:
            time.sleep(0.01)
        assert server.active_requests == 0
    finally:
        adapter.release.set()
        server.stop()


def test_main_window_glows_icon_while_remote_control_is_active(tmp_path, monkeypatch):
    from mobipick_gui import main_window as mw

    app, window = _make_window(tmp_path, monkeypatch)
    try:
        server = window.remote_control
        assert server is not None
        timer = window._remote_icon_timer
        assert timer is not None and timer.isActive()
        idle_level = window._remote_icon_level
        assert idle_level == (
            round(mw.REMOTE_ICON_GLOW_IDLE * mw.REMOTE_ICON_GLOW_LEVELS),
            mw.REMOTE_ICON_GLOW_COLOR.rgb(),
        )
        base = window._remote_icon_base
        assert base is not None and not base.isNull()
        assert window.windowIcon().cacheKey() != base.cacheKey()

        # Idle: repeated ticks keep the static glow.
        window._update_remote_icon_glow()
        assert window._remote_icon_level == idle_level

        # Active request: the glow pulses through several levels.
        monkeypatch.setattr(type(server), 'active_requests', property(lambda self: 1))
        seen = set()
        deadline = time.time() + mw.REMOTE_ICON_GLOW_PULSE_S
        while time.time() < deadline:
            window._update_remote_icon_glow()
            seen.add(window._remote_icon_level[0])
            time.sleep(0.02)
        assert len(seen) > 3
        assert min(seen) < idle_level[0] < max(seen)

        # Back to idle: static glow again.
        monkeypatch.setattr(type(server), 'active_requests', property(lambda self: 0))
        window._update_remote_icon_glow()
        assert window._remote_icon_level == idle_level

        # A client declaring presence lights the icon fully until it withdraws.
        # Presence switches the halo to the green in-use colour, which is far
        # easier to spot on the dock than a brightness change alone.
        token = server.declare_presence('claude')['token']
        window._update_remote_icon_glow()
        assert window._remote_icon_level == (
            mw.REMOTE_ICON_GLOW_LEVELS,
            mw.REMOTE_ICON_GLOW_COLOR_IN_USE.rgb(),
        )
        assert mw.REMOTE_ICON_GLOW_COLOR_IN_USE.rgb() != mw.REMOTE_ICON_GLOW_COLOR.rgb()
        assert window._remote_client_names == {'claude'}
        server.withdraw_presence('claude', token=token)
        window._update_remote_icon_glow()
        assert window._remote_icon_level == idle_level
        assert window._remote_client_names == set()

        window._stop_remote_control()
        assert not timer.isActive()
        assert window.windowIcon().cacheKey() == base.cacheKey()
    finally:
        if window.remote_control is not None:
            window._stop_remote_control()
        window.close()


def test_remote_glow_is_cleared_before_exit_cleanup(monkeypatch):
    from types import SimpleNamespace

    from PyQt5.QtWidgets import QApplication

    from mobipick_gui import main_window as mw

    app = QApplication.instance() or QApplication([])  # noqa: F841 - keep alive
    events = []
    harness = SimpleNamespace(
        _exit_in_progress=False,
        _exit_dialog=None,
        _stop_remote_icon_glow=lambda: events.append('clear_glow'),
        _save_window_state=lambda: events.append('save'),
        _console_log=lambda *_args: events.append('log'),
        hide=lambda: events.append('hide'),
        keep_window_above=lambda _dialog: events.append('dialog'),
        _perform_exit_cleanup=lambda: events.append('cleanup'),
    )
    monkeypatch.setattr(
        mw.QTimer,
        'singleShot',
        lambda _delay, callback: events.append(('scheduled', callback)),
    )

    mw.MainWindow._begin_exit_sequence(harness)

    assert events[:4] == ['clear_glow', 'save', 'log', 'hide']
    assert events[-1][0] == 'scheduled'
    harness._exit_dialog.close()


def test_stopping_absent_remote_server_still_clears_glow():
    from types import SimpleNamespace

    from mobipick_gui.main_window import MainWindow

    cleared = []
    harness = SimpleNamespace(
        remote_control=None,
        _stop_remote_icon_glow=lambda: cleared.append(True),
    )

    MainWindow._stop_remote_control(harness)

    assert cleared == [True]


def test_remote_glow_icon_adds_halo_margin():
    from PyQt5.QtCore import Qt
    from PyQt5.QtGui import QColor, QIcon, QPixmap
    from PyQt5.QtWidgets import QApplication

    from mobipick_gui.main_window import REMOTE_ICON_GLOW_MARGIN, remote_glow_icon

    app = QApplication.instance() or QApplication([])  # noqa: F841 - keep alive
    source = QPixmap(64, 64)
    source.fill(Qt.white)
    base = QIcon(source)
    glow = remote_glow_icon(base, 1.0, size=64)
    pixmap = glow.pixmap(256, 256)
    expected = 64 + 2 * int(round(64 * REMOTE_ICON_GLOW_MARGIN))
    assert pixmap.width() == expected and pixmap.height() == expected
    image = pixmap.toImage()
    corner = image.pixelColor(1, 1)
    assert corner.alpha() > 0 and corner.blue() > corner.red()
    assert image.pixelColor(pixmap.width() // 2, pixmap.height() // 2) == QColor(Qt.white)
    assert remote_glow_icon(QIcon(), 1.0).isNull()


def test_presence_endpoints_track_clients(monkeypatch):
    adapter = FakeAdapter()
    server = RemoteControlServer(adapter, host='127.0.0.1', port=0)
    host, port = server.start()
    try:
        api = _Api(f'http://{host}:{port}')
        status, payload = api('GET', '/status')
        assert status == 200 and payload['in_use'] is False and payload['clients'] == []

        before = server.events.last_seq
        status, payload = api('POST', '/presence', {'name': 'claude', 'ttl_s': 5, 'note': 'tables demo'})
        assert status == 200 and payload['client']['name'] == 'claude'
        token = payload['client']['token']
        assert 0 < payload['client']['expires_in_s'] <= 5
        assert [c['name'] for c in payload['clients']] == ['claude']
        assert server.in_use
        events = server.events.since(before, names=['client_connected'])
        assert len(events) == 1 and events[0]['data']['note'] == 'tables demo'

        # Refreshing does not emit a second connect event.
        assert 'token' not in api('POST', '/presence', {'name': 'claude'})[1]['client']
        assert len(server.events.since(before, names=['client_connected'])) == 1

        status, payload = api('GET', '/status')
        assert payload['in_use'] is True and payload['clients'][0]['name'] == 'claude'

        status, payload = api('POST', '/presence', {})
        assert status == 400

        status, payload = api('DELETE', '/presence', {'name': 'claude', 'token': token})
        assert status == 200 and payload['removed'] is True and payload['clients'] == []
        assert not server.in_use
        assert server.events.since(before, names=['client_disconnected'])[-1]['data']['name'] == 'claude'
        status, payload = api('DELETE', '/presence', {'name': 'claude', 'token': token})
        assert payload['removed'] is False
    finally:
        server.stop()


def test_presence_expires_after_ttl(monkeypatch):
    server = RemoteControlServer(FakeAdapter(), host='127.0.0.1', port=0)
    now = [1000.0]
    monkeypatch.setattr('mobipick_gui.remote_control.time.time', lambda: now[0])
    server.declare_presence('claude', ttl_s=10)
    assert server.in_use
    now[0] += 9
    assert server.in_use
    now[0] += 2
    before = server.events.last_seq
    assert not server.in_use
    event = server.events.since(before, names=['client_disconnected'])[-1]
    assert event['data']['name'] == 'claude' and event['data']['expired'] is True


def test_stop_tab_endpoint_and_ownership_cleanup():
    adapter = FakeAdapter()
    server = RemoteControlServer(adapter, host='127.0.0.1', port=0)
    adapter.server = server
    host, port = server.start()
    try:
        api = _Api(f'http://{host}:{port}')
        token = api('POST', '/presence', {'name': 'claude'})[1]['client']['token']
        assert api('POST', '/buttons/roscore/start', {})[1]['accepted']
        assert api('POST', '/command', {'command': 'roslaunch x y.launch'})[1]['tab'] == 'custom1'
        owned = {(e['kind'], e['key']) for e in server.owned_by('claude')}
        assert owned == {('button', 'roscore'), ('tab', 'custom1')}

        # Stopping through the API releases ownership.
        status, payload = api('POST', '/tabs/custom1/stop', {})
        assert status == 200 and payload['stopped'] and adapter.stopped_tabs == ['custom1']
        assert {(e['kind'], e['key']) for e in server.owned_by('claude')} == {('button', 'roscore')}
        # A click on a running button is a stop and releases it too.
        api('POST', '/buttons/roscore/click', {})
        assert server.owned_by('claude') == []
        assert adapter.states['roscore'] == 'red'

        # Bye with leftovers: the GUI side collects them for cleanup.
        api('POST', '/buttons/sim/start', {})
        api('POST', '/command', {'command': 'rostopic echo /x'})
        assert len(server.owned_by('claude')) == 2
        api('DELETE', '/presence', {'name': 'claude', 'token': token})
        assert server.leave_reason('claude') == 'done'
        leftovers = server.take_owned('claude')
        assert {(e['kind'], e['key']) for e in leftovers} == {('button', 'sim'), ('tab', 'custom1')}
        assert adapter.stop_owned('claude', leftovers) == ['stopped button sim', 'stopped tab custom1']
        assert server.take_owned('claude') == []

        # Bye with keep: nothing is collected.
        token = api('POST', '/presence', {'name': 'claude'})[1]['client']['token']
        api('POST', '/buttons/sim/start', {})
        api('DELETE', '/presence', {'name': 'claude', 'keep': True, 'token': token})
        assert server.leave_reason('claude') == 'done (processes kept)'
        assert server.take_owned('claude') == []

        # Unknown tab
        assert api('POST', '/tabs/nope/stop', {})[0] == 404 or adapter.stopped_tabs[-1] == 'nope'
    finally:
        server.stop()


def test_requests_refresh_presence(monkeypatch):
    """Any request is activity: a client working through the API never needs a heartbeat."""
    server = RemoteControlServer(FakeAdapter(), host='127.0.0.1', port=0)
    now = [1000.0]
    monkeypatch.setattr('mobipick_gui.remote_control.time.time', lambda: now[0])
    server.declare_presence('claude', ttl_s=10)
    now[0] += 8
    status, _ = server.handle('GET', '/buttons', {}, {})
    assert status == 200
    now[0] += 8                      # 16 s after declaring, 8 s after the last request
    assert [c['name'] for c in server.clients()] == ['claude']
    assert server.clients()[0]['idle_s'] == 8.0
    # reads of the endpoint list and of the presence itself are not activity
    now[0] += 4
    server.handle('GET', '/', {}, {})
    server.handle('GET', '/presence', {}, {})
    assert not server.in_use


def test_named_client_refreshes_only_itself(monkeypatch):
    server = RemoteControlServer(FakeAdapter(), host='127.0.0.1', port=0)
    now = [1000.0]
    monkeypatch.setattr('mobipick_gui.remote_control.time.time', lambda: now[0])
    server.declare_presence('codex', ttl_s=10)
    server.declare_presence('claude', ttl_s=10)
    now[0] += 8
    # the header names the client; a body / query field does the same
    server.handle('GET', '/buttons', {}, {}, client='claude')
    server.handle('GET', '/buttons', {'client': 'claude'}, {})
    # without a name the request refreshes the client present the longest (codex)
    server.handle('GET', '/tabs', {}, {})
    now[0] += 4
    assert {c['name'] for c in server.clients()} == {'codex', 'claude'}
    now[0] += 5                      # codex: 9 s idle, claude: 9 s idle -> both still there
    assert {c['name'] for c in server.clients()} == {'codex', 'claude'}
    server.handle('GET', '/tabs', {}, {}, client='claude')
    now[0] += 3                      # codex 12 s idle -> gone; claude refreshed
    assert [c['name'] for c in server.clients()] == ['claude']
    # unknown names never create a client
    server.handle('GET', '/tabs', {}, {}, client='nobody')
    assert [c['name'] for c in server.clients()] == ['claude']


def test_running_shell_command_keeps_client_present(api_server, monkeypatch):
    server, adapter, api = api_server
    api('POST', '/presence', {'name': 'claude', 'ttl_s': 1})
    session_id = api('POST', '/shell', {'name': 'claude'})[1]['session']['id']
    api('POST', f'/shell/{session_id}/exec', {'command': 'sleep 2.5; echo done', 'wait': False})
    time.sleep(1.6)                  # well past the TTL, no request in between
    assert [c['name'] for c in server.clients()] == ['claude']
    assert server.clients()[0]['expires_in_s'] > 0
    deadline = time.time() + 5
    while time.time() < deadline and server.session(session_id).busy:
        time.sleep(0.1)
    assert not server.session(session_id).busy
    time.sleep(1.3)                  # idle after the command: the TTL applies again
    assert server.clients() == []
    assert server.leave_reason('claude') == 'presence expired'


def test_open_stream_keeps_client_present(api_server, monkeypatch):
    server, adapter, api = api_server
    api('POST', '/presence', {'name': 'claude', 'ttl_s': 1})
    # a follow stream that waits 2.5 s for events (none come) is activity throughout
    lines = api.stream('/events?follow=1&timeout=2.5&names=never', timeout=10)
    assert lines[-1]['name'] == 'stream_timeout'
    assert [c['name'] for c in server.clients()] == ['claude']


def test_presence_ttl_is_capped():
    server = RemoteControlServer(FakeAdapter(), host='127.0.0.1', port=0)
    entry = server.declare_presence('claude', ttl_s=99999)
    assert entry['ttl_s'] == 1800.0
    entry = server.declare_presence('claude')
    assert entry['ttl_s'] == 600.0


def test_main_window_cleans_up_after_expired_client(tmp_path, monkeypatch):
    app, window = _make_window(tmp_path, monkeypatch)
    try:
        server = window.remote_control
        now = [1000.0]
        monkeypatch.setattr('mobipick_gui.remote_control.time.time', lambda: now[0])
        stopped = []
        monkeypatch.setattr(type(server.adapter), 'stop_owned', lambda self, name, entries: stopped.append((name, entries)) or ['stopped button roscore'])
        server.declare_presence('claude', ttl_s=10)
        server._record_owned('button', 'roscore')
        window._update_remote_icon_glow()
        assert window._remote_client_names == {'claude'}
        now[0] += 11
        window._update_remote_icon_glow()
        assert window._remote_client_names == set()
        assert stopped == [('claude', [{'kind': 'button', 'key': 'roscore', 'started': 1000.0}])]
        text = server.adapter.tab_text('log')
        assert 'claude: presence expired' in text and 'stopped button roscore' in text
    finally:
        window._stop_remote_control()
        window.close()


def test_refresh_installed_skill_updates_only_existing_copy(tmp_path):
    from mobipick_gui.remote_client import bundled_skill_path, install_skill, refresh_installed_skill

    root = tmp_path / 'skills'
    assert refresh_installed_skill(root) is None  # nothing installed: opt-in stays opt-in
    installed = install_skill(root)
    assert refresh_installed_skill(root) is None  # already current
    installed.write_text('stale copy\n')
    assert refresh_installed_skill(root) == installed
    assert installed.read_bytes() == bundled_skill_path().read_bytes()


def test_remote_glow_icon_honours_colour():
    from PyQt5.QtCore import Qt
    from PyQt5.QtGui import QColor, QIcon, QPixmap
    from PyQt5.QtWidgets import QApplication

    from mobipick_gui.main_window import (
        REMOTE_ICON_GLOW_COLOR,
        REMOTE_ICON_GLOW_COLOR_IN_USE,
        remote_glow_icon,
    )

    app = QApplication.instance() or QApplication([])  # noqa: F841 - keep alive
    source = QPixmap(64, 64)
    source.fill(Qt.white)
    base = QIcon(source)

    def corner(color):
        image = remote_glow_icon(base, 1.0, size=64, color=color).pixmap(256, 256).toImage()
        return image.pixelColor(1, 1)

    blue, green = corner(REMOTE_ICON_GLOW_COLOR), corner(REMOTE_ICON_GLOW_COLOR_IN_USE)
    assert blue.blue() > blue.green() and green.green() > green.blue()


def test_gnome_app_glow_sends_colour_changes(monkeypatch):
    from mobipick_gui.window_control import GnomeAppGlow

    glow = GnomeAppGlow('app', probe=False)
    glow._available = True
    calls = []
    monkeypatch.setattr(GnomeAppGlow, '_call', lambda self, method, *args: calls.append((method, args)) or (1,))

    glow.set_level(0.5, 'rgba(1, 2, 3, 0.95)')
    deadline = time.time() + 5
    while not calls and time.time() < deadline:
        time.sleep(0.01)
    # same level, new colour must still be sent
    glow.set_level(0.5, 'rgba(9, 9, 9, 0.95)')
    deadline = time.time() + 5
    while len(calls) < 2 and time.time() < deadline:
        time.sleep(0.01)
    glow.clear()
    colours = [args[2] for method, args in calls if method == 'SetAppGlow']
    assert 'rgba(1, 2, 3, 0.95)' in colours and 'rgba(9, 9, 9, 0.95)' in colours


def test_reload_endpoint_rereads_button_profile():
    adapter = FakeAdapter()
    server = RemoteControlServer(adapter, host='127.0.0.1', port=0)
    host, port = server.start()
    try:
        api = _Api(f'http://{host}:{port}')
        status, payload = api('POST', '/reload', {})
        assert status == 200 and payload['reloaded'] is True
        assert payload['buttons'] == ['sim', 'rviz'] and adapter.reloads == [True]
        assert api('GET', '/reload')[0] == 404
    finally:
        server.stop()


def test_reload_configuration_picks_up_changed_command(tmp_path, monkeypatch):
    """The real reload path: editing the profile changes the button command."""
    app, window = _make_window(tmp_path, monkeypatch)
    try:
        before = window._config_buttons.get('sim', {}).get('command')
        layouts = [list(window._button_layout)]
        changed = [
            dict(entry, command='roslaunch demo demo_sim.launch grasp_fix:=true')
            if str(entry.get('key')) == 'sim' else entry
            for entry in layouts[0]
        ]
        monkeypatch.setattr('mobipick_gui.main_window.load_button_layout', lambda *a, **k: changed)
        summary = window.reload_configuration()
        assert summary['reloaded'] is True
        after = window._config_buttons.get('sim', {}).get('command')
        assert after != before and 'grasp_fix:=true' in after
        assert 'sim' in summary['buttons']
    finally:
        window._stop_remote_control()
        window.close()


def test_presence_supports_several_named_agents():
    """The client name is runtime data: any agent may use its own name."""
    adapter = FakeAdapter()
    server = RemoteControlServer(adapter, host='127.0.0.1', port=0)
    adapter.server = server
    host, port = server.start()
    try:
        api = _Api(f'http://{host}:{port}')
        codex = api('POST', '/presence', {'name': 'codex'})[1]['client']['token']
        alice = api('POST', '/presence', {'name': 'alice-laptop'})[1]['client']['token']
        assert {c['name'] for c in server.clients()} == {'codex', 'alice-laptop'}
        assert server.in_use

        # Ownership is tracked per agent, not globally.
        api('POST', '/buttons/roscore/start', {})
        owner = {e['key'] for e in server.owned_by('codex')}
        assert owner == {'roscore'}  # first-declared agent owns what it started
        assert server.owned_by('alice-laptop') == []

        # The GUI stays "in use" until the last agent leaves.
        api('DELETE', '/presence', {'name': 'alice-laptop', 'token': alice})
        assert server.in_use and [c['name'] for c in server.clients()] == ['codex']
        api('DELETE', '/presence', {'name': 'codex', 'token': codex})
        assert not server.in_use
        assert {e['key'] for e in server.take_owned('codex')} == {'roscore'}
    finally:
        server.stop()


def test_gnome_app_glow_survives_a_transient_call_failure(monkeypatch):
    from mobipick_gui.window_control import GnomeAppGlow

    glow = GnomeAppGlow('app', probe=False)
    glow._available = True
    glow.retry_delay = 0.01
    calls = []

    def fake_call(self, method, *args):
        calls.append(method)
        if method == 'Version':
            return (4,)
        # first SetAppGlow fails the way a disposed dock icon makes it fail
        return None if calls.count('SetAppGlow') == 1 else (1,)

    monkeypatch.setattr(GnomeAppGlow, '_call', fake_call)
    glow.set_level(0.7)
    deadline = time.time() + 5
    while calls.count('SetAppGlow') < 2 and time.time() < deadline:
        time.sleep(0.01)
    assert glow.available
    assert calls.count('SetAppGlow') == 2 and 'Version' in calls
    glow.clear()


def test_gnome_app_glow_gives_up_when_extension_is_gone(monkeypatch):
    from mobipick_gui.window_control import GnomeAppGlow

    glow = GnomeAppGlow('app', probe=False)
    glow._available = True
    monkeypatch.setattr(GnomeAppGlow, '_call', lambda self, method, *args: None)
    glow.set_level(0.7)
    deadline = time.time() + 5
    while glow.available and time.time() < deadline:
        time.sleep(0.01)
    assert not glow.available


def test_recording_endpoints_control_segments_and_emit_events():
    adapter = FakeAdapter()
    server = RemoteControlServer(adapter, host='127.0.0.1', port=0)
    host, port = server.start()
    try:
        api = _Api(f'http://{host}:{port}')
        status, payload = api('GET', '/recording')
        assert status == 200 and payload['recording']['active'] is False
        status, payload = api('POST', '/recording/pause', {})
        assert status == 200 and payload['accepted'] is False and 'no recording' in payload['reason']
        status, payload = api('POST', '/recording/start', {})
        assert status == 200 and payload['accepted'] is True and payload['segments'] == 1
        status, payload = api('POST', '/recording/start', {})
        assert payload['accepted'] is False
        status, payload = api('POST', '/recording/pause', {})
        assert payload['accepted'] is True and payload['paused'] is True
        status, payload = api('POST', '/recording/resume', {})
        assert payload['accepted'] is True and payload['paused'] is False and payload['segments'] == 2
        status, payload = api('POST', '/recording/stop', {})
        assert payload['accepted'] is True and payload['active'] is False
        assert api('POST', '/recording/rewind', {})[0] == 404
        names = [e['name'] for e in api('GET', '/events?since=0')[1]['events']]
        assert ['recording_started', 'recording_paused', 'recording_resumed', 'recording_stopped'] == [
            n for n in names if n.startswith('recording_')
        ]
    finally:
        server.stop()


def test_recording_is_stopped_when_its_client_leaves():
    adapter = FakeAdapter()
    server = RemoteControlServer(adapter, host='127.0.0.1', port=0)
    host, port = server.start()
    try:
        api = _Api(f'http://{host}:{port}')
        token = api('POST', '/presence', {'name': 'claude'})[1]['client']['token']
        api('POST', '/recording/start', {})
        assert server.owned_by('claude') and server.owned_by('claude')[0]['kind'] == 'recording'
        api('DELETE', '/presence', {'name': 'claude', 'token': token})
        leftovers = server.take_owned('claude')
        assert [(e['kind'], e['key']) for e in leftovers] == [('recording', 'screen')]
        assert server.take_owned('claude') == []
    finally:
        server.stop()


def test_presence_bye_needs_the_registration_token():
    """Two sessions that picked the same name cannot withdraw each other."""
    adapter = FakeAdapter()
    server = RemoteControlServer(adapter, host='127.0.0.1', port=0)
    host, port = server.start()
    try:
        api = _Api(f'http://{host}:{port}')
        status, payload = api('POST', '/presence', {'name': 'claude', 'note': 'session A'})
        token = payload['client']['token']
        assert 'token' not in api('GET', '/presence')[1]['clients'][0]
        # session B, same name: its refresh gets no token and its bye is refused
        assert 'token' not in api('POST', '/presence', {'name': 'claude', 'note': 'session B'})[1]['client']
        status, payload = api('DELETE', '/presence', {'name': 'claude'})
        assert status == 409 and 'another session' in payload['error']
        status, payload = api('DELETE', '/presence', {'name': 'claude', 'token': 'wrong'})
        assert status == 409 and server.in_use
        status, payload = api('DELETE', '/presence', {'name': 'claude', 'token': token})
        assert status == 200 and payload['removed'] is True and not server.in_use
    finally:
        server.stop()
