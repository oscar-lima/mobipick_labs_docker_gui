"""Command-line client for the GUI remote-control API.

Installed as ``mobipick-labs-docker-gui-remote``. It only needs the Python
standard library so it can run on any host that can reach the GUI, including
from automation agents. Every command prints JSON unless ``--text`` is given,
in which case output lines are printed raw (handy for humans and for agents
that want fewer tokens).

Examples::

    mobipick-labs-docker-gui-remote status
    mobipick-labs-docker-gui-remote click auto_launch --wait window_layout_applied --timeout 240
    mobipick-labs-docker-gui-remote shell open
    mobipick-labs-docker-gui-remote shell exec 1 "rostopic list" --tail 20
    mobipick-labs-docker-gui-remote shell exec 1 "roslaunch pkg file.launch" --no-wait
    mobipick-labs-docker-gui-remote shell output 1 --follow --grep ERROR
    mobipick-labs-docker-gui-remote shell interrupt 1
"""
from __future__ import annotations

import argparse
import json
import os
import shutil
import sys
from pathlib import Path
from typing import Any, Iterator, Sequence
from urllib import error as urlerror
from urllib import request as urlrequest
from urllib.parse import urlencode

DEFAULT_URL = os.environ.get('MOBIPICK_GUI_REMOTE_URL') or 'http://127.0.0.1:8765'
SKILL_NAME = 'mobipick-gui-remote'


def bundled_skill_path() -> Path:
    """Return the packaged Claude Code skill file for the remote API."""
    return Path(__file__).resolve().parent / 'resources' / 'skills' / SKILL_NAME / 'SKILL.md'


def install_skill(target_root: Path) -> Path:
    """Copy the bundled skill into ``<target_root>/<skill name>/SKILL.md``."""
    source = bundled_skill_path()
    destination = Path(target_root).expanduser() / SKILL_NAME / 'SKILL.md'
    destination.parent.mkdir(parents=True, exist_ok=True)
    shutil.copyfile(source, destination)
    return destination


DEFAULT_SKILLS_ROOT = Path('~/.claude/skills')


def refresh_installed_skill(target_root: Path | str = DEFAULT_SKILLS_ROOT) -> Path | None:
    """Overwrite a previously installed copy of the skill when the bundled one changed.

    Only an existing installation is touched (installing is opt-in through
    ``skill --install``), so the repository stays the single source of truth
    and a GUI upgrade cannot leave a stale skill behind.  Returns the updated
    path, or ``None`` when nothing was installed or it was already current.
    """
    destination = Path(target_root).expanduser() / SKILL_NAME / 'SKILL.md'
    source = bundled_skill_path()
    try:
        if not destination.is_file() or destination.read_bytes() == source.read_bytes():
            return None
        shutil.copyfile(source, destination)
    except OSError:
        return None
    return destination


class RemoteClient:
    """Tiny JSON-over-HTTP client for :mod:`mobipick_gui.remote_control`."""

    def __init__(self, base_url: str = DEFAULT_URL, token: str | None = None, timeout: float = 30.0):
        self.base_url = base_url.rstrip('/')
        self.token = token if token is not None else os.environ.get('MOBIPICK_GUI_REMOTE_TOKEN', '')
        self.timeout = timeout

    def _request(self, method: str, path: str, *, query: dict | None = None, body: dict | None = None, timeout: float | None = None):
        url = f'{self.base_url}{path}'
        if query:
            clean = {k: v for k, v in query.items() if v is not None and v != ''}
            if clean:
                url += ('&' if '?' in url else '?') + urlencode(clean)
        data = None
        headers = {'Accept': 'application/json'}
        if body is not None:
            data = json.dumps(body).encode('utf-8')
            headers['Content-Type'] = 'application/json'
        if self.token:
            headers['Authorization'] = f'Bearer {self.token}'
        req = urlrequest.Request(url, data=data, method=method, headers=headers)
        return urlrequest.urlopen(req, timeout=self.timeout if timeout is None else timeout)

    def call(self, method: str, path: str, *, query: dict | None = None, body: dict | None = None, timeout: float | None = None) -> dict:
        try:
            with self._request(method, path, query=query, body=body, timeout=timeout) as response:
                raw = response.read()
        except urlerror.HTTPError as exc:
            raw = exc.read()
            try:
                payload = json.loads(raw.decode('utf-8'))
            except Exception:
                payload = {'ok': False, 'error': f'HTTP {exc.code}: {raw.decode("utf-8", "replace")}'}
            payload.setdefault('ok', False)
            payload['http_status'] = exc.code
            return payload
        return json.loads(raw.decode('utf-8'))

    def stream(self, path: str, *, query: dict | None = None, timeout: float | None = None) -> Iterator[dict]:
        query = dict(query or {})
        query['follow'] = '1'
        with self._request('GET', path, query=query, timeout=timeout) as response:
            for line in response:
                line = line.strip()
                if line:
                    yield json.loads(line.decode('utf-8'))


def _print(payload: Any, *, text: bool = False) -> None:
    if text and isinstance(payload, dict):
        lines = payload.get('output') or payload.get('lines')
        if isinstance(lines, list):
            for entry in lines:
                print(entry['text'] if isinstance(entry, dict) else entry)
            extras = {
                key: payload[key]
                for key in ('exit_code', 'timed_out', 'truncated', 'matched_lines', 'line_count', 'error')
                if key in payload
            }
            command = payload.get('command')
            if isinstance(command, dict) and 'exit_code' in command:
                extras['exit_code'] = command['exit_code']
                extras['running'] = command.get('running')
            if extras:
                print('--', json.dumps(extras), file=sys.stderr)
            return
    print(json.dumps(payload, indent=2, ensure_ascii=False))


def _events_list(value: str | None) -> list[str]:
    if not value:
        return []
    return [item.strip() for item in value.split(',') if item.strip()]


def _arg_values(items: Sequence[str]) -> dict[str, str]:
    """Parse NAME=VALUE pairs into a dict; raises SystemExit on a malformed pair."""
    values: dict[str, str] = {}
    for item in items:
        name, sep, value = str(item).partition('=')
        if not sep or not name.strip():
            raise SystemExit(f'expected NAME=VALUE, got {item!r}')
        values[name.strip()] = value.strip()
    return values


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog='mobipick-labs-docker-gui-remote',
        description='Drive a running Mobipick Labs Docker GUI over its remote control API.',
    )
    parser.add_argument('--url', default=DEFAULT_URL, help='API base URL (env MOBIPICK_GUI_REMOTE_URL)')
    parser.add_argument('--token', default=None, help='Bearer token (env MOBIPICK_GUI_REMOTE_TOKEN)')
    parser.add_argument('--text', action='store_true', help='Print output lines as plain text instead of JSON')
    parser.add_argument('--http-timeout', type=float, default=None, help='Socket timeout in seconds')
    sub = parser.add_subparsers(dest='command', required=True)

    sub.add_parser('api', help='List API endpoints')
    sub.add_parser('status', help='GUI status summary')
    sub.add_parser('buttons', help='List toolbar buttons with their states, arguments and readiness estimates')
    sub.add_parser('args', help='List toolbar argument dropdowns (name, value, options) and the world selector')
    p = sub.add_parser('set-args', help='Select toolbar argument values, e.g. anygrasp_mode=real world=moelk_tables')
    p.add_argument('values', nargs='+', metavar='NAME=VALUE')

    p = sub.add_parser('hello', help='Declare that you are using the GUI (lights the window icon until "bye")')
    p.add_argument('name', help='your agent name, shown in the GUI log (any string, e.g. claude, codex, alice-laptop)')
    p.add_argument('--ttl', type=float, default=None, help='seconds until the declaration expires (default 600, max 1800); repeat hello to refresh')
    p.add_argument('--note', default='', help='what you are doing, shown in /status')
    p = sub.add_parser('bye', help='Declare that you are done; the GUI stops what you started unless --keep')
    p.add_argument('name', help='client name given to hello')
    p.add_argument('--keep', action='store_true', help='leave the processes you started running')
    p = sub.add_parser('stop-tab', help='Stop the process behind a log tab (button process, customN command, remote shell)')
    p.add_argument('key', help='tab key as listed by "tabs", e.g. custom1, sim, terminal-remote1')
    sub.add_parser('clients', help='List clients that declared presence')
    sub.add_parser('reload', help='Re-read config and button profile without restarting the GUI')

    for name, help_text in (
        ('click', 'Press a toolbar button'),
        ('start', 'Press a button only if it is not running'),
        ('stop', 'Press a button only if it is running'),
    ):
        p = sub.add_parser(name, help=help_text)
        p.add_argument('key', help='button key, e.g. roscore, sim, rviz, auto_launch, terminal')
        p.add_argument('--wait', dest='wait_for', default=None, help='comma-separated event names to wait for after pressing, e.g. button_ready')
        p.add_argument('--timeout', type=float, default=180.0, help='seconds to wait for the event')
        p.add_argument('--arg', dest='args', action='append', default=[], metavar='NAME=VALUE', help='select a toolbar argument before pressing (repeatable)')

    p = sub.add_parser('wait', help='Block until an event arrives')
    p.add_argument('events', help='comma-separated event names, e.g. window_layout_applied,auto_launch_complete')
    p.add_argument('--since', type=int, default=None, help='only accept events newer than this sequence number')
    p.add_argument('--timeout', type=float, default=180.0)
    p.add_argument('--key', default=None, help='only accept keyed events (button_state, button_ready, process_finished) for this button/tab')

    p = sub.add_parser('events', help='List or follow events')
    p.add_argument('--since', type=int, default=0)
    p.add_argument('--names', default=None, help='comma-separated event filter')
    p.add_argument('--follow', action='store_true', help='stream events as NDJSON until --timeout')
    p.add_argument('--timeout', type=float, default=300.0)
    p.add_argument('--limit', type=int, default=200)

    sub.add_parser('tabs', help='List log tabs')
    p = sub.add_parser('tab', help='Read a log tab')
    p.add_argument('key')
    p.add_argument('--tail', type=int, default=100)
    p.add_argument('--grep', default=None)

    sub.add_parser('dialogs', help='Show the active modal dialog')
    p = sub.add_parser('dismiss', help='Close the active modal dialog')
    p.add_argument('button', nargs='?', default='reject', help='button text, or accept/reject')

    p = sub.add_parser('command', help='Run text through the GUI custom command box')
    p.add_argument('text')

    sub.add_parser('quit', help='Close the GUI with cleanup')

    p = sub.add_parser('skill', help='Print or install the bundled Claude Code skill for this API')
    p.add_argument('--install', metavar='SKILLS_DIR', default=None,
                   help='copy the skill into this skills directory, e.g. ~/.claude/skills or <repo>/.claude/skills')
    p.add_argument('--path', action='store_true', help='print only the path of the bundled skill file')

    shell = sub.add_parser('shell', help='Persistent shell sessions on the robot or in the ROS container')
    shell_sub = shell.add_subparsers(dest='shell_command', required=True)
    shell_sub.add_parser('list', help='List sessions')
    p = shell_sub.add_parser('open', help='Open a session (blocks until the shell is ready)')
    p.add_argument('--name', default='')
    p.add_argument('--no-stream', dest='stream', action='store_false', default=True, help='default exec responses omit output')
    root = p.add_mutually_exclusive_group()
    root.add_argument('--root', dest='root', action='store_true', default=None)
    root.add_argument('--user', dest='root', action='store_false')
    where = p.add_mutually_exclusive_group()
    where.add_argument('--robot', dest='robot', action='store_true', default=None,
                       help='ssh onto the robot (the default in remote ROS master mode)')
    where.add_argument('--container', dest='robot', action='store_false',
                       help='open the shell in the local ROS tool container instead')
    p.add_argument('--timeout', type=float, default=None)
    p = shell_sub.add_parser('info', help='Session details')
    p.add_argument('id', type=int)
    p = shell_sub.add_parser('exec', help='Run a command in a session')
    p.add_argument('id', type=int)
    p.add_argument('cmd', help='shell command (quote it)')
    stream = p.add_mutually_exclusive_group()
    stream.add_argument('--stream', dest='stream', action='store_true', default=None, help='include output lines')
    stream.add_argument('--no-stream', dest='stream', action='store_false', help='return only exit code and line count')
    p.add_argument('--tail', type=int, default=None, help='only the last N lines')
    p.add_argument('--grep', default=None, help='only lines matching this regex')
    p.add_argument('--max-lines', type=int, default=None)
    p.add_argument('--timeout', type=float, default=None, help='seconds to wait for completion')
    p.add_argument('--no-wait', dest='wait', action='store_false', default=True, help='return immediately (background command)')
    p = shell_sub.add_parser('output', help='Fetch or follow buffered output')
    p.add_argument('id', type=int)
    p.add_argument('--since', type=int, default=None)
    p.add_argument('--command', dest='command_id', type=int, default=None, help='restrict to one command id')
    p.add_argument('--tail', type=int, default=None)
    p.add_argument('--grep', default=None)
    p.add_argument('--max-lines', type=int, default=None)
    p.add_argument('--follow', action='store_true', help='stream new lines until the command finishes or --timeout')
    p.add_argument('--timeout', type=float, default=300.0)
    p = shell_sub.add_parser('interrupt', help='Send a signal to the running command')
    p.add_argument('id', type=int)
    p.add_argument('--signal', default='INT', help='INT (default), TERM, KILL, HUP')
    p = shell_sub.add_parser('stream', help='Change the session default for streaming output')
    p.add_argument('id', type=int)
    p.add_argument('value', choices=['on', 'off'])
    p = shell_sub.add_parser('close', help='Close a session')
    p.add_argument('id', type=int)
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    parser = _build_parser()
    args = parser.parse_args(list(sys.argv[1:] if argv is None else argv))
    client = RemoteClient(args.url, args.token, timeout=args.http_timeout or 30.0)
    text = bool(args.text)

    def long_timeout(seconds: float | None) -> float:
        return (seconds or 0) + 15.0 if args.http_timeout is None else args.http_timeout

    try:
        cmd = args.command
        if cmd == 'api':
            payload = client.call('GET', '/')
        elif cmd == 'status':
            payload = client.call('GET', '/status')
        elif cmd == 'buttons':
            payload = client.call('GET', '/buttons')
        elif cmd == 'args':
            payload = client.call('GET', '/args')
        elif cmd == 'set-args':
            payload = client.call('POST', '/args', body=_arg_values(args.values))
        elif cmd == 'hello':
            body = {'name': args.name, 'note': args.note}
            if args.ttl is not None:
                body['ttl_s'] = args.ttl
            payload = client.call('POST', '/presence', body=body)
        elif cmd == 'bye':
            payload = client.call('DELETE', '/presence', body={'name': args.name, 'keep': bool(args.keep)})
        elif cmd == 'stop-tab':
            payload = client.call('POST', f'/tabs/{args.key}/stop', body={})
        elif cmd == 'clients':
            payload = client.call('GET', '/presence')
        elif cmd == 'reload':
            payload = client.call('POST', '/reload', body={})
        elif cmd in {'click', 'start', 'stop'}:
            body: dict = {}
            if args.wait_for:
                body = {'wait_for': _events_list(args.wait_for), 'timeout': args.timeout}
            if args.args:
                body['args'] = _arg_values(args.args)
            payload = client.call('POST', f'/buttons/{args.key}/{cmd}', body=body, timeout=long_timeout(args.timeout))
        elif cmd == 'wait':
            body = {'events': _events_list(args.events), 'timeout': args.timeout}
            if args.since is not None:
                body['since'] = args.since
            if args.key:
                body['key'] = args.key
            payload = client.call('POST', '/wait', body=body, timeout=long_timeout(args.timeout))
        elif cmd == 'events':
            query = {'since': args.since, 'names': args.names, 'limit': args.limit, 'timeout': args.timeout}
            if args.follow:
                for item in client.stream('/events', query=query, timeout=long_timeout(args.timeout)):
                    print(json.dumps(item, ensure_ascii=False), flush=True)
                return 0
            payload = client.call('GET', '/events', query=query)
        elif cmd == 'tabs':
            payload = client.call('GET', '/tabs')
        elif cmd == 'tab':
            payload = client.call('GET', f'/tabs/{args.key}', query={'tail': args.tail, 'grep': args.grep})
        elif cmd == 'dialogs':
            payload = client.call('GET', '/dialogs')
        elif cmd == 'dismiss':
            payload = client.call('POST', '/dialogs/dismiss', body={'button': args.button})
        elif cmd == 'command':
            payload = client.call('POST', '/command', body={'command': args.text})
        elif cmd == 'quit':
            payload = client.call('POST', '/quit', body={})
        elif cmd == 'skill':
            source = bundled_skill_path()
            if args.install:
                destination = install_skill(Path(args.install))
                print(f'installed {destination}')
            elif args.path:
                print(source)
            else:
                print(source.read_text(encoding='utf-8'), end='')
            return 0
        elif cmd == 'shell':
            payload = _shell_command(client, args, text)
            if payload is None:
                return 0
        else:  # pragma: no cover - argparse enforces choices
            parser.error(f'unknown command {cmd}')
            return 2
    except urlerror.URLError as exc:
        print(json.dumps({'ok': False, 'error': f'cannot reach {client.base_url}: {exc.reason}'}), file=sys.stderr)
        return 3
    except KeyboardInterrupt:
        return 130
    _print(payload, text=text)
    return 0 if payload.get('ok', True) else 1


def _shell_command(client: RemoteClient, args: argparse.Namespace, text: bool) -> dict | None:
    sc = args.shell_command
    if sc == 'list':
        return client.call('GET', '/shell')
    if sc == 'open':
        body: dict = {'name': args.name, 'stream': args.stream}
        if args.root is not None:
            body['root'] = args.root
        if args.robot is not None:
            body['robot'] = args.robot
        if args.timeout is not None:
            body['timeout'] = args.timeout
        return client.call('POST', '/shell', body=body, timeout=(args.timeout or 180.0) + 15.0)
    if sc == 'info':
        return client.call('GET', f'/shell/{args.id}')
    if sc == 'exec':
        body = {'command': args.cmd, 'wait': args.wait}
        for key in ('stream', 'tail', 'grep', 'max_lines', 'timeout'):
            value = getattr(args, key)
            if value is not None:
                body[key] = value
        return client.call(
            'POST',
            f'/shell/{args.id}/exec',
            body=body,
            timeout=(args.timeout or 60.0) + 15.0,
        )
    if sc == 'output':
        query = {
            'since': args.since,
            'command': args.command_id,
            'tail': args.tail,
            'grep': args.grep,
            'max_lines': args.max_lines,
            'timeout': args.timeout,
        }
        if args.follow:
            for item in client.stream(f'/shell/{args.id}/output', query=query, timeout=args.timeout + 15.0):
                if text and 'text' in item:
                    print(item['text'], flush=True)
                else:
                    print(json.dumps(item, ensure_ascii=False), flush=True)
            return None
        return client.call('GET', f'/shell/{args.id}/output', query=query)
    if sc == 'interrupt':
        return client.call('POST', f'/shell/{args.id}/interrupt', body={'signal': args.signal})
    if sc == 'stream':
        return client.call('POST', f'/shell/{args.id}/settings', body={'stream': args.value == 'on'})
    if sc == 'close':
        return client.call('DELETE', f'/shell/{args.id}')
    raise SystemExit(f'unknown shell command {sc}')


if __name__ == '__main__':  # pragma: no cover
    sys.exit(main())


__all__ = ['RemoteClient', 'bundled_skill_path', 'install_skill', 'main', 'refresh_installed_skill']
