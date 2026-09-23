"""Bridge between :class:`RemoteControlServer` and :class:`MainWindow`.

Widget actions run through :class:`~mobipick_gui.remote_control.GuiInvoker`.
Read-only status, button, and tab inventories are snapshots that HTTP worker
threads can read without depending on the Qt event thread.
"""
from __future__ import annotations

import html
import threading
import uuid
from typing import TYPE_CHECKING

from PyQt5.QtWidgets import QApplication, QDialog, QMessageBox, QPushButton

from .remote_control import (
    Conflict,
    GuiAdapter,
    NotFound,
    RemoteControlError,
    RemoteShellSession,
)
from .version import __version__

if TYPE_CHECKING:  # pragma: no cover
    from .main_window import MainWindow

REMOTE_SHELL_TAB_PREFIX = 'terminal-remote'
REMOTE_SHELL_XHOST_SOURCE = 'remote-shell'


class MainWindowRemoteAdapter(GuiAdapter):
    """Expose MainWindow state and actions to the remote-control server."""

    def __init__(self, window: 'MainWindow'):
        self.window = window
        self._snapshot_lock = threading.Lock()
        self._snapshot: dict = {}
        # Lightweight harnesses and shell-only adapters do not necessarily
        # provide widgets. A real MainWindow publishes immediately.
        if hasattr(window, '_button_widgets') and hasattr(window, 'tasks'):
            self.publish_snapshot()

    # -- status --------------------------------------------------------

    def _status_from_gui(self) -> dict:
        window = self.window
        registry = getattr(window, '_workspace_registry', None)
        return {
            'version': __version__,
            'workspace': getattr(registry, 'active', None) if registry else None,
            'image': getattr(window, '_selected_image', None),
            'world': window._current_world() if hasattr(window, '_current_world') else None,
            'remote_master': bool(window._remote_master_enabled()),
            'master_uri': window._current_master_uri(),
            'shell': self.shell_targets(),
            'roscore_running': bool(getattr(window, '_roscore_running_cached', False)),
            'sim_running': bool(getattr(window, '_sim_running_cached', False)),
            'terminal_running': bool(getattr(window, '_terminal_running_cached', False)),
            'auto_launch_running': bool(getattr(window, '_auto_launch_running', False)),
            'auto_launch_active': list(getattr(window, '_auto_launch_active_keys', []) or []),
            'auto_launch_ready': sorted(getattr(window, '_auto_launch_ready_keys', set()) or []),
            'exit_in_progress': bool(getattr(window, '_exit_in_progress', False)),
            'buttons': self.buttons(),
            'tabs': self.tabs(),
            'dialog': self.active_dialog(),
        }

    def publish_snapshot(self) -> None:
        """Publish widget-derived state for HTTP threads to read safely."""
        snapshot = self._status_from_gui()
        with self._snapshot_lock:
            self._snapshot = snapshot

    def status(self) -> dict:
        """Return the latest GUI snapshot without accessing Qt objects."""
        with self._snapshot_lock:
            return dict(self._snapshot)

    def cached_buttons(self) -> list[dict]:
        with self._snapshot_lock:
            return [dict(entry) for entry in self._snapshot.get('buttons', [])]

    def cached_tabs(self) -> list[dict]:
        with self._snapshot_lock:
            return [dict(entry) for entry in self._snapshot.get('tabs', [])]

    def shell_targets(self) -> dict:
        """Describe where ``POST /shell`` can open a session, and where by default."""
        window = self.window
        target = window._robot_ssh_target()
        return {
            'default': 'robot' if (
                target and window._robot_shell_by_default()
            ) else 'container',
            'container_service': window._ros_tool_service(),
            'robot_available': bool(target),
            'robot_target': target,
            'hint': (
                'ROS work (rostopic, roslaunch, the workspace) belongs in the '
                'container shell; open a robot shell ({"robot": true}) only to '
                'debug the robot PC itself'
                if target else
                'robot shells need remote ROS master mode'
            ),
        }

    # -- screen recording ------------------------------------------------

    def recording(self) -> dict:
        return self.window.recording_status()

    def recording_action(self, action: str) -> dict:
        window = self.window
        status_before = window.recording_status()
        if action == 'start':
            if status_before.get('active'):
                return {'accepted': False, 'reason': 'recording already active', **window.recording_status()}
            window._log_event('remote client started screen recording')
            window._start_screen_recording()
            status = window.recording_status()
            return {
                'accepted': bool(status.get('active')),
                'reason': None if status.get('active') else 'ffmpeg did not start',
                **status,
            }
        if not status_before.get('active'):
            return {'accepted': False, 'reason': 'no recording active', **status_before}
        if action == 'pause':
            accepted = window.pause_recording()
            reason = None if accepted else 'recording is already paused or stopping'
        elif action == 'resume':
            accepted = window.resume_recording()
            reason = None if accepted else 'recording is not paused'
        elif action == 'stop':
            window._stop_screen_recording(save_logs=True, reason='remote client stopped screen recording')
            accepted = True
            reason = None
        else:
            raise NotFound(f'unknown recording action {action!r}')
        return {'accepted': accepted, 'reason': reason, **window.recording_status()}

    # -- buttons -------------------------------------------------------

    def _button_widget(self, key: str) -> QPushButton | None:
        window = self.window
        widget = getattr(window, '_button_widgets', {}).get(key)
        if widget is None:
            widget = getattr(window, f'{key}_button', None)
        return widget if isinstance(widget, QPushButton) else None

    def _button_keys(self) -> list[str]:
        window = self.window
        keys: list[str] = []
        for key in (
            'roscore',
            *getattr(window, '_config_button_order', []),
            'terminal',
            'auto_launch',
        ):
            if key not in keys:
                keys.append(key)
        for key in getattr(window, '_button_widgets', {}):
            if key not in keys:
                keys.append(key)
        return keys

    def _describe_button(self, key: str) -> dict:
        window = self.window
        widget = self._button_widget(key)
        config = window._config_buttons.get(key, {})
        state = window._toggle_states.get(key)
        if state is None and widget is not None:
            state = 'red' if widget.isEnabled() else 'grey'
        return {
            'key': key,
            'label': str(config.get('label') or (widget.text() if widget else key)),
            'text': widget.text() if widget else '',
            'state': state,
            'enabled': bool(widget.isEnabled()) if widget else False,
            'running': state == 'green',
            'busy': state == 'yellow',
            'kind': str(config.get('kind') or 'builtin'),
            'command': config.get('command'),
            'tooltip': (widget.toolTip() if widget else '') or str(config.get('tooltip') or ''),
            'runs_on': 'host' if window._config_runs_on_host(config) else 'container',
            'tab': key if key in window.tasks else None,
            'args': window.button_args(key),
            'full_command': (
                window._command_with_generic_args(str(config.get('command')), config)
                if config.get('command') else None
            ),
            **window.button_readiness(key),
        }

    def buttons(self) -> list[dict]:
        return [self._describe_button(key) for key in self._button_keys()]

    # -- toolbar arguments ----------------------------------------------

    def args(self) -> list[dict]:
        return self.window.generic_args()

    def set_args(self, values: dict) -> list[dict]:
        try:
            return self.window.set_generic_args(values)
        except ValueError as exc:
            raise RemoteControlError(str(exc)) from exc

    def press_button(self, key: str, action: str = 'click') -> dict:
        widget = self._button_widget(key)
        if widget is None:
            raise NotFound(
                f'unknown button {key!r}; known keys: {", ".join(self._button_keys())}'
            )
        before = self._describe_button(key)
        if before['busy']:
            return {
                'accepted': False,
                'reason': f'button is busy: {before["text"]}',
                'button': before,
            }
        if not before['enabled']:
            return {
                'accepted': False,
                'reason': f'button is disabled: {before["text"]}',
                'button': before,
            }
        if action == 'start' and before['running']:
            return {'accepted': False, 'reason': 'already running', 'button': before}
        if action == 'stop' and not before['running']:
            return {'accepted': False, 'reason': 'not running', 'button': before}
        if not before['running']:
            blocked = self.window._start_blocked_reason(key)
            if blocked:
                return {
                    'accepted': False,
                    'reason': f'blocked by option rules: {blocked}',
                    'button': before,
                }
        self.window._log_info(f'remote control: {action} {key}')
        widget.click()
        return {
            'accepted': True,
            'action': action,
            'was_running': bool(before['running']),
            'button': self._describe_button(key),
        }

    # -- tabs ----------------------------------------------------------

    def tabs(self) -> list[dict]:
        window = self.window
        entries = []
        for key, tab in window.tasks.items():
            index = window.tabs.indexOf(tab.output)
            entries.append({
                'key': key,
                'label': window.tabs.tabText(index) if index >= 0 else tab.label,
                'running': bool(tab.is_running()),
                'container': tab.container_name,
                'closable': bool(tab.closable),
            })
        return entries

    def reload_configuration(self) -> dict:
        """Re-read config and the button profile without restarting the GUI."""
        return self.window.reload_configuration()

    def stop_tab(self, key: str) -> dict:
        """Stop the process behind tab ``key``; remote shells are handled by the server."""
        window = self.window
        tab = window.tasks.get(key)
        if tab is None:
            raise NotFound(f'unknown tab {key!r}; known tabs: {", ".join(window.tasks)}')
        if self._button_widget(key) is not None and key in window._toggle_states:
            result = self.press_button(key, 'stop')
            return {'tab': key, 'stopped': bool(result.get('accepted')), 'kind': 'button', **result}
        if not tab.is_running():
            return {'tab': key, 'stopped': False, 'reason': 'not running', 'kind': 'process'}
        window._log_info(f'remote control: stop tab {key}')
        if key == window._script_active_tab_key:
            window.set_script_visual('yellow', 'Stopping Script...', False)
        window._stop_custom_tab(tab)
        return {'tab': key, 'stopped': True, 'kind': 'process'}

    def stop_owned(self, name: str, entries: list[dict]) -> list[str]:
        """Stop what remote client ``name`` left running; returns human-readable notes."""
        window = self.window
        notes: list[str] = []
        for entry in entries:
            kind, key = entry.get('kind'), entry.get('key')
            try:
                if kind == 'button':
                    result = self.press_button(str(key), 'stop')
                    if result.get('accepted'):
                        notes.append(f'stopped button {key}')
                elif kind == 'tab':
                    result = self.stop_tab(str(key))
                    if result.get('stopped'):
                        notes.append(f'stopped tab {key}')
                elif kind == 'recording':
                    if window.recording_status().get('active'):
                        window._stop_screen_recording(
                            save_logs=True, reason=f'remote client {name} left; stopping recording'
                        )
                        notes.append('stopped screen recording')
                elif kind == 'shell':
                    server = window.remote_control
                    if server is not None and any(s.id == key for s in server.sessions()):
                        # Runs on the GUI thread: the container teardown
                        # (docker stop/rm) happens in the background so the
                        # window never freezes after a client lapses.
                        server.close_session(key, wait=False)
                        notes.append(f'closed shell {key}')
            except Exception as exc:  # noqa: BLE001 - best effort cleanup
                notes.append(f'could not stop {kind} {key}: {exc}')
        return notes

    def tab_text(self, key: str) -> str:
        tab = self.window.tasks.get(key)
        if tab is None:
            raise NotFound(
                f'unknown tab {key!r}; known tabs: {", ".join(self.window.tasks)}'
            )
        flush = getattr(tab.output, '_flush', None)
        if callable(flush):
            flush()
        return tab.output.toPlainText()

    # -- dialogs -------------------------------------------------------

    @staticmethod
    def _button_texts(dialog) -> list[str]:
        texts = []
        if isinstance(dialog, QMessageBox):
            for button in dialog.buttons():
                texts.append(button.text().replace('&', ''))
        else:
            for button in dialog.findChildren(QPushButton):
                text = button.text().replace('&', '').strip()
                if text:
                    texts.append(text)
        return texts

    def active_dialog(self) -> dict | None:
        app = QApplication.instance()
        if app is None:
            return None
        dialog = app.activeModalWidget()
        if dialog is None:
            return None
        info = {
            'class': type(dialog).__name__,
            'title': dialog.windowTitle(),
            'buttons': self._button_texts(dialog),
        }
        if isinstance(dialog, QMessageBox):
            info['text'] = dialog.text()
            info['informative_text'] = dialog.informativeText()
        return info

    def dismiss_dialog(self, button: str = 'reject') -> dict:
        app = QApplication.instance()
        dialog = app.activeModalWidget() if app else None
        if dialog is None:
            return {'dismissed': False, 'reason': 'no active modal dialog'}
        info = self.active_dialog()
        wanted = (button or 'reject').strip()
        lowered = wanted.lower()
        if isinstance(dialog, QMessageBox):
            for candidate in dialog.buttons():
                if candidate.text().replace('&', '').strip().lower() == lowered:
                    candidate.click()
                    return {'dismissed': True, 'button': candidate.text().replace('&', ''), 'dialog': info}
        else:
            for candidate in dialog.findChildren(QPushButton):
                if candidate.text().replace('&', '').strip().lower() == lowered:
                    candidate.click()
                    return {'dismissed': True, 'button': candidate.text().replace('&', ''), 'dialog': info}
        if lowered in {'accept', 'ok', 'yes'} and isinstance(dialog, QDialog):
            dialog.accept()
            return {'dismissed': True, 'button': 'accept', 'dialog': info}
        if lowered in {'reject', 'cancel', 'close', 'no'} and isinstance(dialog, QDialog):
            dialog.reject()
            return {'dismissed': True, 'button': 'reject', 'dialog': info}
        raise RemoteControlError(
            f'no button named {wanted!r} on the active dialog',
            dialog=info,
        )

    # -- custom command box -------------------------------------------

    def run_gui_command(self, command: str) -> dict:
        window = self.window
        command_input = getattr(window, 'command_input', None)
        if command_input is None:
            raise RemoteControlError('the GUI command box is not available')
        command_input.setText(command)
        window._log_info(f'remote control: custom command {command}')
        window.run_custom_command()
        key = window._current_tab_key()
        return {'accepted': True, 'tab': key}

    # -- remote shells -------------------------------------------------

    def robot_shell_spec(self, session_id: int, label: str) -> dict:
        """Return the spec of a shell that ssh's onto the robot itself.

        This is the shell for debugging the robot PC - its own nodes, drivers,
        logs, disks, services - which a container cannot see. ROS work stays in
        the container shell, which carries the workspace chain and talks to the
        same master over the network. The ssh key is expected to be in place
        (``BatchMode=yes``: no password prompt, a clear error instead).
        """
        window = self.window
        target = window._robot_ssh_target()
        if not target:
            raise RemoteControlError(
                'no robot to open a shell on: remote ROS master mode is off, '
                'or its ROS_MASTER_URI names no host. Enable it, or ask for a '
                'container shell with {"robot": false}.'
            )
        options = window._robot_ssh_options()
        tab_key = f'{REMOTE_SHELL_TAB_PREFIX}{session_id}'
        argv = ['ssh', *options, target, 'bash', '--noprofile', '--norc']
        master = window._current_master_uri()
        tab = window._ensure_tab(tab_key, label, closable=True)
        tab.container_name = None
        tab.exec_id = None
        window._append_gui_html(
            tab_key,
            f'<i>Remote shell {session_id} starting on the robot '
            f'{html.escape(target)}: {html.escape(" ".join(argv))}</i>',
        )
        window._focus_tab(tab_key)
        window._log_info(
            f'remote control: opening shell {session_id} on {target} over ssh'
        )
        return {
            'argv': argv,
            # ssh inherits the GUI environment (ssh agent, known hosts)
            'env': None,
            'cwd': None,
            'container_name': None,
            'tab_key': tab_key,
            'target': target,
            'runs_on': 'robot',
            # signals go to the command on the robot, not to the local ssh
            'signal_prefix': ['ssh', *options, target],
            'signal_quote': True,
            'init_command': (
                '[ -r /etc/profile ] && . /etc/profile >/dev/null 2>&1; '
                '[ -r "$HOME/.bashrc" ] && . "$HOME/.bashrc" >/dev/null 2>&1; '
                'if [ -z "${ROS_DISTRO:-}" ]; then '
                'for s in /opt/ros/*/setup.bash; do [ -r "$s" ] && '
                '. "$s" >/dev/null 2>&1 && break; done; fi; '
                f'export ROS_MASTER_URI="${{ROS_MASTER_URI:-{master}}}"; '
                'echo "remote shell host: $(whoami)@$(hostname) '
                '(ROS_DISTRO=${ROS_DISTRO:-unset})"; '
                'echo "remote shell ROS_MASTER_URI=${ROS_MASTER_URI:-unset} '
                'ROS_IP=${ROS_IP:-unset}"'
            ),
        }

    def shell_spec(
        self,
        session_id: int,
        label: str,
        *,
        root: bool | None,
        robot: bool | None = None,
    ) -> dict:
        from .main_window import CONTAINER_SCRIPTS_DIR

        window = self.window
        if window._exit_in_progress:
            raise Conflict('the GUI is shutting down')
        if robot is None:
            # the container is the default: ROS work belongs there, and only
            # debugging of the robot PC itself asks for a shell on the robot
            robot = (
                window._remote_master_enabled()
                and window._robot_shell_by_default()
            )
        if robot:
            return self.robot_shell_spec(session_id, label)
        exec_id = uuid.uuid4().hex
        container_name = f'mobipick-remote-shell-{exec_id[:10]}'
        tab_key = f'{REMOTE_SHELL_TAB_PREFIX}{session_id}'
        run_as_root = window._terminal_run_as_root_requested() if root is None else bool(root)
        env_overrides: dict[str, str] = {}
        if run_as_root:
            env_overrides = {
                'MOBIPICK_UID': '0',
                'MOBIPICK_GID': '0',
                'MOBIPICK_HOST_USER': 'root',
                'MOBIPICK_HOST_GROUP': 'root',
                'MOBIPICK_HOST_HOME': '/root',
                'MOBIPICK_CONTAINER_USER': 'root',
            }
        window._grant_x(REMOTE_SHELL_XHOST_SOURCE, log_key='log')
        argv = [
            'docker', 'compose', 'run', '--rm', '-T', '--name', container_name,
            '--label', f'mobipick.exec={exec_id}',
            '--label', 'mobipick.role=remote-shell',
            '--label', f'mobipick.tab={tab_key}',
            '--user', 'root',
            *window._compose_env_args(env_overrides, container_name=container_name),
            window._ros_tool_service(),
            'python3',
            f'{CONTAINER_SCRIPTS_DIR}/enter_host_shell.py',
            'bash', '--noprofile', '--norc',
        ]
        env = window._prepare_run_env({})['env']
        env.update(env_overrides)
        tab = window._ensure_tab(tab_key, label, closable=True)
        tab.container_name = container_name
        tab.exec_id = exec_id
        window._append_gui_html(
            tab_key,
            f'<i>Remote shell {session_id} starting in container '
            f'{html.escape(container_name)}: {html.escape(" ".join(argv))}</i>',
        )
        window._focus_tab(tab_key)
        window._log_info(f'remote control: opening shell {session_id} ({container_name})')
        return {
            'argv': argv,
            'ensure_docker_network': 'mobipick',
            'env': env,
            'cwd': str(window._project_root),
            'container_name': container_name,
            'tab_key': tab_key,
            'target': container_name,
            'runs_on': 'container',
            'init_command': (
                f'source {CONTAINER_SCRIPTS_DIR}/terminal.bashrc; '
                'echo "remote shell workspace: ${MOBIPICK_WORKSPACE_NAME:-Docker image default}'
                ' (ROS_WORKSPACE=${ROS_WORKSPACE:-unset})"; '
                'echo "remote shell workspace chain: ${MOBIPICK_WORKSPACE_DEVEL_PATHS:-none}"; '
                'echo "remote shell ROS_MASTER_URI=${ROS_MASTER_URI:-unset} ROS_IP=${ROS_IP:-unset}"'
            ),
        }

    def mirror_shell_output(self, session: RemoteShellSession, text: str) -> None:
        window = self.window
        if window._exit_in_progress or not session.tab_key:
            return
        tab = window._ensure_tab(session.tab_key, session.name, closable=True)
        data = window._filter_terminal_escapes(text)
        data = window._collapse_carriage_returns(data)
        window._prepare_tab_for_origin(session.tab_key, 'container')
        tab._enqueue_output_lines(data)

    def shell_session_exited(self, session: RemoteShellSession) -> None:
        window = self.window
        if window._exit_in_progress:
            return
        if session.tab_key and session.tab_key in window.tasks:
            window._append_gui_html(
                session.tab_key,
                f'<i>Remote shell {session.id} closed (exit code {session._exit_code}).</i>',
            )
            tab = window.tasks[session.tab_key]
            tab.container_name = None
            tab.exec_id = None
        server = window.remote_control
        if server is None or not server.sessions():
            window._revoke_x(REMOTE_SHELL_XHOST_SOURCE, log_key='log')

    # -- quit ----------------------------------------------------------

    def quit(self) -> None:
        self.window.close()


__all__ = ['MainWindowRemoteAdapter', 'REMOTE_SHELL_TAB_PREFIX']
