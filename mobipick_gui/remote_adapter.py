"""Bridge between :class:`RemoteControlServer` and :class:`MainWindow`.

Every method here runs on the Qt GUI thread; the server marshals calls
through :class:`mobipick_gui.remote_control.GuiInvoker`.
"""
from __future__ import annotations

import html
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

    # -- status --------------------------------------------------------

    def status(self) -> dict:
        window = self.window
        registry = getattr(window, '_workspace_registry', None)
        return {
            'version': __version__,
            'workspace': getattr(registry, 'active', None) if registry else None,
            'image': getattr(window, '_selected_image', None),
            'world': window._current_world() if hasattr(window, '_current_world') else None,
            'remote_master': bool(window._remote_master_enabled()),
            'master_uri': window._current_master_uri(),
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

    # -- buttons -------------------------------------------------------

    def _button_widget(self, key: str) -> QPushButton | None:
        window = self.window
        widget = window._button_widgets.get(key)
        if widget is None:
            widget = getattr(window, f'{key}_button', None)
        return widget if isinstance(widget, QPushButton) else None

    def _button_keys(self) -> list[str]:
        window = self.window
        keys: list[str] = []
        for key in ('roscore', *window._config_button_order, 'terminal', 'auto_launch'):
            if key not in keys:
                keys.append(key)
        for key in window._button_widgets:
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
        }

    def buttons(self) -> list[dict]:
        return [self._describe_button(key) for key in self._button_keys()]

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
                elif kind == 'shell':
                    server = window.remote_control
                    if server is not None and any(s.id == key for s in server.sessions()):
                        server.close_session(key)
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

    def shell_spec(self, session_id: int, label: str, *, root: bool | None) -> dict:
        from .main_window import CONTAINER_SCRIPTS_DIR

        window = self.window
        if window._exit_in_progress:
            raise Conflict('the GUI is shutting down')
        window._ensure_network(log_key='log')
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
            'env': env,
            'cwd': str(window._project_root),
            'container_name': container_name,
            'tab_key': tab_key,
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
