"""Process tab widget wrapper."""
from __future__ import annotations

import html
from typing import TYPE_CHECKING

from PyQt5.QtCore import QProcess

from .ansi import ansi_to_html
from .log_widget import LogTextEdit

if TYPE_CHECKING:  # pragma: no cover
    from .main_window import MainWindow


class ProcessTab:
    """Wraps a QProcess and associated log widget for a tab."""

    def __init__(
        self,
        key: str,
        label: str,
        parent: 'MainWindow',
        closable: bool,
        *,
        output: LogTextEdit | None = None,
        notify_parent_finished: bool = True,
    ):
        self.key = key
        self.label = label
        self.parent = parent
        self.closable = closable
        self.notify_parent_finished = notify_parent_finished

        self.output = output or LogTextEdit()

        self.environment_overrides: dict[str, str] = {}
        self.proc = QProcess(parent)
        self.proc.setProcessChannelMode(QProcess.MergedChannels)
        self._apply_env()

        self.proc.readyReadStandardOutput.connect(self._on_stdout_buf)
        self.proc.readyReadStandardError.connect(self._on_stderr_buf)
        self.proc.finished.connect(self._drain_remaining)

        if notify_parent_finished:
            self.proc.finished.connect(
                lambda code, st: parent.on_task_finished(self.key, code, st)
            )

        self.container_name: str | None = None
        self.exec_id: str | None = None
        self.xhost_token: str | None = None

    def start_shell(self, bash_cmd: str):
        self._append_command_line(bash_cmd)
        self.parent._log_cmd(bash_cmd)
        self._apply_env()
        self.proc.start('bash', ['-lc', bash_cmd])

    def start_program(self, program: str, args: list[str]):
        cmdline = program + ' ' + ' '.join(args)
        self._append_command_line(cmdline)
        self.parent._log_cmd([program] + args)
        self._apply_env()
        self.proc.start(program, args)

    def pid(self) -> int | None:
        p = self.proc.processId()
        return int(p) if p and p > 0 else None

    def kill(self):
        try:
            self.proc.kill()
        except Exception:
            pass

    def is_running(self) -> bool:
        return self.proc.state() != QProcess.NotRunning

    def append_line_html(self, html_text: str):
        self.output.enqueue(True, html_text + '<br>')

    def _on_stdout_buf(self):
        data = bytes(self.proc.readAllStandardOutput())
        if data:
            self._append_raw(data)

    def _on_stderr_buf(self):
        data = bytes(self.proc.readAllStandardError())
        if data:
            self._append_raw(data)

    def _drain_remaining(self, *_):
        data_out = bytes(self.proc.readAllStandardOutput())
        if data_out:
            self._append_raw(data_out)
        data_err = bytes(self.proc.readAllStandardError())
        if data_err:
            self._append_raw(data_err)

    def _append_raw(self, data_bytes: bytes):
        if not data_bytes:
            return
        data = data_bytes.decode(errors='replace')
        if not data:
            return
        data = self.parent._filter_terminal_escapes(data)
        data = self.parent._collapse_carriage_returns(data)
        if self.notify_parent_finished:
            self.parent._prepare_tab_for_origin(self.key, 'container')
        if '\x1b[' in data:
            self.output.enqueue(True, ansi_to_html(data))
        else:
            self.output.enqueue(False, data)

    def _append_command_line(self, command: str) -> None:
        line = f'<i>&gt; {html.escape(command)}</i>'
        if self.notify_parent_finished:
            self.parent._append_gui_html(
                self.key,
                line,
                color=self.parent._command_log_color,
            )
            return
        color = html.escape(self.parent._command_log_color)
        self.append_line_html(f'<span style="color:{color}">{line}</span>')

    def _apply_env(self):
        env = self.parent._build_process_environment(self.environment_overrides)
        self.proc.setProcessEnvironment(env)

    def refresh_environment(self):
        if not self.is_running():
            self._apply_env()

    def set_environment_overrides(self, values: dict[str, str] | None):
        """Set environment values applied the next time this tab starts."""
        self.environment_overrides = dict(values or {})
        if not self.is_running():
            self._apply_env()


__all__ = ['ProcessTab']
