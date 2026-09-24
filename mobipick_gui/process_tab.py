"""Process tab widget wrapper."""
from __future__ import annotations

import codecs
import html
import re
from typing import TYPE_CHECKING

from PyQt5.QtCore import QProcess

from .ansi import CSI_SEQ_RE, ansi_to_html
from .log_widget import LogTextEdit

if TYPE_CHECKING:  # pragma: no cover
    from .main_window import MainWindow


ROS_WARNING_RE = re.compile(r'^\[\s*WARN(?:ING)?\s*\]')
ROS_WARNING_COLOR = '#f1fa8c'


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
        self._reset_output_stream()
        self._shutting_down = False

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
        self.run_generation = 0

    def start_shell(self, bash_cmd: str):
        self.run_generation += 1
        self._reset_output_stream()
        self._append_command_line(bash_cmd)
        self.parent._log_cmd(bash_cmd)
        self._apply_env()
        self.proc.start('bash', ['-lc', bash_cmd])

    def start_program(self, program: str, args: list[str]):
        self.run_generation += 1
        self._reset_output_stream()
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

    def stop_for_shutdown(self, timeout_ms: int = 1000) -> bool:
        """Request process stop and disable GUI callbacks without waiting."""
        self._shutting_down = True
        process_signals = (
            self.proc.readyReadStandardOutput,
            self.proc.readyReadStandardError,
            self.proc.finished,
            self.proc.errorOccurred,
        )
        for process_signal in process_signals:
            try:
                process_signal.disconnect()
            except (RuntimeError, TypeError):
                pass
        try:
            if self.proc.state() == QProcess.NotRunning:
                return True
            self.proc.kill()
            return True
        except RuntimeError:
            return True

    def is_running(self) -> bool:
        return self.proc.state() != QProcess.NotRunning

    def append_line_html(self, html_text: str):
        self.output.enqueue(True, html_text + '<br>')

    def _on_stdout_buf(self):
        if self._shutting_down:
            return
        data = bytes(self.proc.readAllStandardOutput())
        if data:
            self._append_raw(data)

    def _on_stderr_buf(self):
        if self._shutting_down:
            return
        data = bytes(self.proc.readAllStandardError())
        if data:
            self._append_raw(data)

    def _drain_remaining(self, *_):
        if self._shutting_down:
            return
        data_out = bytes(self.proc.readAllStandardOutput())
        if data_out:
            self._append_raw(data_out)
        data_err = bytes(self.proc.readAllStandardError())
        if data_err:
            self._append_raw(data_err)
        self._output_pending += self._output_decoder.decode(b'', final=True)
        self._flush_output_pending(final=True)

    def _append_raw(self, data_bytes: bytes):
        if not data_bytes:
            return
        data = self._output_decoder.decode(data_bytes)
        if not data:
            return
        self._output_pending += data
        self._flush_output_pending()

    def _reset_output_stream(self) -> None:
        """Start fresh decoding state when a reusable tab starts a process."""
        self._output_decoder = codecs.getincrementaldecoder('utf-8')(
            errors='replace'
        )
        self._output_pending = ''

    def _flush_output_pending(self, *, final: bool = False) -> None:
        """Render complete lines while retaining an unfinished stream line."""
        if final:
            data = self._output_pending
            self._output_pending = ''
        else:
            line_end = self._output_pending.rfind('\n')
            if line_end < 0:
                return
            data = self._output_pending[:line_end + 1]
            self._output_pending = self._output_pending[line_end + 1:]
        if not data:
            return
        data = self.parent._filter_terminal_escapes(data)
        data = self.parent._collapse_carriage_returns(data)
        if self.notify_parent_finished:
            self.parent._prepare_tab_for_origin(self.key, 'container')
        self._enqueue_output_lines(data)

    def _enqueue_output_lines(self, data: str) -> None:
        """Preserve ANSI colors and highlight uncolored ROS warnings."""
        for line in data.splitlines(keepends=True):
            plain_line = CSI_SEQ_RE.sub('', line)
            if ROS_WARNING_RE.match(plain_line):
                content = plain_line.rstrip('\r\n')
                rendered = (
                    f'<span style="color:{ROS_WARNING_COLOR}">'
                    f'{html.escape(content)}</span>'
                )
                if line.endswith('\n'):
                    rendered += '<br>'
                self.output.enqueue(True, rendered)
            elif '\x1b[' in line:
                self.output.enqueue(True, ansi_to_html(line))
            else:
                self.output.enqueue(False, line)

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
