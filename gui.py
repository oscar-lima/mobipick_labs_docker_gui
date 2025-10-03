import sys
import os
import signal
import subprocess
import html
import re
import uuid
import shlex
from pathlib import Path
from datetime import datetime

from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QPushButton, QVBoxLayout, QWidget, QTextEdit,
    QLineEdit, QHBoxLayout, QLabel, QSizePolicy, QFileDialog,
    QComboBox, QTabWidget, QMessageBox, QTabBar, QCheckBox
)
from PyQt5.QtCore import QProcess, QTimer
from PyQt5.QtGui import QTextCursor, QTextDocument
from collections import deque
from typing import Deque

# compiled once, reused by ansi_to_html
SGR_RE = re.compile(r'\x1b\[(\d+(?:;\d+)*)m')
COLOR_MAP = {
    30: '#000000', 31: '#ff5555', 32: '#50fa7b', 33: '#f1fa8c',
    34: '#bd93f9', 35: '#ff79c6', 36: '#8be9fd', 37: '#bbbbbb',
    90: '#666666', 91: '#ff6e6e', 92: '#69ff94', 93: '#ffffa5',
    94: '#d6acff', 95: '#ff92df', 96: '#a4ffff', 97: '#ffffff'
}

# Requires: PyQt5, pyyaml
SCRIPT_CLEAN = './clean.bash'
DEFAULT_YAML_PATH = './config/worlds.yaml'

# limit how much we render per coalesce tick to keep UI responsive on very chatty streams
MAX_BYTES_PER_TICK = 128 * 1024  # 128 KiB per stream per tick


def ansi_to_html(chunk: str) -> str:
    # only called when '\x1b[' is present
    text = html.escape(chunk)
    span_stack, out = [], []
    i = 0
    for m in SGR_RE.finditer(text):
        out.append(text[i:m.start()])
        params = m.group(1)
        if params == '0':
            while span_stack:
                out.append('</span>')
                span_stack.pop()
            i = m.end()
            continue
        styles = []
        for p in params.split(';'):
            try:
                code = int(p)
            except ValueError:
                continue
            if code == 1:
                styles.append('font-weight:bold')
            elif code in COLOR_MAP:
                styles.append(f'color:{COLOR_MAP[code]}')
            elif code == 39:
                styles.append('color:#ffffff')
            elif code == 22:
                styles.append('font-weight:normal')
        if styles:
            out.append(f"<span style=\"{' ;'.join(styles)}\">")
            span_stack.append('</span>')
        i = m.end()
    out.append(text[i:])
    while span_stack:
        out.append(span_stack.pop())
    res = ''.join(out)
    # normalize newlines to <br> once at the end
    return res.replace('\r\n', '\n').replace('\r', '\n').replace('\n', '<br>')


class LogTextEdit(QTextEdit):
    def __init__(self):
        super().__init__()
        self.setAcceptRichText(True)
        self.setReadOnly(True)
        self.setUndoRedoEnabled(False)                     # faster
        self.document().setMaximumBlockCount(20000)        # bound memory
        self.setStyleSheet('QTextEdit { background-color: #000000; color: #ffffff; font-family: monospace; }')

        # batching buffer and timer
        self._buf: Deque[tuple[bool, str]] = deque()       # (is_html, text)
        self._flush_timer = QTimer(self)
        self._flush_timer.setInterval(30)                  # 30 ms flush cadence
        self._flush_timer.timeout.connect(self._flush)

    def enqueue(self, is_html: bool, text: str):
        self._buf.append((is_html, text))
        if not self._flush_timer.isActive():
            self._flush_timer.start()

    def _flush(self):
        if not self._buf:
            self._flush_timer.stop()
            return
        bar = self.verticalScrollBar()
        prev_value = bar.value()
        prev_max = bar.maximum()
        tolerance = max(2, bar.singleStep())
        at_bottom = prev_value >= max(0, prev_max - tolerance)

        self.setUpdatesEnabled(False)
        doc = self.document()
        doc.blockSignals(True)
        cursor = QTextCursor(doc)
        cursor.movePosition(QTextCursor.End)
        try:
            # drain everything queued since last tick
            while self._buf:
                is_html, s = self._buf.popleft()
                if is_html:
                    cursor.insertHtml(s)
                else:
                    cursor.insertText(s)
            if at_bottom:
                bar.setValue(bar.maximum())
            else:
                bar.setValue(min(prev_value, bar.maximum()))
        finally:
            doc.blockSignals(False)
            self.setUpdatesEnabled(True)
            if not self._buf:
                self._flush_timer.stop()


class ProcessTab:
    def __init__(self, key: str, label: str, parent: 'MainWindow', closable: bool):
        self.key = key
        self.label = label
        self.parent = parent
        self.closable = closable

        self.output = LogTextEdit()

        self.proc = QProcess(parent)
        self.proc.setProcessChannelMode(QProcess.MergedChannels)

        # coalesce very chatty streams to reduce signal storms
        self._coalesce_timer = QTimer(parent)
        self._coalesce_timer.setInterval(25)
        self._coalesce_timer.setSingleShot(True)
        self._stdout_buf = bytearray()
        self._stderr_buf = bytearray()
        self.proc.readyReadStandardOutput.connect(self._on_stdout_buf)
        self.proc.readyReadStandardError.connect(self._on_stderr_buf)
        self._coalesce_timer.timeout.connect(self._flush_coalesced)

        self.proc.finished.connect(lambda code, st: parent.on_task_finished(self.key, code, st))

        # per tab docker container name (for custom commands)
        self.container_name: str | None = None

    def start_shell(self, bash_cmd: str):
        self.append_line_html(f'<i>&gt; {html.escape(bash_cmd)}</i>')
        self.parent._log_cmd(bash_cmd)
        self.proc.start('bash', ['-lc', bash_cmd])

    def start_program(self, program: str, args: list[str]):
        cmdline = program + ' ' + ' '.join(args)
        self.append_line_html(f'<i>&gt; {html.escape(cmdline)}</i>')
        self.parent._log_cmd([program] + args)
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

    # send HTML lines through the batcher
    def append_line_html(self, html_text: str):
        self.output.enqueue(True, html_text + '<br>')

    # buffered handlers
    def _on_stdout_buf(self):
        self._stdout_buf += bytes(self.proc.readAllStandardOutput())
        self._coalesce_timer.start()

    def _on_stderr_buf(self):
        self._stderr_buf += bytes(self.proc.readAllStandardError())
        self._coalesce_timer.start()

    def _flush_coalesced(self):
        # process a bounded slice each tick to keep UI responsive
        did_work = False
        if self._stdout_buf:
            chunk = bytes(self._stdout_buf[:MAX_BYTES_PER_TICK])
            del self._stdout_buf[:len(chunk)]
            self._append_raw(chunk)
            did_work = True
        if self._stderr_buf:
            chunk = bytes(self._stderr_buf[:MAX_BYTES_PER_TICK])
            del self._stderr_buf[:len(chunk)]
            self._append_raw(chunk)
            did_work = True

        # if backlog remains, reschedule another tick
        if self._stdout_buf or self._stderr_buf:
            self._coalesce_timer.start()
        elif not did_work:
            # nothing to do
            return

    # fast path for plain text, HTML only if ANSI present
    def _append_raw(self, data_bytes: bytes):
        if not data_bytes:
            return
        data = data_bytes.decode(errors='replace')
        if not data:
            return
        if '\x1b[' in data:
            self.output.enqueue(True, ansi_to_html(data))
        else:
            # preserve newlines as plain text
            self.output.enqueue(False, data)


class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()

        self.setWindowTitle('Mobipick Labs Control')
        self.setGeometry(100, 100, 1100, 780)

        self._killing = False
        self._last_search = ''
        self._yaml_path = None
        self._custom_counter = 0

        # sim state
        self._sim_container_name = 'mobipick-run'
        self._sim_xhost_granted = False
        self._sim_running_cached = False  # event driven sim state

        self.tasks: dict[str, ProcessTab] = {}

        central = QWidget()
        self.setCentralWidget(central)
        root = QVBoxLayout(central)

        # top controls
        top = QHBoxLayout()
        self.sim_toggle_button = QPushButton()
        self.sim_toggle_button.clicked.connect(self._on_sim_toggle_clicked)
        top.addWidget(self.sim_toggle_button)

        # optional manual refresh button for rare external changes
        self.refresh_sim_button = QPushButton('Refresh')
        self.refresh_sim_button.clicked.connect(self._on_refresh_clicked)
        top.addWidget(self.refresh_sim_button)

        self.clear_button = QPushButton('Clear Current Tab')
        self.clear_button.clicked.connect(self.clear_current_tab)
        top.addWidget(self.clear_button)

        self.interrupt_button = QPushButton('Interrupt Current')
        self.interrupt_button.setToolTip('Send ctrl+c to the running process in the current tab')
        self.interrupt_button.clicked.connect(self._on_interrupt_clicked)
        top.addWidget(self.interrupt_button)

        spacer = QWidget()
        spacer.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Preferred)
        top.addWidget(spacer)
        root.addLayout(top)

        # actions row
        actions = QHBoxLayout()

        self.tables_demo_button = QPushButton('Run Tables Demo')
        self.tables_demo_button.clicked.connect(self._on_tables_demo_clicked)
        actions.addWidget(self.tables_demo_button)

        self.rviz_button = QPushButton('Open RViz')
        self.rviz_button.clicked.connect(self._on_rviz_clicked)
        actions.addWidget(self.rviz_button)

        self.rqt_tables_button = QPushButton('Open RQt Tables Demo')
        self.rqt_tables_button.clicked.connect(self._on_rqt_tables_clicked)
        actions.addWidget(self.rqt_tables_button)

        self.world_label = QLabel('world_config:')
        actions.addWidget(self.world_label)

        self.world_combo = QComboBox()
        actions.addWidget(self.world_combo)

        self.browse_yaml_button = QPushButton('Load YAML')
        self.browse_yaml_button.clicked.connect(self._on_load_yaml_clicked)
        actions.addWidget(self.browse_yaml_button)

        root.addLayout(actions)

        # custom command row
        cmdrow = QHBoxLayout()
        self.command_input = QLineEdit()
        self.command_input.setPlaceholderText('Enter custom command, press Enter to run')
        self.command_input.returnPressed.connect(self._on_command_input_return)
        cmdrow.addWidget(self.command_input)

        self.run_command_button = QPushButton('Run Command')
        self.run_command_button.clicked.connect(self._on_run_command_clicked)
        cmdrow.addWidget(self.run_command_button)

        self.reuse_checkbox = QCheckBox('Run in current custom tab')
        self.reuse_checkbox.setChecked(True)
        cmdrow.addWidget(self.reuse_checkbox)
        root.addLayout(cmdrow)

        # search row
        search = QHBoxLayout()
        search.addWidget(QLabel('Search:'))
        self.search_input = QLineEdit()
        self.search_input.setPlaceholderText('Find text in current tab')
        self.search_input.returnPressed.connect(self.find_next)
        search.addWidget(self.search_input)
        self.find_prev_button = QPushButton('Prev')
        self.find_prev_button.clicked.connect(self.find_prev)
        search.addWidget(self.find_prev_button)
        self.find_next_button = QPushButton('Next')
        self.find_next_button.clicked.connect(self.find_next)
        search.addWidget(self.find_next_button)
        root.addLayout(search)

        # tabs
        self.tabs = QTabWidget()
        self.tabs.setTabsClosable(False)
        self.tabs.tabCloseRequested.connect(self.on_tab_close_requested)
        root.addWidget(self.tabs)

        # fixed tabs
        self._ensure_tab('sim', 'Sim', closable=False)
        self._ensure_tab('tables', 'Tables Demo', closable=False)
        self._ensure_tab('rviz', 'RViz', closable=False)
        self._ensure_tab('rqt', 'RQt Tables', closable=False)
        # central log tab
        self._ensure_tab('log', 'Log', closable=False)

        # polling
        self.poll_timer = QTimer(self)
        self.poll_timer.timeout.connect(self._poll)
        self.poll_timer.start(1200)

        try:
            signal.signal(signal.SIGINT, signal.SIG_IGN)
        except Exception:
            pass

        self.load_yaml(DEFAULT_YAML_PATH)
        self._update_buttons()
        self.update_sim_status_from_poll()

    # ---------- Log tab helpers ----------

    def _fmt_args(self, args_or_str) -> str:
        if isinstance(args_or_str, str):
            return args_or_str
        return ' '.join(shlex.quote(s) for s in args_or_str)

    def _append_log_html(self, html_text: str):
        if 'log' not in self.tasks:
            return
        self.tasks['log'].append_line_html(html_text)

    def _log_cmd(self, args_or_str):
        ts = datetime.now().strftime('%H:%M:%S')
        line = f'[{ts}] $ {self._fmt_args(args_or_str)}'
        self._append_log_html(html.escape(line))

    def _log_event(self, details: str):
        ts = datetime.now().strftime('%H:%M:%S')
        line = f'[{ts}] event: {details}'
        self._append_log_html(html.escape(line))

    def _log_button_click(self, button: QPushButton, fallback: str | None = None):
        label = button.text().strip()
        if not label:
            label = fallback or 'button'
        self._log_event(f'user clicked {label}')

    def _on_sim_toggle_clicked(self):
        self._log_button_click(self.sim_toggle_button, 'Sim Toggle')
        self.toggle_sim()

    def _on_refresh_clicked(self):
        self._log_button_click(self.refresh_sim_button)
        self.update_sim_status_from_poll(force=True)

    def _on_interrupt_clicked(self):
        self._log_button_click(self.interrupt_button)
        self.interrupt_current_tab()

    def _on_tables_demo_clicked(self):
        self._log_button_click(self.tables_demo_button)
        self.run_tables_demo()

    def _on_rviz_clicked(self):
        self._log_button_click(self.rviz_button)
        self.open_rviz()

    def _on_rqt_tables_clicked(self):
        self._log_button_click(self.rqt_tables_button)
        self.open_rqt_tables_demo()

    def _on_load_yaml_clicked(self):
        self._log_button_click(self.browse_yaml_button)
        self.load_yaml_dialog()

    def _on_run_command_clicked(self):
        self._log_button_click(self.run_command_button)
        self.run_custom_command()

    def _on_command_input_return(self):
        self._log_event('user pressed enter to run command')
        self.run_custom_command()

    def _sp_run(self, args: list[str], **kwargs):
        # wrapper around subprocess.run with logging into the Log tab
        self._log_cmd(args)
        return subprocess.run(args, **kwargs)

    # ---------- Tabs and process management ----------

    def _ensure_tab(self, key: str, label: str, closable: bool) -> ProcessTab:
        if key in self.tasks:
            return self.tasks[key]
        tab = ProcessTab(key, label, self, closable)
        idx = self.tabs.addTab(tab.output, label)
        self._apply_close_button(idx, closable)
        if key == 'sim':
            self.tabs.setCurrentIndex(idx)
        self.tasks[key] = tab
        return tab

    def _apply_close_button(self, index: int, closable: bool):
        bar = self.tabs.tabBar()
        bar.setTabButton(index, QTabBar.RightSide, None)
        if closable:
            btn = QPushButton('✕', self)
            btn.setToolTip('Close tab')
            btn.setFixedSize(18, 18)
            btn.setStyleSheet('QPushButton { border: none; padding: 0px; }')
            btn.clicked.connect(self._on_close_button_clicked)
            bar.setTabButton(index, QTabBar.RightSide, btn)

    def _on_close_button_clicked(self):
        sender = self.sender()
        bar = self.tabs.tabBar()
        for i in range(self.tabs.count()):
            if bar.tabButton(i, QTabBar.RightSide) is sender:
                self.on_tab_close_requested(i)
                break

    def _new_custom_tab_key(self, always_new: bool = False) -> str:
        if not always_new:
            for k, t in self.tasks.items():
                if k.startswith('custom') and not t.is_running():
                    self._ensure_close_for_key(k)
                    return k
        self._custom_counter += 1
        key = f'custom{self._custom_counter}'
        label = f'Custom {self._custom_counter}'
        self._ensure_tab(key, label, closable=True)
        return key

    def _ensure_close_for_key(self, key: str):
        w = self.tasks[key].output
        for i in range(self.tabs.count()):
            if self.tabs.widget(i) is w:
                self._apply_close_button(i, True)
                break

    def _current_tab_key(self) -> str | None:
        w = self.tabs.currentWidget()
        for k, t in self.tasks.items():
            if t.output is w:
                return k
        return None

    def on_tab_close_requested(self, index: int):
        widget = self.tabs.widget(index)
        tab_text = self.tabs.tabText(index)
        if tab_text:
            self._log_event(f'user clicked close for {tab_text}')
        key = None
        for k, t in self.tasks.items():
            if t.output is widget:
                key = k
                break
        if key is None:
            return
        tab = self.tasks[key]
        if not key.startswith('custom'):
            QMessageBox.information(self, 'Info', 'Only custom tabs can be closed.')
            return
        if tab.is_running():
            try:
                tab.kill()
            except Exception:
                pass
        self.tabs.removeTab(index)
        del self.tasks[key]

    def _focus_tab(self, key: str):
        w = self.tasks[key].output
        for i in range(self.tabs.count()):
            if self.tabs.widget(i) is w:
                self.tabs.setCurrentIndex(i)
                break

    # ---------- YAML ----------

    def load_yaml_dialog(self):
        path, _ = QFileDialog.getOpenFileName(self, 'Select YAML', '', 'YAML files (*.yaml *.yml);;All files (*)')
        if path:
            self.load_yaml(path)

    def load_yaml(self, path: str):
        self.world_combo.clear()
        values = []
        try:
            import yaml  # type: ignore
            if Path(path).is_file():
                with open(path, 'r', encoding='utf-8') as f:
                    data = yaml.safe_load(f)
                if isinstance(data, dict) and 'world_configs' in data and isinstance(data['world_configs'], list):
                    values = [str(x) for x in data['world_configs']]
                elif isinstance(data, list):
                    values = [str(x) for x in data]
                else:
                    self._append_html('sim', f'<i>YAML format not recognized in {html.escape(path)}</i>')
                self._yaml_path = path
            else:
                self._append_html('sim', f'<i>YAML not found: {html.escape(path)}</i>')
        except ImportError:
            self._append_html('sim', '<i>PyYAML not installed, using default option set.</i>')
        except Exception as e:
            self._append_html('sim', f'<i>Failed to load YAML: {html.escape(str(e))}</i>')
        if not values:
            values = ['moelk_tables']
        self.world_combo.addItems(values)

    # ---------- Sim control ----------

    def _grant_x(self):
        if not self._sim_xhost_granted:
            self._sp_run(['xhost', '+local:root'], check=False)
            self._sim_xhost_granted = True

    def _revoke_x(self):
        if self._sim_xhost_granted:
            self._sp_run(['xhost', '-local:root'], check=False)
            self._sim_xhost_granted = False

    def is_sim_running(self) -> bool:
        try:
            cp = self._sp_run(['docker', 'ps', '--format', '{{.Names}}'],
                              stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, check=False, text=True)
            names = set(cp.stdout.strip().splitlines())
            return (self._sim_container_name in names) or self.tasks['sim'].is_running()
        except Exception:
            return False

    def toggle_sim(self):
        if self._killing:
            return
        if self.is_sim_running():
            self.shutdown_sim()
        else:
            self.bring_up_sim()

    # event driven bring up
    def bring_up_sim(self):
        self._sp_run(['docker', 'network', 'create', 'mobipick'], check=False,
                    stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        self._grant_x()

        tab = self._ensure_tab('sim', 'Sim', closable=False)
        tab.container_name = self._sim_container_name  # ensure sim tab is addressable

        args = ['compose', 'run', '--rm', '--name', self._sim_container_name, 'mobipick']
        tab.start_program('docker', args)

        # event driven state
        self._sim_running_cached = True
        self._killing = False
        self.set_toggle_visual('green', 'Stop Sim', enabled=True)

    def _graceful_stop_container(self, name: str, tab: ProcessTab | None = None):
        try:
            # act only if the exact-named container exists
            cp = self._sp_run(['docker', 'ps', '-q', '--filter', f'name=^{name}$'],
                            stdout=subprocess.PIPE, stderr=subprocess.DEVNULL,
                            check=False, text=True)
            cid = cp.stdout.strip()
            if not cid:
                if tab:
                    tab.append_line_html(f'<i>No running container named {html.escape(name)}</i>')
                return
            # send INT first for a graceful stop of GUI apps, then stop
            self._sp_run(['docker', 'kill', '-s', 'INT', name], check=False,
                        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            if tab:
                tab.append_line_html(f'<i>docker kill -s INT {html.escape(name)}</i>')
            self._sp_run(['docker', 'stop', name], check=False,
                        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            if tab:
                tab.append_line_html(f'<i>docker stop {html.escape(name)}</i>')
        except Exception as e:
            if tab:
                tab.append_line_html(f'<i>Targeted stop error: {html.escape(str(e))}</i>')

    # event driven shutdown
    def shutdown_sim(self):
        self.set_toggle_visual('yellow', 'Wait shutdown', enabled=False)
        self._killing = True

        tab = self._ensure_tab('sim', 'Sim', closable=False)

        pid = tab.pid()
        if pid:
            try:
                os.kill(pid, signal.SIGINT)
                tab.append_line_html('<i>Sent SIGINT to docker compose (graceful stop)...</i>')
                self._log_cmd(f'kill -SIGINT {pid}')
            except Exception as e:
                tab.append_line_html(f'<i>Failed to send SIGINT: {html.escape(str(e))}</i>')

        def _fallbacks():
            # stop sim container if present
            self._docker_stop_if_exists(self._sim_container_name, tab)
            # stop everything related, including rqt/rviz/mobipick_cmd one offs
            self._stop_all_related(tab)

            if Path(SCRIPT_CLEAN).is_file() and os.access(SCRIPT_CLEAN, os.X_OK):
                tab.append_line_html('<i>Invoking clean.bash for final cleanup...</i>')
                try:
                    self._sp_run([SCRIPT_CLEAN], check=False)
                except Exception as e:
                    tab.append_line_html(f'<i>clean.bash failed: {html.escape(str(e))}</i>')
            else:
                tab.append_line_html('<i>clean.bash not found or not executable.</i>')

            self._revoke_x()
            self._sim_running_cached = False
            self._killing = False
            self.set_toggle_visual('red', 'Start Sim', enabled=True)

        QTimer.singleShot(2500, _fallbacks)

    def _docker_stop_if_exists(self, name: str, tab: ProcessTab | None = None):
        try:
            cp = self._sp_run(['docker', 'ps', '-q', '--filter', f'name=^{name}$'],
                              stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, check=False, text=True)
            cid = cp.stdout.strip()
            if cid:
                self._sp_run(['docker', 'stop', name], check=False,
                             stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
                if tab:
                    tab.append_line_html(f'<i>docker stop {html.escape(name)}</i>')
        except Exception as e:
            if tab:
                tab.append_line_html(f'<i>docker stop error: {html.escape(str(e))}</i>')

    # Stop all related containers, robust name/image/label pattern matching.
    # Sends INT first for a graceful shutdown of GUIs, then docker stop.
    def _stop_all_related(self, tab: ProcessTab):
        patterns = [
            'mobipick', 'mobipick_cmd', 'mobipick-run',
            'brean/mobipick_labs', 'rqt', 'rviz'
        ]
        try:
            cp = self._sp_run(
                ['docker', 'ps', '-a', '--format', '{{.ID}} {{.Names}} {{.Image}} {{.Labels}}'],
                stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, check=False, text=True
            )
            ids: list[str] = []
            for ln in cp.stdout.splitlines():
                ln = ln.strip()
                if not ln:
                    continue
                parts = ln.split(' ', 3)  # id, name, image, labels
                if len(parts) < 3:
                    continue
                cid, cname, cimage = parts[0], parts[1], parts[2]
                clabels = parts[3] if len(parts) == 4 else ''
                hay = f'{cname} {cimage} {clabels}'
                if any(p in hay for p in patterns):
                    ids.append(cid)

            ids = list(dict.fromkeys(ids))  # unique, preserve order

            if not ids:
                tab.append_line_html('<i>No related containers found.</i>')
                return

            tab.append_line_html(f'<i>Sending INT to related containers: {html.escape(" ".join(ids))}</i>')
            self._sp_run(['docker', 'kill', '-s', 'INT'] + ids, check=False,
                         stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

            tab.append_line_html(f'<i>Stopping related containers: {html.escape(" ".join(ids))}</i>')
            self._sp_run(['docker', 'stop'] + ids, check=False,
                         stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        except Exception as e:
            tab.append_line_html(f'<i>Error while stopping related containers: {html.escape(str(e))}</i>')

    # optionally keep a manual refresh helper for rare external changes
    def update_sim_status_from_poll(self, force=False):
        if self._killing:
            self.set_toggle_visual('yellow', 'Wait shutdown', enabled=False)
            return
        if not force:
            self.set_toggle_visual('green', 'Stop Sim', True) if self._sim_running_cached \
                else self.set_toggle_visual('red', 'Start Sim', True)
            return
        try:
            cp = self._sp_run(['docker', 'ps', '--format', '{{.Names}}'],
                              stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, check=False, text=True)
            names = set(cp.stdout.strip().splitlines())
            self._sim_running_cached = (self._sim_container_name in names) or self.tasks['sim'].is_running()
        except Exception:
            pass
        self.set_toggle_visual('green', 'Stop Sim', True) if self._sim_running_cached \
            else self.set_toggle_visual('red', 'Start Sim', True)

    def set_toggle_visual(self, state: str, text: str, enabled: bool):
        self.sim_toggle_button.setText(text)
        if state == 'green':
            bg = '#28a745'; fg = 'white'
        elif state == 'red':
            bg = '#dc3545'; fg = 'white'
        elif state == 'yellow':
            bg = '#ffc107'; fg = 'black'
        else:
            bg = '#6c757d'; fg = 'white'
        self.sim_toggle_button.setStyleSheet(
            f'QPushButton {{ background-color: {bg}; color: {fg}; border: none; padding: 6px; }}'
            f'QPushButton:disabled {{ opacity: 0.85; }}'
        )
        self.sim_toggle_button.setEnabled(enabled)

    # ---------- Buffering control helper ----------
    def _wrap_line_buffered(self, inner: str) -> str:
        # ensure unbuffered python, utf8, and force line buffered stdout and stderr when available
        # no TTY required
        return (
            'export PYTHONUNBUFFERED=1 PYTHONIOENCODING=UTF-8; '
            'if command -v stdbuf >/dev/null 2>&1; then '
            f'stdbuf -oL -eL {inner}; '
            'else '
            f'{inner}; '
            'fi'
        )

    # ---------- Actions ----------

    def run_tables_demo(self):
        tab = self._ensure_tab('tables', 'Tables Demo', closable=False)
        tab.container_name = f'mpcmd-{uuid.uuid4().hex[:10]}'
        inner = 'rosrun tables_demo_planning tables_demo_node.py'
        args = [
            'compose', 'run', '--rm', '--name', tab.container_name,
            'mobipick_cmd', 'bash', '-lc', self._wrap_line_buffered(inner)
        ]
        tab.start_program('docker', args)
        self._focus_tab('tables')

    def open_rviz(self):
        tab = self._ensure_tab('rviz', 'RViz', closable=False)
        tab.container_name = f'mpcmd-{uuid.uuid4().hex[:10]}'
        rviz_cmd = 'rosrun rviz rviz -d $(rospack find tables_demo_bringup)/config/pick_n_place.rviz __ns:=mobipick'
        args = [
            'compose', 'run', '--rm', '--name', tab.container_name,
            'mobipick_cmd', 'bash', '-lc', self._wrap_line_buffered(rviz_cmd)
        ]
        tab.start_program('docker', args)
        self._focus_tab('rviz')

    def open_rqt_tables_demo(self):
        world = self.world_combo.currentText().strip() or 'moelk_tables'
        tab = self._ensure_tab('rqt', 'RQt Tables', closable=False)
        tab.container_name = f'mpcmd-{uuid.uuid4().hex[:10]}'
        cmd = f'roslaunch rqt_tables_demo rqt_tables_demo.launch namespace:=mobipick world_config:={self._sh_quote(world)}'
        args = [
            'compose', 'run', '--rm', '--name', tab.container_name,
            'mobipick_cmd', 'bash', '-lc', self._wrap_line_buffered(cmd)
        ]
        tab.start_program('docker', args)
        self._focus_tab('rqt')

    def run_custom_command(self):
        text = self.command_input.text().strip()
        if not text:
            return

        if self.reuse_checkbox.isChecked():
            # reuse current idle custom tab or any idle one
            key_target = None
            cur_key = self._current_tab_key()
            if cur_key and cur_key.startswith('custom') and not self.tasks[cur_key].is_running():
                key_target = cur_key
            else:
                for k, t in self.tasks.items():
                    if k.startswith('custom') and not t.is_running():
                        key_target = k
                        break
            if key_target is None:
                key_target = self._new_custom_tab_key(always_new=True)
        else:
            # always open a brand new tab when unchecked
            key_target = self._new_custom_tab_key(always_new=True)

        tab = self.tasks[key_target]
        tab.container_name = f'mpcmd-{uuid.uuid4().hex[:10]}'
        wrapped = self._wrap_line_buffered(text)
        args = [
            'compose', 'run', '--rm', '--name', tab.container_name,
            'mobipick_cmd', 'bash', '-lc', wrapped
        ]
        tab.start_program('docker', args)
        self._focus_tab(key_target)

    # ---------- Interrupt current tab (ctrl+c / SIGINT) ----------

    def interrupt_current_tab(self):
        key = self._current_tab_key()
        if not key or not key.startswith('custom'):  # only custom tabs are interruptible
            return
        tab = self.tasks[key]
        if not tab.is_running():
            return

        # 1) send SIGINT to the client process in this tab only
        pid = tab.pid()
        if pid:
            try:
                os.kill(pid, signal.SIGINT)
                tab.append_line_html('<i>Sent SIGINT to client (ctrl+c)...</i>')
                self._log_cmd(f'kill -SIGINT {pid}')
            except Exception as e:
                tab.append_line_html(f'<i>Failed to SIGINT client: {html.escape(str(e))}</i>')

        # 2) if this tab spawned a named docker container, stop only that container
        def _container_sigint_then_stop():
            if tab.container_name:
                self._graceful_stop_container(tab.container_name, tab)

        QTimer.singleShot(1000, _container_sigint_then_stop)

    # ---------- Output and search ----------

    def clear_current_tab(self):
        widget = self.tabs.currentWidget()
        if isinstance(widget, QTextEdit):
            widget.clear()

    def _append_html(self, key: str, html_text: str):
        self._ensure_tab(key, key.title(), closable=(key.startswith('custom'))).append_line_html(html_text)

    # ---------- Utils ----------

    @staticmethod
    def _sh_quote(s: str) -> str:
        return "'" + s.replace("'", "'\\''") + "'"

    # ---------- Search within current tab ----------

    def _current_text_edit(self) -> QTextEdit | None:
        w = self.tabs.currentWidget()
        return w if isinstance(w, QTextEdit) else None

    def _do_find(self, edit: QTextEdit, pattern: str, flags=QTextDocument.FindFlags()):
        if not pattern:
            return False
        if pattern != self._last_search:
            cursor = edit.textCursor()
            cursor.movePosition(QTextCursor.Start)
            edit.setTextCursor(cursor)
            self._last_search = pattern
        found = edit.find(pattern, flags)
        if not found:
            cursor = edit.textCursor()
            if flags & QTextDocument.FindBackward:
                cursor.movePosition(QTextCursor.End)
            else:
                cursor.movePosition(QTextCursor.Start)
            edit.setTextCursor(cursor)
            found = edit.find(pattern, flags)
        return found

    def find_next(self):
        edit = self._current_text_edit()
        if not edit:
            return
        pattern = self.search_input.text()
        if not self._do_find(edit, pattern) and pattern:
            edit.insertHtml(f"<i>No match for '{html.escape(pattern)}'</i><br>")

    def find_prev(self):
        edit = self._current_text_edit()
        if not edit:
            return
        pattern = self.search_input.text()
        if not self._do_find(edit, pattern, QTextDocument.FindBackward) and pattern:
            edit.insertHtml(f"<i>No match for '{html.escape(pattern)}'</i><br>")

    # ---------- Process completion callback ----------

    def on_task_finished(self, key: str, exit_code: int, exit_status):
        status_name = 'NormalExit' if int(exit_status) == int(QProcess.NormalExit) else 'Crashed'
        if key in self.tasks:
            self._append_html(key, f'<i>Process finished with code {exit_code} [{status_name}]</i>')
            self.tasks[key].container_name = None
        if key == 'sim':
            self._revoke_x()
            self._killing = False
            # sim process ended, reflect stopped state immediately
            self._sim_running_cached = False
            self.set_toggle_visual('red', 'Start Sim', enabled=True)

    # ---------- Poll and initial states ----------

    # manage only Interrupt button here
    def _poll(self):
        key = self._current_tab_key()
        running_custom = bool(key and key.startswith('custom') and self.tasks[key].is_running())
        self.interrupt_button.setEnabled(running_custom)

    def _update_buttons(self):
        # initial deterministic state to avoid AttributeError
        self.interrupt_button.setEnabled(False)

    # ---------- Close ----------

    def closeEvent(self, event):
        for p in list(self.tasks.values()):
            if p.is_running():
                try:
                    p.kill()
                except Exception:
                    pass
        self._revoke_x()
        event.accept()


if __name__ == '__main__':
    app = QApplication(sys.argv)
    w = MainWindow()
    w.show()
    sys.exit(app.exec_())
