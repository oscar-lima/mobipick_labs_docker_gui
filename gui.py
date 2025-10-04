import sys
import os
import signal
import subprocess
import html
import re
import uuid
import shlex
import argparse
import copy
import time
import yaml
from pathlib import Path
from datetime import datetime

from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QPushButton, QVBoxLayout, QWidget, QTextEdit,
    QLineEdit, QHBoxLayout, QLabel, QSizePolicy, QFileDialog,
    QComboBox, QTabWidget, QMessageBox, QTabBar, QCheckBox
)
from PyQt5.QtCore import QProcess, QTimer, QProcessEnvironment, Qt
from PyQt5.QtGui import QTextCursor, QTextDocument
from collections import deque
from typing import Deque, Callable, Optional

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
CONFIG_FILE = Path(__file__).resolve().parent / 'config' / 'gui_settings.yaml'

CONFIG_DEFAULTS = {
    'log': {
        'max_block_count': 20000,
        'flush_interval_ms': 30,
        'background_color': '#000000',
        'text_color': '#ffffff',
        'gui_log_color': '#ff00ff',
        'command_log_color': '#4da3ff',
        'font_family': 'monospace',
        'scroll_tolerance_min': 2,
    },
    'window': {
        'geometry': [100, 100, 1100, 780],
        'title': 'Mobipick Labs Control',
    },
    'timers': {
        'poll_ms': 1200,
        'sigint_check_ms': 100,
        'custom_tab_sigint_delay_ms': 1000,
        'sim_shutdown_delay_ms': 2500,
        'roscore_start_delay_ms': 1000,
    },
    'buttons': {
        'sim_toggle': {
            'padding_px': 6,
            'disabled_opacity': 0.85,
            'states': {
                'green': {'bg': '#28a745', 'fg': 'white'},
                'red': {'bg': '#dc3545', 'fg': 'white'},
                'yellow': {'bg': '#ffc107', 'fg': 'black'},
                'grey': {'bg': '#6c757d', 'fg': 'white'},
            },
        },
        'close': {
            'text': '✕',
            'tooltip': 'Close tab',
            'size': 18,
            'stylesheet': 'QPushButton { border: none; padding: 0px; }',
        },
    },
    'process': {
        'qprocess_env': {
            'COMPOSE_IGNORE_ORPHANS': '1',
        },
        'compose_run_env': {
            'PYTHONUNBUFFERED': '1',
            'PYTHONIOENCODING': 'UTF-8',
        },
    },
    'exit': {
        'dialog_title': 'Shutting Down',
        'dialog_message': 'Shutting down simulation and cleaning up. Please wait...',
        'log_start_message': 'Shutting down containers before exit...',
        'log_done_message': 'Shutdown complete. Exiting...',
    },
    'images': {
        'default': 'brean/mobipick_labs:noetic',
        'discovery_filters': ['mobipick'],
        'include_none_tag': False,
        'related_container_keywords': ['mobipick', 'mobipick_cmd', 'mobipick-run', 'rqt', 'rviz'],
        'related_image_keywords': ['mobipick_labs'],
    },
    'worlds': {
        'default': 'moelk_tables',
    },
    'terminal': {
        'launcher': 'gnome-terminal --title "{title}" -- bash -lc "{command}"',
        'title': 'Mobipick Terminal',
        'container_prefix': 'mobipick-terminal',
    },
}


def _deep_update(base: dict, updates: dict) -> dict:
    for key, value in updates.items():
        if isinstance(value, dict) and isinstance(base.get(key), dict):
            base[key] = _deep_update(base[key], value)
        else:
            base[key] = value
    return base


def _load_config() -> dict:
    config = copy.deepcopy(CONFIG_DEFAULTS)
    try:
        if CONFIG_FILE.is_file():
            with open(CONFIG_FILE, 'r', encoding='utf-8') as f:
                data = yaml.safe_load(f) or {}
            if isinstance(data, dict):
                _deep_update(config, data)
    except Exception as exc:
        print(f'Warning: failed to load configuration from {CONFIG_FILE}: {exc}', file=sys.stderr)
    return config


CONFIG = _load_config()

_SIGINT_TRIGGERED = False


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
        log_cfg = CONFIG['log']
        self.document().setMaximumBlockCount(log_cfg['max_block_count'])        # bound memory
        self.setStyleSheet(
            f"QTextEdit {{ background-color: {log_cfg['background_color']}; "
            f"color: {log_cfg['text_color']}; font-family: {log_cfg['font_family']}; }}"
        )
        self._scroll_tolerance_min = max(0, int(log_cfg.get('scroll_tolerance_min', 2)))

        # batching buffer and timer
        self._buf: Deque[tuple[bool, str]] = deque()       # (is_html, text)
        self._flush_timer = QTimer(self)
        self._flush_timer.setInterval(int(log_cfg['flush_interval_ms']))
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
        tolerance = max(self._scroll_tolerance_min, bar.singleStep())
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
        self._apply_env()

        self.proc.readyReadStandardOutput.connect(self._on_stdout_buf)
        self.proc.readyReadStandardError.connect(self._on_stderr_buf)
        self.proc.finished.connect(self._drain_remaining)

        self.proc.finished.connect(lambda code, st: parent.on_task_finished(self.key, code, st))

        # per tab docker container name (for custom commands)
        self.container_name: str | None = None
        self.exec_id: str | None = None

    def start_shell(self, bash_cmd: str):
        self.parent._append_gui_html(
            self.key,
            f'<i>&gt; {html.escape(bash_cmd)}</i>',
            color=self.parent._command_log_color,
        )
        self.parent._log_cmd(bash_cmd)
        self._apply_env()
        self.proc.start('bash', ['-lc', bash_cmd])

    def start_program(self, program: str, args: list[str]):
        cmdline = program + ' ' + ' '.join(args)
        self.parent._append_gui_html(
            self.key,
            f'<i>&gt; {html.escape(cmdline)}</i>',
            color=self.parent._command_log_color,
        )
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

    # send HTML lines through the batcher
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

    # fast path for plain text, HTML only if ANSI present
    def _append_raw(self, data_bytes: bytes):
        if not data_bytes:
            return
        data = data_bytes.decode(errors='replace')
        if not data:
            return
        data = data.replace('\r\n', '\n').replace('\r', '\n')
        self.parent._prepare_tab_for_origin(self.key, 'container')
        if '\x1b[' in data:
            self.output.enqueue(True, ansi_to_html(data))
        else:
            # preserve newlines as plain text
            self.output.enqueue(False, data)

    def _apply_env(self):
        env = self.parent._build_process_environment()
        self.proc.setProcessEnvironment(env)

    def refresh_environment(self):
        if not self.is_running():
            self._apply_env()


class MainWindow(QMainWindow):
    def __init__(self, verbosity: int = 1):
        super().__init__()

        try:
            value = int(verbosity)
        except (TypeError, ValueError):
            value = 1
        self._verbosity = max(1, min(3, value))

        window_cfg = CONFIG['window']
        self.setWindowTitle(window_cfg['title'])
        if len(window_cfg.get('geometry', [])) == 4:
            self.setGeometry(*window_cfg['geometry'])

        self._killing = False
        self._last_search = ''
        self._yaml_path = None
        self._custom_counter = 0
        self._timers_cfg = CONFIG['timers']
        self._images_cfg = CONFIG['images']
        self._selected_image = self._images_cfg.get('default', '')
        self._image_choices: list[str] = []
        self._related_patterns: list[str] = []
        self._worlds_cfg = CONFIG['worlds']
        self._default_world = self._worlds_cfg.get('default', 'moelk_tables')
        self._selected_world = self._default_world
        self._scripts_dir = Path(__file__).resolve().parent / 'scripts'
        self._script_choices: list[str] = []
        self._script_active_tab_key: str | None = None
        self._roscore_container_name = 'mobipick-roscore'
        self._roscore_running_cached = False
        self._roscore_stopping = False
        self._roscore_last_start_ts: float | None = None
        self._terminal_cfg = CONFIG.get('terminal', {})
        self._terminal_launcher_template = str(self._terminal_cfg.get('launcher', 'gnome-terminal --title "{title}" -- bash -lc "{command}"'))
        self._terminal_title = str(self._terminal_cfg.get('title', 'Mobipick Terminal'))
        self._terminal_container_prefix = str(self._terminal_cfg.get('container_prefix', 'mobipick-terminal'))
        self._terminal_proc: QProcess | None = None
        self._terminal_container_name: str | None = None
        self._terminal_exec_id: str | None = None
        self._terminal_running_cached = False
        self._terminal_stopping = False
        self._terminal_stream_tab_key: str | None = None
        self._terminal_stream_counter = 0
        self._project_root = Path(__file__).resolve().parent
        self._toggle_states: dict[str, str] = {}
        self._last_log_origin: dict[str, str] = {}
        self._gui_log_color = str(CONFIG['log'].get('gui_log_color', '#ff00ff'))
        self._command_log_color = str(CONFIG['log'].get('command_log_color', '#4da3ff'))

        # sim state
        self._sim_container_name = 'mobipick-run'
        self._sim_xhost_granted = False
        self._sim_running_cached = False  # event driven sim state

        self.tasks: dict[str, ProcessTab] = {}
        self._bg_procs: list[QProcess] = []
        self._cleanup_done = False
        self._exit_in_progress = False
        self._exit_dialog: Optional[QMessageBox] = None

        central = QWidget()
        self.setCentralWidget(central)
        root = QVBoxLayout(central)

        # top controls
        top = QHBoxLayout()
        self.roscore_button = QPushButton()
        self.roscore_button.clicked.connect(self._on_roscore_toggle_clicked)
        top.addWidget(self.roscore_button)

        self.sim_toggle_button = QPushButton()
        self.sim_toggle_button.clicked.connect(self._on_sim_toggle_clicked)
        top.addWidget(self.sim_toggle_button)

        self.tables_button = QPushButton()
        self.tables_button.clicked.connect(self._on_tables_toggle_clicked)
        top.addWidget(self.tables_button)

        self.rviz_button = QPushButton()
        self.rviz_button.clicked.connect(self._on_rviz_toggle_clicked)
        top.addWidget(self.rviz_button)

        self.rqt_button = QPushButton()
        self.rqt_button.clicked.connect(self._on_rqt_toggle_clicked)
        top.addWidget(self.rqt_button)

        self.terminal_button = QPushButton()
        self.terminal_button.clicked.connect(self._on_terminal_toggle_clicked)
        top.addWidget(self.terminal_button)

        spacer = QWidget()
        spacer.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Preferred)
        top.addWidget(spacer)
        root.addLayout(top)

        self.clear_button = QPushButton('Clear Current Tab')
        self.clear_button.clicked.connect(self.clear_current_tab)

        self.clear_all_button = QPushButton('Clear All Tabs')
        self.clear_all_button.clicked.connect(self.clear_all_tabs)

        self.refresh_sim_button = QPushButton('Update Status')
        self.refresh_sim_button.setToolTip('Re-check running status for toggles')
        self.refresh_sim_button.clicked.connect(self._on_refresh_clicked)

        self.save_current_button = QPushButton('Save Current Log')
        self.save_current_button.clicked.connect(self.save_current_log)

        self.load_log_button = QPushButton('Load Log')
        self.load_log_button.clicked.connect(self.load_log_file)

        self.save_all_button = QPushButton('Save All Logs')
        self.save_all_button.clicked.connect(self.save_all_logs)

        # actions row
        actions = QHBoxLayout()

        self.world_label = QLabel('world_config:')
        actions.addWidget(self.world_label)

        self.world_combo = QComboBox()
        actions.addWidget(self.world_combo)
        self.world_combo.currentIndexChanged.connect(self._on_world_changed)

        self.image_label = QLabel('image:')
        actions.addWidget(self.image_label)

        self.image_combo = QComboBox()
        actions.addWidget(self.image_combo)
        self.image_combo.currentIndexChanged.connect(self._on_image_changed)

        self.reload_images_button = QPushButton('Refresh Images')
        self.reload_images_button.clicked.connect(self._reload_images)
        actions.addWidget(self.reload_images_button)

        root.addLayout(actions)

        scripts_row = QHBoxLayout()
        scripts_row.addWidget(QLabel('Scripts:'))
        self.script_combo = QComboBox()
        self.script_combo.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Preferred)
        scripts_row.addWidget(self.script_combo)
        self.refresh_scripts_button = QPushButton('Refresh Scripts')
        self.refresh_scripts_button.clicked.connect(self._on_refresh_scripts_clicked)
        scripts_row.addWidget(self.refresh_scripts_button)
        self.run_script_button = QPushButton('Run Script')
        self.run_script_button.clicked.connect(self._on_run_script_clicked)
        scripts_row.addWidget(self.run_script_button)
        root.addLayout(scripts_row)

        self.run_script_button.setEnabled(False)
        self.script_combo.setEnabled(False)

        self._ensure_scripts_dir()

        self._update_related_patterns()
        self._load_available_images()

        # custom command row
        cmdrow = QHBoxLayout()
        self.command_input = QLineEdit()
        self.command_input.setPlaceholderText('Enter custom command, press Enter to run')
        self.command_input.returnPressed.connect(self._on_command_input_return)
        cmdrow.addWidget(self.command_input)

        self.run_command_button = QPushButton('Run Command')
        self.run_command_button.clicked.connect(self._on_run_command_clicked)
        cmdrow.addWidget(self.run_command_button)

        self.stop_custom_button = QPushButton('Stop Command')
        self.stop_custom_button.setEnabled(False)
        self.stop_custom_button.clicked.connect(self._on_stop_custom_clicked)
        cmdrow.addWidget(self.stop_custom_button)

        self.reuse_checkbox = QCheckBox('Run in current custom tab')
        self.reuse_checkbox.setChecked(True)
        cmdrow.addWidget(self.reuse_checkbox)
        root.addLayout(cmdrow)

        # tabs
        self.tabs = QTabWidget()
        self.tabs.setTabsClosable(False)
        self.tabs.tabCloseRequested.connect(self.on_tab_close_requested)
        root.addWidget(self.tabs)

        controls_row = QHBoxLayout()
        controls_row.addWidget(self.clear_button)
        controls_row.addWidget(self.clear_all_button)

        spacer_controls = QWidget()
        spacer_controls.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Preferred)
        controls_row.addWidget(spacer_controls)

        controls_row.addWidget(self.save_current_button)
        controls_row.addWidget(self.load_log_button)
        controls_row.addWidget(self.save_all_button)
        controls_row.addWidget(self.refresh_sim_button)
        root.addLayout(controls_row)

        # search row (bottom)
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

        # fixed tabs
        self._ensure_tab('roscore', 'Roscore', closable=False)
        self._ensure_tab('sim', 'Sim', closable=False)
        self._ensure_tab('tables', 'Tables Demo', closable=False)
        self._ensure_tab('rviz', 'RViz', closable=False)
        self._ensure_tab('rqt', 'RQt Tables', closable=False)
        # central log tab
        self._ensure_tab('log', 'Log', closable=False)

        self._apply_env_to_all_tabs()
        self._refresh_script_options()

        # polling
        self.poll_timer = QTimer(self)
        self.poll_timer.timeout.connect(self._poll)
        self.poll_timer.start(int(self._timers_cfg['poll_ms']))

        self._sigint_timer = QTimer(self)
        self._sigint_timer.timeout.connect(self._check_sigint)
        self._sigint_timer.start(int(self._timers_cfg['sigint_check_ms']))

        self.load_yaml(DEFAULT_YAML_PATH)
        self._update_buttons()
        self.update_sim_status_from_poll(force=True)

        self._console_log(1, f'Mobipick Labs Control ready (verbosity {self._verbosity})')

        app_instance = QApplication.instance()
        if app_instance:
            app_instance.aboutToQuit.connect(self._ensure_cleanup_before_exit)

    # ---------- Log tab helpers ----------

    def _fmt_args(self, args_or_str) -> str:
        if isinstance(args_or_str, str):
            return args_or_str
        return ' '.join(shlex.quote(s) for s in args_or_str)

    def _compose_env_args(self) -> list[str]:
        env_args: list[str] = []
        compose_env = dict(CONFIG['process']['compose_run_env'])
        if self._selected_image:
            compose_env['MOBIPICK_IMAGE'] = self._selected_image
        world = self._current_world()
        if world:
            compose_env['MOBIPICK_WORLD'] = world
        master_uri = self._current_master_uri()
        if master_uri:
            compose_env['ROS_MASTER_URI'] = master_uri
        for key, value in compose_env.items():
            env_args.extend(['--env', f'{key}={value}'])
        return env_args

    def _current_master_uri(self) -> str:
        if getattr(self, '_roscore_stopping', False):
            return 'http://mobipick:11311'
        if getattr(self, '_roscore_running_cached', False):
            return f'http://{self._roscore_container_name}:11311'
        if 'roscore' in self.tasks and self.tasks['roscore'].is_running():
            self._roscore_running_cached = True
            return f'http://{self._roscore_container_name}:11311'
        return 'http://mobipick:11311'

    def _build_process_environment(self, extra: Optional[dict[str, str]] = None) -> QProcessEnvironment:
        env = QProcessEnvironment.systemEnvironment()
        for key, value in CONFIG['process']['qprocess_env'].items():
            env.insert(str(key), str(value))
        if self._selected_image:
            env.insert('MOBIPICK_IMAGE', self._selected_image)
        world = self._current_world()
        if world:
            env.insert('MOBIPICK_WORLD', world)
        if extra:
            for key, value in extra.items():
                env.insert(str(key), str(value))
        return env

    def _prepare_run_env(self, run_kwargs: dict) -> dict:
        env = run_kwargs.get('env')
        if env is None:
            env = os.environ.copy()
        else:
            env = {str(k): str(v) for k, v in env.items()}
        for key, value in CONFIG['process']['qprocess_env'].items():
            env[str(key)] = str(value)
        if self._selected_image:
            env['MOBIPICK_IMAGE'] = self._selected_image
        world = self._current_world()
        if world:
            env['MOBIPICK_WORLD'] = world
        run_kwargs['env'] = env
        return run_kwargs

    def _safe_docker_cmd(self, *docker_args: str, suppress_output: bool = True) -> list[str]:
        shell_cmd = shlex.join(['docker', *docker_args])
        if suppress_output:
            shell_cmd += ' >/dev/null 2>&1'
        shell_cmd += ' || true'
        return ['bash', '-lc', shell_cmd]

    def _ensure_network(self, *, log_key: str = 'log') -> bool:
        try:
            cp = self._sp_run(
                ['docker', 'network', 'ls', '-q', '--filter', 'name=^mobipick$'],
                stdout=subprocess.PIPE,
                stderr=subprocess.DEVNULL,
                check=False,
                text=True,
                log_key=log_key,
                log_stdout=False,
                log_stderr=False,
            )
            if (cp.stdout or '').strip():
                return True
        except Exception as exc:
            self._console_log(1, f'Failed to inspect docker network mobipick: {exc}')
        self._sp_run(['docker', 'network', 'create', 'mobipick'], check=False, log_key=log_key)
        return True

    @staticmethod
    def _split_image_ref(image_ref: str) -> tuple[str, str]:
        if not image_ref:
            return '', ''
        if ':' in image_ref and not image_ref.endswith(']'):
            repo, tag = image_ref.rsplit(':', 1)
            return repo, tag
        return image_ref, ''

    def _current_world(self) -> str:
        world = (self._selected_world or '').strip()
        if not world:
            world = self._default_world
        return world or 'moelk_tables'

    def _update_related_patterns(self):
        images_cfg = self._images_cfg
        patterns = list(images_cfg.get('related_container_keywords', []))
        if self._selected_image:
            patterns.append(self._selected_image)
        repo, tag = self._split_image_ref(self._selected_image)
        if repo:
            patterns.append(repo)
            patterns.extend(part for part in repo.split('/') if part)
        if tag:
            patterns.append(tag)
        patterns.extend(images_cfg.get('related_image_keywords', []))
        self._related_patterns = list(dict.fromkeys(p for p in patterns if p))

    def _reload_images(self):
        self._load_available_images(show_feedback=True)

    def _load_available_images(self, show_feedback: bool = False):
        images_cfg = self._images_cfg
        filters = [f.lower() for f in images_cfg.get('discovery_filters', []) if f]
        include_none = bool(images_cfg.get('include_none_tag', False))
        choices: list[str] = []
        try:
            run_kwargs = {'stdout': subprocess.PIPE, 'stderr': subprocess.PIPE, 'text': True, 'check': False}
            run_kwargs = self._prepare_run_env(run_kwargs)
            cp = subprocess.run(['docker', 'images', '--format', '{{.Repository}}:{{.Tag}}'], **run_kwargs)
            output_lines = (cp.stdout or '').splitlines() if isinstance(cp.stdout, str) else []
            if cp.returncode not in (0, None):
                self._console_log(1, f'docker images returned {cp.returncode}')
        except Exception as exc:
            output_lines = []
            self._console_log(1, f'Failed to list docker images: {exc}')

        for line in output_lines:
            line = line.strip()
            if not line:
                continue
            if ':' in line:
                repo, tag = line.rsplit(':', 1)
            else:
                repo, tag = line, ''
            if not include_none and ('<none>' in (repo.strip(), tag.strip())):
                continue
            if filters and not any(f in line.lower() for f in filters):
                continue
            choices.append(line)

        default_image = images_cfg.get('default', '')
        if default_image:
            choices.insert(0, default_image)

        choices = [choice for choice in dict.fromkeys(choice for choice in choices if choice)]

        prev_selection = self._selected_image
        if choices:
            if prev_selection not in choices:
                self._selected_image = choices[0]
            else:
                self._selected_image = prev_selection
        else:
            self._selected_image = ''

        self._image_choices = choices

        self.image_combo.blockSignals(True)
        self.image_combo.clear()
        if choices:
            for choice in choices:
                self.image_combo.addItem(choice)
            index = choices.index(self._selected_image)
            self.image_combo.setCurrentIndex(index)
            self.image_combo.setEnabled(True)
        else:
            self.image_combo.addItem('No images found')
            self.image_combo.setEnabled(False)
        self.image_combo.blockSignals(False)
        self.image_combo.setToolTip(self._selected_image or 'No image selected')

        if show_feedback:
            if choices:
                self._console_log(2, f'Available images: {", ".join(choices)}')
            else:
                QMessageBox.warning(self, 'Images', 'No matching docker images were found.')

        self._update_related_patterns()
        self._apply_env_to_all_tabs()

    def _apply_env_to_all_tabs(self):
        for tab in self.tasks.values():
            tab.refresh_environment()

    def _on_image_changed(self, index: int):
        if not self._image_choices:
            return
        if index < 0 or index >= len(self._image_choices):
            return
        new_image = self._image_choices[index]
        if new_image == self._selected_image:
            return
        self._selected_image = new_image
        self._console_log(2, f'Selected image: {new_image}')
        self.image_combo.setToolTip(new_image)
        self._update_related_patterns()
        self._apply_env_to_all_tabs()

    def _on_world_changed(self, index: int):
        if index < 0:
            return
        new_world = self.world_combo.itemText(index).strip()
        if not new_world:
            return
        if new_world == self._selected_world:
            return
        self._selected_world = new_world
        self._console_log(2, f'Selected world: {new_world}')
        self._apply_env_to_all_tabs()

    def _cleanup_script_available(self) -> bool:
        return Path(SCRIPT_CLEAN).is_file() and os.access(SCRIPT_CLEAN, os.X_OK)

    def _is_docker_command(self, args_or_str) -> bool:
        if isinstance(args_or_str, str):
            tokens = args_or_str.strip().split()
            return bool(tokens) and tokens[0] == 'docker'
        return bool(args_or_str) and args_or_str[0] == 'docker'

    def _console_log(self, level: int, message: str):
        if self._verbosity >= level:
            print(message, flush=True)

    @staticmethod
    def _decode_output(data) -> str:
        if data is None:
            return ''
        if isinstance(data, bytes):
            return data.decode(errors='replace')
        return str(data)

    def _append_log_html(self, html_text: str):
        if 'log' not in self.tasks:
            return
        tab = self._prepare_tab_for_origin('log', 'gui')
        tab.append_line_html(html_text)

    def _append_command_output(self, key: str, text, *, is_html: bool = False):
        if text is None:
            return
        if isinstance(text, bytes):
            text = text.decode(errors='replace')
        if not text:
            return
        lines = text.splitlines()
        if not lines:
            return
        for line in lines:
            if not line:
                self._append_html(key, '&nbsp;')
                continue
            if is_html:
                self._append_html(key, line)
            elif '\x1b[' in line:
                self._append_html(key, ansi_to_html(line))
            else:
                self._append_html(key, html.escape(line))

    def _log_cmd(self, args_or_str):
        ts = datetime.now().strftime('%H:%M:%S')

        if isinstance(args_or_str, str):
            fmt = args_or_str.strip()
        else:
            fmt = self._fmt_args(args_or_str)
        is_docker = self._is_docker_command(args_or_str)
        line = f'[{ts}] $ {fmt}'
        self._append_gui_html('log', html.escape(line), color=self._command_log_color)
        self._console_log(3, line)

    def _log_event(self, details: str):
        ts = datetime.now().strftime('%H:%M:%S')
        line = f'[{ts}] event: {details}'
        self._append_log_html(f'<span style="color:#ffa94d">{html.escape(line)}</span>')
        self._console_log(3, line)

    def _log_info(self, details: str):
        ts = datetime.now().strftime('%H:%M:%S')
        line = f'[{ts}] [INFO] {details}'
        self._append_log_html(f'<span style="color:#50fa7b">{html.escape(line)}</span>')
        self._console_log(2, line)

    def _log_button_click(self, button: QPushButton, fallback: str | None = None):
        label = button.text().strip()
        if not label:
            label = fallback or 'button'
        self._log_event(f'user clicked {label}')

    def _on_sim_toggle_clicked(self):
        self._log_button_click(self.sim_toggle_button, 'Sim Toggle')
        if not self._guard_toggle_action('sim', self.sim_toggle_button):
            return
        self.toggle_sim()

    def _on_refresh_clicked(self):
        self._log_button_click(self.refresh_sim_button)
        self._log_info('refreshing sim status view')
        self.update_sim_status_from_poll(force=True)

    def _on_stop_custom_clicked(self):
        self._log_button_click(self.stop_custom_button, 'Stop Command')
        self._log_info('stopping custom command tab')
        self.stop_custom_process()

    def _on_tables_toggle_clicked(self):
        self._log_button_click(self.tables_button, 'Tables Demo')
        if not self._guard_toggle_action('tables', self.tables_button):
            return
        self.toggle_tables_demo()

    def _on_rviz_toggle_clicked(self):
        self._log_button_click(self.rviz_button, 'RViz')
        if not self._guard_toggle_action('rviz', self.rviz_button):
            return
        self.toggle_rviz()

    def _on_rqt_toggle_clicked(self):
        self._log_button_click(self.rqt_button, 'RQt Tables Demo')
        if not self._guard_toggle_action('rqt', self.rqt_button):
            return
        self.toggle_rqt_tables_demo()

    def _on_terminal_toggle_clicked(self):
        self._log_button_click(self.terminal_button, 'Terminal')
        if not self._guard_toggle_action('terminal', self.terminal_button):
            return
        self.toggle_terminal()

    def _on_roscore_toggle_clicked(self):
        self._log_button_click(self.roscore_button, 'Roscore')
        if not self._guard_toggle_action('roscore', self.roscore_button):
            return
        self.toggle_roscore()

    def _on_refresh_scripts_clicked(self):
        self._log_button_click(self.refresh_scripts_button, 'Refresh Scripts')
        count = self._refresh_script_options()
        self._log_info(f'Refreshed scripts list ({count} available)')

    def _on_run_script_clicked(self):
        self._log_button_click(self.run_script_button, 'Run Script')
        if not self._guard_toggle_action('script', self.run_script_button):
            return
        self.toggle_script_execution()

    def _select_custom_tab_key(self) -> str:
        if self.reuse_checkbox.isChecked():
            cur_key = self._current_tab_key()
            if cur_key and cur_key.startswith('custom') and not self.tasks[cur_key].is_running():
                self._ensure_close_for_key(cur_key)
                return cur_key
            for k, t in self.tasks.items():
                if k.startswith('custom') and not t.is_running():
                    self._ensure_close_for_key(k)
                    return k
        return self._new_custom_tab_key(always_new=True)

    def toggle_script_execution(self):
        if not self._script_choices:
            QMessageBox.information(self, 'Scripts', 'No scripts available. Click Refresh Scripts to update the list.')
            return
        script = self.script_combo.currentText().strip()
        if not script or script not in self._script_choices:
            QMessageBox.information(self, 'Scripts', 'Selected script is not available.')
            return

        active_key = self._script_active_tab_key
        if active_key and active_key in self.tasks and self.tasks[active_key].is_running():
            self.set_script_visual('yellow', 'Stopping Script...', False)
            self._stop_script_tab()
            return

        self.set_script_visual('yellow', 'Starting Script...', False)

        def _run_script():
            nonlocal script
            self._log_info(f'running script: {script}')

            key_target = self._select_custom_tab_key()
            tab = self.tasks[key_target]
            exec_id = uuid.uuid4().hex
            tab.exec_id = exec_id
            tab.container_name = f'mpcmd-{exec_id[:10]}'
            inner = f"python3 /root/scripts/{self._sh_quote(script)}"
            args = [
                'compose', 'run', '--rm', '--name', tab.container_name,
                '--label', f'mobipick.exec={exec_id}', '--label', f'mobipick.tab={key_target}',
                *self._compose_env_args(),
                'mobipick_cmd', 'bash', '-lc', self._wrap_line_buffered(inner)
            ]
            tab.start_program('docker', args)
            self._script_active_tab_key = key_target
            self.set_script_visual('green', 'Stop Script', True)
            self._focus_tab(key_target)
            self._update_stop_custom_enabled()

        self._ensure_roscore_ready(_run_script)

    def _stop_script_tab(self):
        key = self._script_active_tab_key
        if not key or key not in self.tasks:
            self._script_active_tab_key = None
            self.set_script_visual('red', 'Run Script', bool(self._script_choices))
            self._update_stop_custom_enabled()
            return
        tab = self.tasks[key]
        if not tab.is_running():
            self._script_active_tab_key = None
            self.set_script_visual('red', 'Run Script', bool(self._script_choices))
            self._update_stop_custom_enabled()
            return
        self._stop_custom_tab(
            tab,
            on_stopped=self._finalize_script_stop,
        )

    def _finalize_script_stop(self):
        self._script_active_tab_key = None
        self.set_script_visual('red', 'Run Script', bool(self._script_choices))

    def _ensure_scripts_dir(self):
        try:
            self._scripts_dir.mkdir(parents=True, exist_ok=True)
        except Exception as exc:
            self._console_log(1, f'Failed to ensure scripts directory {self._scripts_dir}: {exc}')

    def _update_stop_custom_enabled(self):
        if not hasattr(self, 'stop_custom_button'):
            return
        running_custom = any(
            tab.is_running() for key, tab in self.tasks.items() if key.startswith('custom')
        )
        self.stop_custom_button.setEnabled(running_custom)

    def _docker_ps_ids(self, filters: list[str]) -> list[str]:
        if not filters:
            return []
        cmd = ['docker', 'ps', '-q']
        for flt in filters:
            cmd.extend(['--filter', flt])
        try:
            cp = subprocess.run(
                cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.DEVNULL,
                check=False,
                text=True,
            )
            return [line.strip() for line in (cp.stdout or '').splitlines() if line.strip()]
        except Exception as exc:
            self._console_log(1, f'Failed to query docker ps {filters}: {exc}')
            return []

    def _resolve_container_ids(self, *, name: str | None = None, exec_id: str | None = None) -> list[str]:
        ids: list[str] = []
        if exec_id:
            ids.extend(self._docker_ps_ids([f'label=mobipick.exec={exec_id}']))
        if name:
            ids.extend(self._docker_ps_ids([f'name={name}']))
            ids.extend(self._docker_ps_ids([f'label=com.docker.compose.oneoff.name={name}']))
        return list(dict.fromkeys(ids))

    def _extract_widget_html(self, widget: QTextEdit) -> str | None:
        doc = widget.document()
        plain = doc.toPlainText()
        if not plain or not plain.strip():
            return None
        html_content = doc.toHtml()
        return html_content if html_content.strip() else None

    def _suggest_log_filename(self, label: str) -> str:
        slug = re.sub(r'[^a-zA-Z0-9_-]+', '_', label.strip().lower()).strip('_')
        if not slug:
            slug = 'log'
        return f'{slug}.html'

    @staticmethod
    def _ensure_html_extension(path: str) -> str:
        return path if path.lower().endswith('.html') else f'{path}.html'

    def _resolve_unique_path(self, directory: str, filename: str) -> str:
        base, ext = os.path.splitext(filename)
        candidate = os.path.join(directory, filename)
        counter = 1
        while os.path.exists(candidate):
            candidate = os.path.join(directory, f'{base}-{counter}{ext}')
            counter += 1
        return candidate

    def _write_html_file(self, path: str, html_content: str) -> bool:
        try:
            with open(path, 'w', encoding='utf-8') as fh:
                fh.write(html_content)
            return True
        except Exception as exc:
            QMessageBox.critical(
                self,
                'Save Log',
                f"Failed to save log to {html.escape(path)}:\n{html.escape(str(exc))}",
            )
            return False


    def _collect_scripts(self) -> list[str]:
        if not self._scripts_dir.exists():
            return []
        try:
            cp = self._sp_run(
                ['ls', str(self._scripts_dir)],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                check=False,
                text=True,
                log_key='log',
                log_stdout=False,
                log_stderr=True,
            )
        except Exception as exc:
            self._console_log(1, f'Failed to list scripts: {exc}')
            return []
        output = self._decode_output(getattr(cp, 'stdout', ''))
        entries = [line.strip() for line in output.splitlines() if line.strip()]
        scripts: list[str] = []
        for entry in entries:
            path = self._scripts_dir / entry
            if path.is_file() and entry.endswith('.py'):
                scripts.append(entry)
        scripts.sort()
        return scripts

    def _refresh_script_options(self) -> int:
        scripts = self._collect_scripts()
        previous = self.script_combo.currentText().strip() if hasattr(self, 'script_combo') else ''
        self._script_choices = scripts
        if hasattr(self, 'script_combo'):
            self.script_combo.blockSignals(True)
            self.script_combo.clear()
            if scripts:
                self.script_combo.addItems(scripts)
                index = scripts.index(previous) if previous in scripts else 0
                self.script_combo.setCurrentIndex(index)
                self.script_combo.setEnabled(True)
            else:
                self.script_combo.addItem('No scripts found')
                self.script_combo.setCurrentIndex(0)
                self.script_combo.setEnabled(False)
            self.script_combo.blockSignals(False)
        script_running = bool(
            self._script_active_tab_key and self._script_active_tab_key in self.tasks and self.tasks[self._script_active_tab_key].is_running()
        )
        if script_running:
            self.set_script_visual('green', 'Stop Script', True)
        else:
            enabled = bool(scripts)
            self.set_script_visual('red', 'Run Script', enabled)
            if not enabled:
                self.run_script_button.setEnabled(False)
        return len(scripts)

    def _on_run_command_clicked(self):
        self._log_button_click(self.run_command_button)
        self.run_custom_command()

    def _on_command_input_return(self):
        self._log_event('user pressed enter to run command')
        self.run_custom_command()

    def _sp_run(
        self,
        args,
        *,
        log_key: str | None = None,
        log_stdout: bool = True,
        log_stderr: bool = True,
        **kwargs,
    ):
        # wrapper around subprocess.run with logging into the Log tab and verbosity-aware console output
        self._log_cmd(args)
        is_docker = self._is_docker_command(args)

        run_kwargs = dict(kwargs)
        run_kwargs = self._prepare_run_env(run_kwargs)
        if 'stdout' not in run_kwargs:
            run_kwargs['stdout'] = subprocess.PIPE
        if 'stderr' not in run_kwargs:
            run_kwargs['stderr'] = subprocess.PIPE
        if run_kwargs.get('text') is None and run_kwargs.get('stdout') == subprocess.PIPE:
            run_kwargs['text'] = True

        try:
            cp = subprocess.run(args, **run_kwargs)
        except subprocess.CalledProcessError as exc:
            msg = f'! command raised {exc.returncode}: {self._fmt_args(args)}'
            self._append_log_html(f"<i>{html.escape(msg)}</i>")
            self._console_log(1, msg)
            if log_key and log_stderr:
                self._append_command_output(log_key, msg)
            raise

        if self._is_clean_command(args) and cp.returncode == 0:
            self._cleanup_done = True

        self._maybe_emit_subprocess_output(cp, run_kwargs, is_docker)

        if log_key:
            if log_stdout and run_kwargs.get('stdout') == subprocess.PIPE:
                self._append_command_output(log_key, getattr(cp, 'stdout', None))
            if log_stderr and run_kwargs.get('stderr') == subprocess.PIPE:
                self._append_command_output(log_key, getattr(cp, 'stderr', None))

        return cp

    def _is_clean_command(self, args_or_str) -> bool:
        if isinstance(args_or_str, str):
            return args_or_str.strip().split()[:1] == [SCRIPT_CLEAN]
        if not args_or_str:
            return False
        return args_or_str[0] == SCRIPT_CLEAN

    def _maybe_emit_subprocess_output(self, cp: subprocess.CompletedProcess, run_kwargs: dict, is_docker: bool):
        stdout_setting = run_kwargs.get('stdout')
        stderr_setting = run_kwargs.get('stderr')

        if stdout_setting == subprocess.PIPE and not is_docker:
            out_text = self._decode_output(getattr(cp, 'stdout', None)).strip()
            if out_text:
                for line in out_text.splitlines():
                    self._console_log(3, line)

        if stderr_setting == subprocess.PIPE:
            err_text = self._decode_output(getattr(cp, 'stderr', None)).strip()
            if err_text:
                for line in err_text.splitlines():
                    self._console_log(1, line)

    def _ensure_cleanup_before_exit(self):
        if self._exit_in_progress:
            return
        if self._cleanup_done:
            return
        if not self._cleanup_script_available():
            self._console_log(2, 'clean.bash not found or not executable; skipping exit cleanup.')
            return
        self._console_log(1, 'Running clean.bash before exit...')
        try:
            result = self._sp_run([SCRIPT_CLEAN], check=False, log_key='log')
            if isinstance(result, subprocess.CompletedProcess) and result.returncode == 0:
                self._cleanup_done = True
        except Exception as exc:
            self._console_log(1, f'Failed to execute clean.bash during exit: {exc}')

    def _run_command_sequence(
        self,
        commands: list[list[str]],
        *,
        env: dict | None = None,
        on_finished: Callable[[], None] | None = None,
        log_key: str | None = None,
    ):
        if not commands:
            if on_finished:
                QTimer.singleShot(0, on_finished)
            return

        proc = QProcess(self)
        proc.setProcessEnvironment(self._build_process_environment(env or {}))

        queue: deque[list[str]] = deque(commands)
        current: list[str] | None = None

        def start_next():
            nonlocal current
            if not queue:
                cleanup()
                return
            current = queue.popleft()
            self._log_cmd(current)
            proc.setProcessEnvironment(self._build_process_environment(env or {}))
            proc.start(current[0], current[1:])

        def handle_stdout():
            data = bytes(proc.readAllStandardOutput())
            if data and log_key:
                self._append_command_output(log_key, data)

        def handle_stderr():
            data = bytes(proc.readAllStandardError())
            if data and log_key:
                self._append_command_output(log_key, data)

        def handle_finished(code: int, _status):
            nonlocal current
            if current is None:
                return
            handle_stdout()
            handle_stderr()
            if code != 0:
                msg = f'! command exited {code}: {self._fmt_args(current)}'
                self._append_log_html(f"<i>{html.escape(msg)}</i>")
                self._console_log(1, msg)
            if current and self._is_clean_command(current) and code == 0:
                self._cleanup_done = True
            current = None
            QTimer.singleShot(0, start_next)

        def handle_error(_error):
            nonlocal current
            if current is None:
                return
            handle_stdout()
            handle_stderr()
            err = proc.errorString()
            msg = f'! command failed: {self._fmt_args(current)} ({err})'
            self._append_log_html(f"<i>{html.escape(msg)}</i>")
            self._console_log(1, msg)
            if current and self._is_clean_command(current):
                self._cleanup_done = False
            current = None
            QTimer.singleShot(0, start_next)

        def cleanup():
            if proc in self._bg_procs:
                self._bg_procs.remove(proc)
            proc.deleteLater()
            if on_finished:
                QTimer.singleShot(0, on_finished)

        proc.readyReadStandardOutput.connect(handle_stdout)
        proc.readyReadStandardError.connect(handle_stderr)
        proc.finished.connect(handle_finished)
        proc.errorOccurred.connect(handle_error)
        self._bg_procs.append(proc)
        start_next()

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
            close_cfg = CONFIG['buttons']['close']
            btn = QPushButton(close_cfg['text'], self)
            btn.setToolTip(close_cfg['tooltip'])
            size = close_cfg.get('size', 18)
            btn.setFixedSize(size, size)
            btn.setStyleSheet(close_cfg.get('stylesheet', ''))
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
        if key == self._terminal_stream_tab_key:
            self._terminal_stream_tab_key = None
        if not (key.startswith('custom') or key.startswith('loadedlog') or key.startswith('terminal')):
            QMessageBox.information(self, 'Info', 'Only custom, terminal, and loaded log tabs can be closed.')
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
                    self._append_gui_html('sim', f'<i>YAML format not recognized in {html.escape(path)}</i>')
                self._yaml_path = path
            else:
                self._append_gui_html('sim', f'<i>YAML not found: {html.escape(path)}</i>')
        except ImportError:
            self._append_gui_html('sim', '<i>PyYAML not installed, using default option set.</i>')
        except Exception as e:
            self._append_gui_html('sim', f'<i>Failed to load YAML: {html.escape(str(e))}</i>')
        if not values:
            values = ['moelk_tables']
        self.world_combo.blockSignals(True)
        self.world_combo.addItems(values)
        target_world = self._selected_world if self._selected_world in values else self._default_world
        if target_world in values:
            index = values.index(target_world)
        else:
            index = 0
            self._selected_world = values[0]
        self.world_combo.setCurrentIndex(index)
        self.world_combo.blockSignals(False)
        self._selected_world = self.world_combo.currentText().strip() or self._default_world
        self._on_world_changed(self.world_combo.currentIndex())

    # ---------- Sim control ----------

    def _grant_x(self):
        if not self._sim_xhost_granted:
            self._sp_run(['xhost', '+local:root'], check=False, log_key='sim')
            self._sim_xhost_granted = True

    def _revoke_x(self):
        if self._sim_xhost_granted:
            self._sp_run(['xhost', '-local:root'], check=False, log_key='sim')
            self._sim_xhost_granted = False

    def is_roscore_running(self) -> bool:
        try:
            cp = self._sp_run(['docker', 'ps', '--format', '{{.Names}}'],
                              stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, check=False, text=True)
            names = set(cp.stdout.strip().splitlines())
            return (self._roscore_container_name in names) or self.tasks['roscore'].is_running()
        except Exception:
            return self.tasks['roscore'].is_running()

    def is_sim_running(self) -> bool:
        try:
            cp = self._sp_run(['docker', 'ps', '--format', '{{.Names}}'],
                              stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, check=False, text=True)
            names = set(cp.stdout.strip().splitlines())
            return (self._sim_container_name in names) or self.tasks['sim'].is_running()
        except Exception:
            return False

    def toggle_roscore(self):
        if self._roscore_stopping:
            return
        if self.is_roscore_running():
            self.shutdown_roscore()
        else:
            self.bring_up_roscore()

    def toggle_sim(self):
        if self._killing:
            return
        if self.is_sim_running():
            self.shutdown_sim()
        else:
            self.bring_up_sim()

    def _roscore_delay_ms(self) -> int:
        try:
            return max(0, int(self._timers_cfg.get('roscore_start_delay_ms', 1000)))
        except Exception:
            return 1000

    def _ensure_roscore_ready(self, callback: Callable[[], None], *, attempt: int = 0):
        delay_ms = self._roscore_delay_ms()
        last_start = self._roscore_last_start_ts

        if self._roscore_running_cached or self.tasks['roscore'].is_running():
            if delay_ms > 0 and last_start is not None:
                elapsed_ms = int((time.monotonic() - last_start) * 1000)
                remaining = delay_ms - elapsed_ms
                if remaining > 0:
                    QTimer.singleShot(remaining, lambda: self._ensure_roscore_ready(callback, attempt=attempt + 1))
                    return
            callback()
            return

        if self._roscore_stopping:
            self._log_info('roscore is shutting down; retrying shortly...')
            QTimer.singleShot(delay_ms or 1000, lambda: self._ensure_roscore_ready(callback, attempt=attempt + 1))
            return

        try:
            if self.is_roscore_running():
                self._roscore_running_cached = True
                self.set_roscore_visual('green', 'Stop Roscore', enabled=True)
                QTimer.singleShot(delay_ms or 0, lambda: self._ensure_roscore_ready(callback, attempt=attempt + 1))
                return
        except Exception:
            pass

        self._log_info('roscore not running; starting automatically')
        self.bring_up_roscore()
        QTimer.singleShot(delay_ms or 1000, lambda: self._ensure_roscore_ready(callback, attempt=attempt + 1))

    def bring_up_roscore(self):
        if self._roscore_stopping:
            return
        self._log_info('starting roscore master')
        self.set_roscore_visual('yellow', 'Starting Roscore...', False)
        self._ensure_network(log_key='roscore')
        tab = self._ensure_tab('roscore', 'Roscore', closable=False)
        tab.container_name = self._roscore_container_name
        exec_id = uuid.uuid4().hex
        tab.exec_id = exec_id
        inner = 'roscore'
        self._roscore_running_cached = True
        self._roscore_stopping = False
        self._roscore_last_start_ts = time.monotonic()
        self.set_roscore_visual('green', 'Stop Roscore', enabled=True)
        args = [
            'compose', 'run', '--rm', '--name', self._roscore_container_name,
            '--label', f'mobipick.exec={exec_id}', '--label', f'mobipick.tab={tab.key}',
            *self._compose_env_args(),
            'mobipick_cmd', 'bash', '-lc', self._wrap_line_buffered(inner)
        ]
        tab.start_program('docker', args)
        self._focus_tab('roscore')

    def shutdown_roscore(self):
        if self._roscore_stopping:
            return
        self._log_info('stopping roscore master')
        self.set_roscore_visual('yellow', 'Shutting down...', enabled=False)
        self._roscore_stopping = True

        sim_tab = self.tasks.get('sim')
        sim_running = bool(sim_tab and sim_tab.is_running())
        if not sim_running:
            sim_running = bool(self._sim_running_cached)

        if sim_running:
            self._killing = True
            self.set_toggle_visual('yellow', 'Shutting down...', False)
        else:
            self._killing = False
            self._disable_toggle_preserving_visual('sim', self.sim_toggle_button)
        tables_running = 'tables' in self.tasks and self.tasks['tables'].is_running()
        rviz_running = 'rviz' in self.tasks and self.tasks['rviz'].is_running()
        rqt_running = 'rqt' in self.tasks and self.tasks['rqt'].is_running()
        script_running = False
        if self._script_active_tab_key and self._script_active_tab_key in self.tasks:
            script_running = self.tasks[self._script_active_tab_key].is_running()

        if tables_running:
            self.set_tables_visual('yellow', 'Shutting down...', False)
        else:
            self._disable_toggle_preserving_visual('tables', self.tables_button)

        if rviz_running:
            self.set_rviz_visual('yellow', 'Shutting down...', False)
        else:
            self._disable_toggle_preserving_visual('rviz', self.rviz_button)

        if rqt_running:
            self.set_rqt_visual('yellow', 'Shutting down...', False)
        else:
            self._disable_toggle_preserving_visual('rqt', self.rqt_button)

        if script_running:
            self.set_script_visual('yellow', 'Shutting down...', False)
        else:
            self._disable_toggle_preserving_visual('script', self.run_script_button)

        if self._terminal_is_active():
            self.stop_terminal()
        else:
            self._disable_toggle_preserving_visual('terminal', self.terminal_button)

        tab = self._ensure_tab('roscore', 'Roscore', closable=False)

        pid = tab.pid()
        if pid:
            try:
                os.kill(pid, signal.SIGINT)
                self._append_gui_html(tab.key, '<i>Sent SIGINT to roscore (graceful stop)...</i>')
                self._log_cmd(f'kill -SIGINT {pid}')
            except Exception as e:
                self._append_gui_html(tab.key, f'<i>Failed to send SIGINT: {html.escape(str(e))}</i>')

        def _cleanup():
            commands: list[list[str]] = []
            commands += self._docker_stop_if_exists(self._roscore_container_name, tab, exec_id=tab.exec_id)
            commands += self._stop_all_related(tab, exclude={self._roscore_container_name})

            clean_exists = self._cleanup_script_available()
            if not self._cleanup_done and clean_exists:
                self._append_gui_html(tab.key, '<i>Invoking clean.bash for final cleanup...</i>')
                commands.append([SCRIPT_CLEAN])
            elif not clean_exists:
                self._append_gui_html(tab.key, '<i>clean.bash not found or not executable.</i>')

            def _finalize():
                self._roscore_running_cached = False
                self._roscore_stopping = False
                self._roscore_last_start_ts = None
                tab.container_name = None
                tab.exec_id = None
                self.set_roscore_visual('red', 'Start Roscore', enabled=True)
                self._revoke_x()
                self._sim_running_cached = False
                self._killing = False
                self.set_toggle_visual('red', 'Start Sim', enabled=True)
                self.set_tables_visual('red', 'Run Tables Demo', True)
                self.set_rviz_visual('red', 'Start RViz', True)
                self.set_rqt_visual('red', 'Start RQt Tables', True)
                self._script_active_tab_key = None
                self.set_script_visual('red', 'Run Script', bool(self._script_choices))
                self._terminal_stopping = False
                self._terminal_running_cached = False
                if self._terminal_stream_tab_key and self._terminal_stream_tab_key in self.tasks:
                    self._append_gui_html(self._terminal_stream_tab_key, '<i>Terminal session closed.</i>')
                    self._terminal_stream_tab_key = None
                self.set_terminal_visual('red', 'Open Terminal', True)
                self._update_stop_custom_enabled()

            if commands:
                self._run_command_sequence(commands, on_finished=_finalize, log_key=tab.key)
            else:
                _finalize()

        delay = int(self._timers_cfg.get('custom_tab_sigint_delay_ms', 1000))
        QTimer.singleShot(delay, _cleanup)

    # event driven bring up
    def bring_up_sim(self):
        if self._killing:
            return
        self.set_toggle_visual('yellow', 'Starting Sim...', False)

        def _start_sim():
            world = self._current_world()
            self._log_info(f'starting simulation stack (world {world})')
            self._grant_x()

            tab = self._ensure_tab('sim', 'Sim', closable=False)
            tab.container_name = self._sim_container_name  # ensure sim tab is addressable
            exec_id = uuid.uuid4().hex
            tab.exec_id = exec_id

            args = [
                'compose', 'run', '--rm', '--name', self._sim_container_name,
                '--label', f'mobipick.exec={exec_id}', '--label', f'mobipick.tab={tab.key}',
                *self._compose_env_args(),
                'mobipick'
            ]
            tab.start_program('docker', args)
            self._focus_tab('sim')

            # event driven state
            self._sim_running_cached = True
            self._killing = False
            self.set_toggle_visual('green', 'Stop Sim', enabled=True)

        self._ensure_roscore_ready(_start_sim)

    def _graceful_stop_container(
        self,
        name: str | None,
        tab: ProcessTab | None = None,
        exec_id: str | None = None,
        on_finished: Callable[[], None] | None = None,
    ):
        commands = self._collect_container_commands(
            name,
            exec_id=exec_id,
            log_key=(tab.key if tab else 'log'),
        )
        if not commands:
            if tab:
                label = name or (exec_id or 'container')
                self._append_gui_html(tab.key, f'<i>No running container named {html.escape(label)}</i>')
            if on_finished:
                on_finished()
            return
        if tab:
            label = name or (exec_id or 'container')
            self._append_gui_html(tab.key, f'<i>docker kill -s INT {html.escape(label)}</i>')
            self._append_gui_html(tab.key, f'<i>docker stop {html.escape(label)}</i>')
        self._run_command_sequence(
            commands,
            log_key=(tab.key if tab else 'log'),
            on_finished=on_finished,
        )

    # event driven shutdown
    def shutdown_sim(self):
        self._log_info('stopping simulation stack')
        self.set_toggle_visual('yellow', 'Shutting down...', enabled=False)
        self._killing = True

        tab = self._ensure_tab('sim', 'Sim', closable=False)

        pid = tab.pid()
        if pid:
            try:
                os.kill(pid, signal.SIGINT)
                self._append_gui_html(tab.key, '<i>Sent SIGINT to docker compose (graceful stop)...</i>')
                self._log_cmd(f'kill -SIGINT {pid}')
            except Exception as e:
                self._append_gui_html(tab.key, f'<i>Failed to send SIGINT: {html.escape(str(e))}</i>')

        def _fallbacks():
            commands: list[list[str]] = []

            # stop sim container if present
            commands += self._docker_stop_if_exists(self._sim_container_name, tab, exec_id=tab.exec_id)

            def _finalize():
                self._revoke_x()
                self._sim_running_cached = False
                self._killing = False
                self.set_toggle_visual('red', 'Start Sim', enabled=True)
                tab.exec_id = None

            if commands:
                self._run_command_sequence(commands, on_finished=_finalize, log_key=tab.key)
            else:
                _finalize()

        QTimer.singleShot(int(self._timers_cfg['sim_shutdown_delay_ms']), _fallbacks)

    def _collect_container_commands(
        self,
        name: str | None,
        *,
        exec_id: str | None = None,
        log_key: str | None = None,
        include_int: bool = True,
    ) -> list[list[str]]:
        commands: list[list[str]] = []
        ids = self._resolve_container_ids(name=name, exec_id=exec_id)
        if not ids:
            return commands
        for cid in ids:
            if include_int:
                commands.append(self._safe_docker_cmd('kill', '-s', 'INT', cid))
            commands.append(self._safe_docker_cmd('stop', cid))
        return commands

    def _docker_stop_if_exists(self, name: str | None, tab: ProcessTab | None = None, exec_id: str | None = None) -> list[list[str]]:
        commands: list[list[str]] = []
        ids = self._resolve_container_ids(name=name, exec_id=exec_id)
        for cid in ids:
            commands.append(self._safe_docker_cmd('stop', cid))
            if tab:
                self._append_gui_html(tab.key, f'<i>docker stop {html.escape(cid)}</i>')
        return commands

    def _collect_exit_commands(self) -> list[list[str]]:
        commands: list[list[str]] = []
        commands += self._collect_container_commands(self._sim_container_name, log_key='log')
        commands += self._stop_all_related(None)
        if not self._cleanup_done and self._cleanup_script_available():
            commands.append([SCRIPT_CLEAN])
        return commands

    # Stop all related containers, robust name/image/label pattern matching.
    # Sends INT first for a graceful shutdown of GUIs, then docker stop.
    def _stop_all_related(self, tab: ProcessTab | None = None, *, exclude: set[str] | None = None) -> list[list[str]]:
        patterns = list(self._related_patterns or [])
        commands: list[list[str]] = []
        try:
            cp = self._sp_run(
                ['docker', 'ps', '-a', '--format', '{{.ID}}|{{.Names}}|{{.Image}}|{{.Labels}}|{{.Status}}'],
                stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, check=False, text=True
            )
            patterns_lower = [p.lower() for p in patterns]
            matches: list[tuple[str, str, str]] = []  # (id, name, status)
            for ln in cp.stdout.splitlines():
                ln = ln.strip()
                if not ln:
                    continue
                parts = ln.split('|', 4)
                if len(parts) < 5:
                    continue
                cid, cname, cimage, clabels, cstatus = parts
                if exclude and cname in exclude:
                    continue
                hay_lower = f'{cname} {cimage} {clabels}'.lower()
                if patterns_lower and not any(p in hay_lower for p in patterns_lower):
                    continue
                matches.append((cid, cname, cstatus))

            if not matches:
                if tab:
                    self._append_gui_html(tab.key, '<i>No related containers found.</i>')
                return commands

            running_ids: list[str] = []
            skipped: list[str] = []
            for cid, _, cstatus in matches:
                if cstatus.lower().startswith('up'):
                    running_ids.append(cid)
                else:
                    skipped.append(cid)

            if running_ids:
                if tab:
                    self._append_gui_html(
                        tab.key,
                        f'<i>Sending INT to related containers: {html.escape(" ".join(running_ids))}</i>'
                    )
                for cid in running_ids:
                    commands.append(self._safe_docker_cmd('kill', '-s', 'INT', cid))

            if running_ids:
                if tab:
                    self._append_gui_html(
                        tab.key,
                        f'<i>Stopping related containers: {html.escape(" ".join(running_ids))}</i>'
                    )
                for cid in running_ids:
                    commands.append(self._safe_docker_cmd('stop', cid))

            if skipped and tab:
                self._append_gui_html(
                    tab.key,
                    f'<i>Skipping already stopped containers: {html.escape(" ".join(skipped))}</i>'
                )
        except Exception as e:
            if tab:
                self._append_gui_html(tab.key, f'<i>Error while stopping related containers: {html.escape(str(e))}</i>')
            else:
                self._console_log(1, f'Error while stopping related containers: {e}')
        return commands

    # optionally keep a manual refresh helper for rare external changes
    def update_sim_status_from_poll(self, force=False):
        names: set[str] | None = None
        if force:
            try:
                cp = self._sp_run(['docker', 'ps', '--format', '{{.Names}}'],
                                  stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, check=False, text=True)
                names = set(cp.stdout.strip().splitlines())
            except Exception:
                names = None

        self._update_roscore_status(force=force, names=names)
        self._update_terminal_status(force=force, names=names)

        if self._killing:
            self.set_toggle_visual('yellow', 'Shutting down...', enabled=False)
            return

        running = self._sim_running_cached or self.tasks['sim'].is_running()
        if force:
            if names is not None:
                running = (self._sim_container_name in names) or self.tasks['sim'].is_running()
            self._sim_running_cached = running
        else:
            self._sim_running_cached = running

        self.set_toggle_visual('green', 'Stop Sim', True) if self._sim_running_cached \
            else self.set_toggle_visual('red', 'Start Sim', True)

    def _update_roscore_status(self, *, force: bool = False, names: set[str] | None = None):
        if self._roscore_stopping:
            self.set_roscore_visual('yellow', 'Shutting down...', enabled=False)
            return

        running = self._roscore_running_cached or self.tasks['roscore'].is_running()
        if force:
            if names is None:
                try:
                    cp = self._sp_run(['docker', 'ps', '--format', '{{.Names}}'],
                                      stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, check=False, text=True)
                    names = set(cp.stdout.strip().splitlines())
                except Exception:
                    names = None
            if names is not None:
                running = (self._roscore_container_name in names) or self.tasks['roscore'].is_running()

        if running:
            self._roscore_running_cached = True
            self.set_roscore_visual('green', 'Stop Roscore', enabled=True)
        else:
            self._roscore_running_cached = False
            self.set_roscore_visual('red', 'Start Roscore', enabled=True)

    def _update_terminal_status(self, *, force: bool = False, names: set[str] | None = None):
        if self._terminal_stopping:
            self.set_terminal_visual('yellow', 'Closing Terminal...', False)
            return

        running = self._terminal_proc is not None and self._terminal_proc.state() != QProcess.NotRunning

        if force and names is not None and self._terminal_container_name:
            running = running or (self._terminal_container_name in names)

        self._terminal_running_cached = running

        if running:
            self.set_terminal_visual('green', 'Close Terminal', True)
        else:
            self.set_terminal_visual('red', 'Open Terminal', True)

    def _set_toggle_state(self, key: str, button: QPushButton, state: str, text: str, enabled: bool):
        button.setText(text)
        toggle_cfg = CONFIG['buttons']['sim_toggle']
        states = toggle_cfg['states']
        default_state = states.get('grey', next(iter(states.values())))
        state_cfg = states.get(state, default_state)
        padding = toggle_cfg.get('padding_px', 6)
        disabled_opacity = toggle_cfg.get('disabled_opacity', 0.85)
        bg = state_cfg.get('bg', '#6c757d')
        fg = state_cfg.get('fg', '#ffffff')
        button.setStyleSheet(
            f'QPushButton {{ background-color: {bg}; color: {fg}; border: none; padding: {padding}px; }}'
            f'QPushButton:disabled {{ opacity: {disabled_opacity}; }}'
        )
        button.setEnabled(enabled)
        self._toggle_states[key] = state

    def _disable_toggle_preserving_visual(self, key: str, button: QPushButton):
        current_state = self._toggle_states.get(key, 'red')
        current_text = button.text()
        self._set_toggle_state(key, button, current_state, current_text, False)

    def set_toggle_visual(self, state: str, text: str, enabled: bool):
        self._set_toggle_state('sim', self.sim_toggle_button, state, text, enabled)

    def set_roscore_visual(self, state: str, text: str, enabled: bool):
        self._set_toggle_state('roscore', self.roscore_button, state, text, enabled)

    def set_tables_visual(self, state: str, text: str, enabled: bool):
        self._set_toggle_state('tables', self.tables_button, state, text, enabled)

    def set_rviz_visual(self, state: str, text: str, enabled: bool):
        self._set_toggle_state('rviz', self.rviz_button, state, text, enabled)

    def set_rqt_visual(self, state: str, text: str, enabled: bool):
        self._set_toggle_state('rqt', self.rqt_button, state, text, enabled)

    def set_script_visual(self, state: str, text: str, enabled: bool):
        self._set_toggle_state('script', self.run_script_button, state, text, enabled)

    def set_terminal_visual(self, state: str, text: str, enabled: bool):
        self._set_toggle_state('terminal', self.terminal_button, state, text, enabled)

    def _guard_toggle_action(self, key: str, button: QPushButton) -> bool:
        if self._toggle_states.get(key) != 'yellow':
            return True
        text = button.text().strip().lower()
        if 'shutting' in text or 'stop' in text:
            msg = 'Process is shutting down, please wait.'
        elif 'starting' in text or 'start' in text:
            msg = 'Process is starting, please wait.'
        else:
            msg = 'Process is busy, please wait.'
        QMessageBox.information(self, 'Please Wait', msg)
        return False

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

    def toggle_tables_demo(self):
        tab = self.tasks['tables']
        if tab.is_running():
            self.stop_tables_demo()
        else:
            self.run_tables_demo()

    def run_tables_demo(self):
        tab = self.tasks['tables']
        if tab.is_running():
            self.set_tables_visual('green', 'Stop Tables Demo', True)
            self._focus_tab('tables')
            return

        self.set_tables_visual('yellow', 'Starting Tables Demo...', False)

        def _start_tables():
            self._log_info('launching tables demo container')
            tab = self._ensure_tab('tables', 'Tables Demo', closable=False)
            exec_id = uuid.uuid4().hex
            tab.exec_id = exec_id
            tab.container_name = f'mpcmd-{exec_id[:10]}'
            inner = 'rosrun tables_demo_planning tables_demo_node.py'
            args = [
                'compose', 'run', '--rm', '--name', tab.container_name,
                '--label', f'mobipick.exec={exec_id}', '--label', f'mobipick.tab={tab.key}',
                *self._compose_env_args(),
                'mobipick_cmd', 'bash', '-lc', self._wrap_line_buffered(inner)
            ]
            tab.start_program('docker', args)
            self.set_tables_visual('green', 'Stop Tables Demo', True)
            self._focus_tab('tables')

        self._ensure_roscore_ready(_start_tables)

    def stop_tables_demo(self):
        tab = self.tasks['tables']
        if not tab.is_running():
            self.set_tables_visual('red', 'Run Tables Demo', True)
            return
        self.set_tables_visual('yellow', 'Stopping Tables Demo...', False)
        self._stop_custom_tab(
            tab,
            on_stopped=lambda: self.set_tables_visual('red', 'Run Tables Demo', True),
        )

    def toggle_rviz(self):
        tab = self.tasks['rviz']
        if tab.is_running():
            self.stop_rviz()
        else:
            self.open_rviz()

    def open_rviz(self):
        tab = self.tasks['rviz']
        if tab.is_running():
            self.set_rviz_visual('green', 'Stop RViz', True)
            self._focus_tab('rviz')
            return

        self.set_rviz_visual('yellow', 'Starting RViz...', False)

        def _start_rviz():
            self._log_info('starting RViz viewer')
            self._grant_x()
            tab = self._ensure_tab('rviz', 'RViz', closable=False)
            exec_id = uuid.uuid4().hex
            tab.exec_id = exec_id
            tab.container_name = f'mpcmd-{exec_id[:10]}'
            rviz_cmd = 'rosrun rviz rviz -d $(rospack find tables_demo_bringup)/config/pick_n_place.rviz __ns:=mobipick'
            args = [
                'compose', 'run', '--rm', '--name', tab.container_name,
                '--label', f'mobipick.exec={exec_id}', '--label', f'mobipick.tab={tab.key}',
                *self._compose_env_args(),
                'mobipick_cmd', 'bash', '-lc', self._wrap_line_buffered(rviz_cmd)
            ]
            tab.start_program('docker', args)
            self.set_rviz_visual('green', 'Stop RViz', True)
            self._focus_tab('rviz')

        self._ensure_roscore_ready(_start_rviz)

    def stop_rviz(self):
        tab = self.tasks['rviz']
        if not tab.is_running():
            self.set_rviz_visual('red', 'Start RViz', True)
            return
        self.set_rviz_visual('yellow', 'Stopping RViz...', False)
        self._stop_custom_tab(
            tab,
            on_stopped=lambda: self.set_rviz_visual('red', 'Start RViz', True),
        )

    def toggle_rqt_tables_demo(self):
        tab = self.tasks['rqt']
        if tab.is_running():
            self.stop_rqt_tables_demo()
        else:
            self.open_rqt_tables_demo()

    def open_rqt_tables_demo(self):
        tab = self.tasks['rqt']
        if tab.is_running():
            self.set_rqt_visual('green', 'Stop RQt Tables', True)
            self._focus_tab('rqt')
            return

        self.set_rqt_visual('yellow', 'Starting RQt Tables...', False)

        def _start_rqt():
            world = self.world_combo.currentText().strip() or 'moelk_tables'
            self._log_info(f'starting rqt tables demo for {world}')
            self._grant_x()
            tab = self._ensure_tab('rqt', 'RQt Tables', closable=False)
            exec_id = uuid.uuid4().hex
            tab.exec_id = exec_id
            tab.container_name = f'mpcmd-{exec_id[:10]}'
            cmd = f'roslaunch rqt_tables_demo rqt_tables_demo.launch namespace:=mobipick world_config:={self._sh_quote(world)}'
            args = [
                'compose', 'run', '--rm', '--name', tab.container_name,
                '--label', f'mobipick.exec={exec_id}', '--label', f'mobipick.tab={tab.key}',
                *self._compose_env_args(),
                'mobipick_cmd', 'bash', '-lc', self._wrap_line_buffered(cmd)
            ]
            tab.start_program('docker', args)
            self.set_rqt_visual('green', 'Stop RQt Tables', True)
            self._focus_tab('rqt')

        self._ensure_roscore_ready(_start_rqt)

    def stop_rqt_tables_demo(self):
        tab = self.tasks['rqt']
        if not tab.is_running():
            self.set_rqt_visual('red', 'Start RQt Tables', True)
            return
        self.set_rqt_visual('yellow', 'Stopping RQt Tables...', False)
        self._stop_custom_tab(
            tab,
            on_stopped=lambda: self.set_rqt_visual('red', 'Start RQt Tables', True),
        )

    def toggle_terminal(self):
        if self._terminal_stopping:
            return
        if self._terminal_is_active():
            self.stop_terminal()
        else:
            self.open_terminal()

    def open_terminal(self):
        if self._terminal_stopping or self._terminal_is_active():
            return
        self.set_terminal_visual('yellow', 'Starting Terminal...', False)

        def _start_terminal():
            self._ensure_network(log_key='log')
            exec_id = uuid.uuid4().hex
            container_name = f"{self._terminal_container_prefix}-{exec_id[:10]}"

            command_parts = [
                'docker', 'compose', 'run', '--rm', '--name', container_name,
                '--label', f'mobipick.exec={exec_id}',
                '--label', 'mobipick.role=terminal',
                '--label', 'mobipick.tab=terminal',
                *self._compose_env_args(),
                'mobipick_cmd', 'bash'
            ]
            command_str = self._fmt_args(command_parts)
            launcher = self._build_terminal_launcher(command_str)
            if not launcher:
                self._append_gui_html('log', '<i>Terminal launcher is not configured properly.</i>')
                self.set_terminal_visual('red', 'Open Terminal', True)
                return

            self._terminal_stopping = False
            self._terminal_running_cached = True
            self._terminal_container_name = container_name
            self._terminal_exec_id = exec_id

            proc = QProcess(self)
            proc.setProcessEnvironment(self._build_process_environment())
            proc.setWorkingDirectory(str(self._project_root))
            proc.finished.connect(self._on_terminal_proc_finished)
            proc.errorOccurred.connect(self._on_terminal_proc_error)
            self._terminal_proc = proc

            proc.start(launcher[0], launcher[1:])
            if not proc.waitForStarted(5000):
                self._append_gui_html('log', '<i>Failed to launch terminal application.</i>')
                self._terminal_running_cached = False
                self._terminal_proc = None
                proc.deleteLater()
                self._cleanup_terminal_container()
                self._terminal_container_name = None
                self._terminal_exec_id = None
                self.set_terminal_visual('red', 'Open Terminal', True)
                return

            self._start_terminal_stream(container_name, exec_id)
            self._append_gui_html('log', f'<i>Launching terminal: {html.escape(command_str)}</i>')
            self.set_terminal_visual('green', 'Close Terminal', True)

        self._ensure_roscore_ready(_start_terminal)

    def stop_terminal(self):
        if self._terminal_stopping and self._terminal_proc is None:
            self._finalize_terminal_stop()
            return
        if not self._terminal_is_active() and not self._terminal_container_name:
            self._terminal_running_cached = False
            self.set_terminal_visual('red', 'Open Terminal', True)
            return
        if not self._terminal_stopping:
            self._terminal_stopping = True
            self.set_terminal_visual('yellow', 'Closing Terminal...', False)

        if self._terminal_proc and self._terminal_proc.state() != QProcess.NotRunning:
            self._terminal_proc.terminate()
            QTimer.singleShot(2000, self._force_kill_terminal_proc)
        else:
            self._finalize_terminal_stop()

        self._cleanup_terminal_container()

    def _force_kill_terminal_proc(self):
        if self._terminal_proc and self._terminal_proc.state() != QProcess.NotRunning:
            self._terminal_proc.kill()

    def _finalize_terminal_stop(self):
        self._terminal_running_cached = False
        self._terminal_stopping = False
        self.set_terminal_visual('red', 'Open Terminal', True)

    def _terminal_is_active(self) -> bool:
        if self._terminal_proc and self._terminal_proc.state() != QProcess.NotRunning:
            return True
        return bool(self._terminal_running_cached)

    def _build_terminal_launcher(self, command: str) -> list[str] | None:
        template = (self._terminal_launcher_template or '').strip()
        if not template:
            return None
        try:
            formatted = template.format(title=self._terminal_title, command=command)
        except Exception as exc:
            self._append_gui_html('log', f'<i>Failed to format terminal launcher: {html.escape(str(exc))}</i>')
            return None
        try:
            parts = shlex.split(formatted)
        except ValueError as exc:
            self._append_gui_html('log', f'<i>Invalid terminal launcher command: {html.escape(str(exc))}</i>')
            return None
        if not parts:
            return None
        return parts

    def _on_terminal_proc_finished(self, exit_code: int, exit_status):
        if self._terminal_proc:
            self._terminal_proc.deleteLater()
        self._terminal_proc = None
        self._cleanup_terminal_container()
        self._finalize_terminal_stop()

    def _on_terminal_proc_error(self, _error):
        self._append_gui_html('log', '<i>Terminal launcher reported an error.</i>')
        if self._terminal_proc and self._terminal_proc.state() != QProcess.NotRunning:
            self._terminal_proc.kill()
        if self._terminal_proc:
            self._terminal_proc.deleteLater()
        self._terminal_proc = None
        self._cleanup_terminal_container()
        self._terminal_running_cached = False
        self._terminal_stopping = False
        self.set_terminal_visual('red', 'Open Terminal', True)

    def _cleanup_terminal_container(self):
        name = self._terminal_container_name
        if not name:
            return
        self._terminal_container_name = None
        self._terminal_exec_id = None
        try:
            subprocess.run(
                ['docker', 'stop', name],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                check=False,
            )
        except Exception:
            pass
        try:
            subprocess.run(
                ['docker', 'rm', name],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                check=False,
            )
        except Exception:
            pass

    def _start_terminal_stream(self, container_name: str, exec_id: str):
        self._terminal_stream_counter += 1
        key = f'terminal{self._terminal_stream_counter}'
        label = f'Terminal {self._terminal_stream_counter}'
        tab = self._ensure_tab(key, label, closable=True)
        tab.output.clear()
        tab.container_name = container_name
        tab.exec_id = exec_id
        script = (
            f'until docker logs -f {self._sh_quote(container_name)}; do '
            'sleep 0.2; '
            'done'
        )
        tab.start_program('bash', ['-lc', script])
        self._focus_tab(key)
        self._terminal_stream_tab_key = key

    def run_custom_command(self):
        text = self.command_input.text().strip()
        if not text:
            return

        def _run_command():
            self._log_info(f'running custom command: {text}')

            key_target = self._select_custom_tab_key()

            tab = self.tasks[key_target]
            exec_id = uuid.uuid4().hex
            tab.exec_id = exec_id
            tab.container_name = f'mpcmd-{exec_id[:10]}'
            wrapped = self._wrap_line_buffered(text)
            args = [
                'compose', 'run', '--rm', '--name', tab.container_name,
                '--label', f'mobipick.exec={exec_id}', '--label', f'mobipick.tab={key_target}',
                *self._compose_env_args(),
                'mobipick_cmd', 'bash', '-lc', wrapped
            ]
            tab.start_program('docker', args)
            self._focus_tab(key_target)
            self._update_stop_custom_enabled()

        self._ensure_roscore_ready(_run_command)

    # ---------- Interrupt current tab (ctrl+c / SIGINT) ----------

    def stop_custom_process(self):
        key = self._current_tab_key()
        if not key or not key.startswith('custom'):
            key = None
            for candidate, tab_obj in self.tasks.items():
                if candidate.startswith('custom') and tab_obj.is_running():
                    key = candidate
                    break
        if not key or key not in self.tasks:
            self._update_stop_custom_enabled()
            return
        tab = self.tasks[key]
        if not tab.is_running():
            self._update_stop_custom_enabled()
            return

        if key == self._script_active_tab_key:
            self.set_script_visual('yellow', 'Stopping Script...', False)

        self._stop_custom_tab(tab)

    def _stop_custom_tab(
        self,
        tab: ProcessTab,
        *,
        on_stopped: Callable[[], None] | None = None,
    ):
        pid = tab.pid()
        if pid:
            try:
                os.kill(pid, signal.SIGINT)
                self._append_gui_html(tab.key, '<i>Sent SIGINT to client (ctrl+c)...</i>')
                self._log_cmd(f'kill -SIGINT {pid}')
            except Exception as e:
                self._append_gui_html(tab.key, f'<i>Failed to SIGINT client: {html.escape(str(e))}</i>')

        container_name = tab.container_name

        exec_id = getattr(tab, 'exec_id', None)

        def finalize():
            if container_name:
                tab.container_name = None
            tab.exec_id = None
            if on_stopped:
                on_stopped()
            self._update_stop_custom_enabled()

        def _container_sigint_then_stop():
            if container_name or exec_id:
                self._graceful_stop_container(
                    container_name,
                    tab,
                    exec_id=exec_id,
                    on_finished=finalize,
                )
            else:
                finalize()

        QTimer.singleShot(int(self._timers_cfg['custom_tab_sigint_delay_ms']), _container_sigint_then_stop)

    # ---------- Output and search ----------

    def clear_current_tab(self):
        widget = self.tabs.currentWidget()
        if isinstance(widget, QTextEdit):
            widget.clear()
            key = self._current_tab_key()
            if key:
                self._last_log_origin.pop(key, None)

    def clear_all_tabs(self):
        for i in range(self.tabs.count()):
            widget = self.tabs.widget(i)
            if isinstance(widget, QTextEdit):
                widget.clear()
        self._last_log_origin.clear()

    def save_current_log(self):
        index = self.tabs.currentIndex()
        if index < 0:
            return
        widget = self.tabs.widget(index)
        if not isinstance(widget, QTextEdit):
            QMessageBox.information(self, 'Save Log', 'The current tab has no log to save.')
            return
        html = self._extract_widget_html(widget)
        if html is None:
            QMessageBox.information(self, 'Save Log', 'Nothing to save in the current tab.')
            return
        label = self.tabs.tabText(index).strip() or f'tab{index + 1}'
        default_name = self._suggest_log_filename(label)
        path, _ = QFileDialog.getSaveFileName(
            self,
            'Save Current Log',
            default_name,
            'HTML Files (*.html);;All Files (*)',
        )
        if not path:
            return
        path = self._ensure_html_extension(path)
        self._write_html_file(path, html)

    def load_log_file(self):
        path, _ = QFileDialog.getOpenFileName(
            self,
            'Load Log',
            '',
            'HTML Files (*.html);;All Files (*)',
        )
        if not path:
            return
        try:
            with open(path, 'r', encoding='utf-8') as f:
                html_content = f.read()
        except Exception as exc:
            QMessageBox.critical(self, 'Load Log', f'Failed to load log file:\n{exc}')
            return
        if not html_content.strip():
            QMessageBox.information(self, 'Load Log', 'The selected log file is empty.')
            return
        label = Path(path).stem or 'log'
        key = f'loadedlog_{uuid.uuid4().hex}'
        tab = self._ensure_tab(key, label, closable=True)
        widget = tab.output
        widget.clear()
        widget.setHtml(html_content)
        index = self.tabs.indexOf(widget)
        if index >= 0:
            self.tabs.setTabText(index, label)
        self._focus_tab(key)

    def save_all_logs(self):
        entries: list[tuple[str, str]] = []
        for i in range(self.tabs.count()):
            widget = self.tabs.widget(i)
            if not isinstance(widget, QTextEdit):
                continue
            html = self._extract_widget_html(widget)
            if html is None:
                continue
            label = self.tabs.tabText(i).strip() or f'tab{i + 1}'
            entries.append((label, html))
        if not entries:
            QMessageBox.information(self, 'Save Logs', 'There are no logs to save.')
            return
        directory = QFileDialog.getExistingDirectory(self, 'Select folder to save logs')
        if not directory:
            return
        os.makedirs(directory, exist_ok=True)
        for label, html in entries:
            filename = self._suggest_log_filename(label)
            path = self._resolve_unique_path(directory, filename)
            if not self._write_html_file(path, html):
                return
        QMessageBox.information(self, 'Save Logs', f'Saved {len(entries)} log file(s).')

    def _prepare_tab_for_origin(self, key: str, origin: str) -> ProcessTab:
        tab = self._ensure_tab(key, key.title(), closable=(key.startswith('custom')))
        last_origin = self._last_log_origin.get(key)
        if last_origin and last_origin != origin:
            tab.output.enqueue(True, '<br><br>')
        self._last_log_origin[key] = origin
        return tab

    def _append_html(self, key: str, html_text: str, *, gui: bool = False, color: str | None = None):
        origin = 'gui' if gui else 'container'
        tab = self._prepare_tab_for_origin(key, origin)
        if gui:
            color = html.escape(color or self._gui_log_color)
            tab.append_line_html(f'<span style="color:{color}">{html_text}</span>')
        else:
            tab.append_line_html(html_text)

    def _append_gui_html(self, key: str, html_text: str, *, color: str | None = None):
        self._append_html(key, html_text, gui=True, color=color)

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
        original_cursor = edit.textCursor()
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
        if not found:
            edit.setTextCursor(original_cursor)
        return found

    def find_next(self):
        edit = self._current_text_edit()
        if not edit:
            return
        pattern = self.search_input.text()
        if not pattern:
            return
        if not self._do_find(edit, pattern):
            QMessageBox.information(self, 'Search', f"No match for '{pattern}'")

    def find_prev(self):
        edit = self._current_text_edit()
        if not edit:
            return
        pattern = self.search_input.text()
        if not pattern:
            return
        if not self._do_find(edit, pattern, QTextDocument.FindBackward):
            QMessageBox.information(self, 'Search', f"No match for '{pattern}'")

    # ---------- Process completion callback ----------

    def on_task_finished(self, key: str, exit_code: int, exit_status):
        status_name = 'NormalExit' if int(exit_status) == int(QProcess.NormalExit) else 'Crashed'
        if key == self._terminal_stream_tab_key:
            self._terminal_stream_tab_key = None
        if key in self.tasks:
            self._append_gui_html(key, f'<i>Process finished with code {exit_code} [{status_name}]</i>')
            self.tasks[key].container_name = None
            self.tasks[key].exec_id = None
        if key == 'roscore':
            self._roscore_running_cached = False
            if self._roscore_stopping:
                return
            self._roscore_last_start_ts = None
            self.set_roscore_visual('red', 'Start Roscore', enabled=True)
            self._sim_running_cached = False
            self.set_toggle_visual('red', 'Start Sim', enabled=True)
            self.set_tables_visual('red', 'Run Tables Demo', True)
            self.set_rviz_visual('red', 'Start RViz', True)
            self.set_rqt_visual('red', 'Start RQt Tables', True)
            self.stop_terminal()
            self._script_active_tab_key = None
            self.set_script_visual('red', 'Run Script', bool(self._script_choices))
            self._update_stop_custom_enabled()
            return
        if key == 'sim':
            self._revoke_x()
            self._sim_running_cached = False
            if self._killing:
                return
            self.set_toggle_visual('red', 'Start Sim', enabled=True)
            return
        if key == 'tables':
            if self._roscore_stopping or self._toggle_states.get('tables') == 'yellow':
                return
            self.set_tables_visual('red', 'Run Tables Demo', True)
            return
        if key == 'rviz':
            if self._roscore_stopping or self._toggle_states.get('rviz') == 'yellow':
                return
            self.set_rviz_visual('red', 'Start RViz', True)
            return
        if key == 'rqt':
            if self._roscore_stopping or self._toggle_states.get('rqt') == 'yellow':
                return
            self.set_rqt_visual('red', 'Start RQt Tables', True)
            return
        if key == self._script_active_tab_key:
            if self._toggle_states.get('script') == 'yellow':
                return
            self._script_active_tab_key = None
            if self._roscore_stopping:
                self.set_script_visual('yellow', 'Shutting down...', False)
            else:
                self.set_script_visual('red', 'Run Script', bool(self._script_choices))
            self._update_stop_custom_enabled()
            return

        if key.startswith('custom'):
            self._update_stop_custom_enabled()

    # ---------- Poll and initial states ----------

    def _poll(self):
        self._update_stop_custom_enabled()

    def _check_sigint(self):
        global _SIGINT_TRIGGERED
        if not _SIGINT_TRIGGERED:
            return
        _SIGINT_TRIGGERED = False
        self._begin_exit_sequence()

    def _begin_exit_sequence(self):
        if self._exit_in_progress:
            return
        self._exit_in_progress = True
        exit_cfg = CONFIG['exit']
        self._console_log(1, exit_cfg['log_start_message'])
        self.hide()
        self._exit_dialog = QMessageBox()
        self._exit_dialog.setWindowTitle(exit_cfg['dialog_title'])
        self._exit_dialog.setText(exit_cfg['dialog_message'])
        self._exit_dialog.setIcon(QMessageBox.Information)
        self._exit_dialog.setStandardButtons(QMessageBox.NoButton)
        self._exit_dialog.setWindowModality(Qt.ApplicationModal)
        self._exit_dialog.setWindowFlag(Qt.WindowStaysOnTopHint, True)
        self._exit_dialog.show()
        QTimer.singleShot(0, self._perform_exit_cleanup)

    def _perform_exit_cleanup(self):
        self.stop_terminal()

        for proc in list(self._bg_procs):
            try:
                proc.kill()
            except Exception:
                pass
            proc.deleteLater()
        self._bg_procs.clear()

        for p in list(self.tasks.values()):
            if p.is_running():
                try:
                    p.kill()
                except Exception:
                    pass

        commands = self._collect_exit_commands()
        if commands:
            self._run_command_sequence(commands, on_finished=self._finalize_exit, log_key='log')
        else:
            self._finalize_exit()

    def _finalize_exit(self):
        if self._exit_dialog:
            self._exit_dialog.close()
            self._exit_dialog = None

        self._revoke_x()

        if not self._cleanup_done and not self._cleanup_script_available():
            self._cleanup_done = True

        self._console_log(1, CONFIG['exit']['log_done_message'])

        app = QApplication.instance()
        if app:
            QTimer.singleShot(0, app.quit)

    def _update_buttons(self):
        self.set_roscore_visual('red', 'Start Roscore', True)
        self.set_toggle_visual('red', 'Start Sim', True)
        self.set_tables_visual('red', 'Run Tables Demo', True)
        self.set_rviz_visual('red', 'Start RViz', True)
        self.set_rqt_visual('red', 'Start RQt Tables', True)
        self.set_script_visual('red', 'Run Script', bool(self._script_choices))
        self.set_terminal_visual('red', 'Open Terminal', True)
        self.stop_custom_button.setEnabled(False)
        self._update_stop_custom_enabled()

    # ---------- Close ----------

    def closeEvent(self, event):
        if self._exit_in_progress:
            event.ignore()
            return
        event.ignore()
        self._begin_exit_sequence()


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Mobipick Labs Control GUI')
    parser.add_argument(
        '-v', '--v', '--verbose',
        dest='verbosity',
        nargs='?',
        const=3,
        default=1,
        type=int,
        choices=[1, 2, 3],
        help='Verbosity level (1=min, 3=max). If no value provided defaults to 3.'
    )

    parsed_args, qt_args = parser.parse_known_args()
    verbosity = parsed_args.verbosity or 1

    app = QApplication([sys.argv[0]] + qt_args)
    w = MainWindow(verbosity=verbosity)
    w.show()

    def _handle_sigint(_sig, _frame):
        global _SIGINT_TRIGGERED
        _SIGINT_TRIGGERED = True

    signal.signal(signal.SIGINT, _handle_sigint)

    sys.exit(app.exec_())
