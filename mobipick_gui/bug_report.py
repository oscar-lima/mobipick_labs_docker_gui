"""Bug report dialog and text rendering helpers."""
from __future__ import annotations

from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Callable
from urllib.parse import quote

from PyQt5.QtCore import QProcess, QTimer, Qt, QUrl
from PyQt5.QtGui import QFont
from PyQt5.QtWidgets import (
    QApplication,
    QCheckBox,
    QFileDialog,
    QHBoxLayout,
    QLabel,
    QMessageBox,
    QPushButton,
    QSizePolicy,
    QTextEdit,
    QVBoxLayout,
    QWidget,
)

from .external_links import open_external_url
from .version import get_version
from .window_utils import MaximizableDialog as QDialog

BUG_REPORT_EMAIL = 'mobipick-labs@dfki.de'
BUG_REPORT_GITHUB_ISSUE_URL = (
    'https://github.com/oscar-lima/mobipick_labs_docker_gui/issues/new'
)


@dataclass(frozen=True)
class BugReportSection:
    """One optional section in the bug report preview."""

    key: str
    label: str
    default_checked: bool = False


BUG_REPORT_SECTIONS: tuple[BugReportSection, ...] = (
    BugReportSection('ubuntu_version', 'Ubuntu version'),
    BugReportSection('nvidia_smi', 'nvidia-smi output'),
    BugReportSection('gui_version', 'GUI version', True),
    BugReportSection('docker_images', 'Mobipick Docker images'),
    BugReportSection('selected_workspace', 'Selected ROS 1 workspace', True),
    BugReportSection('selected_image_match', 'Selected image/workspace match', True),
    BugReportSection('workspace_graph', 'Available workspaces graph'),
    BugReportSection('setup_diagnostics', 'Setup checks'),
    BugReportSection('log_tab', 'Log tab'),
    BugReportSection('user_notes', 'User input'),
)

COMMAND_SECTION_KEYS = {
    'ubuntu_version',
    'nvidia_smi',
    'docker_images',
}


def default_report_context() -> dict:
    """Return context values that do not depend on the main window state."""
    return {
        'generated_at': datetime.now().astimezone().isoformat(timespec='seconds'),
        'gui_version': get_version(),
    }


def filter_mobipick_docker_images(raw_output: str) -> str:
    """Return only docker image lines that contain the word mobipick."""
    lines = [
        line.strip()
        for line in (raw_output or '').splitlines()
        if 'mobipick' in line.lower()
    ]
    if not lines:
        return '(no docker images containing "mobipick" were listed)'
    return '\n'.join(lines)


def workspace_graph_ascii(workspaces: list[dict]) -> str:
    """Render workspace inheritance as ASCII parent -> child edges."""
    if not workspaces:
        return '(no ROS 1 workspaces configured)'

    names = {str(item.get('name') or '') for item in workspaces}
    edges: list[str] = []
    roots: list[str] = []
    for item in workspaces:
        name = str(item.get('name') or '').strip()
        if not name:
            continue
        parents = [
            str(parent).strip()
            for parent in item.get('extends', []) or []
            if str(parent).strip()
        ]
        if not parents:
            roots.append(name)
        for parent in parents:
            marker = '' if parent in names else ' (missing parent)'
            edges.append(f'{parent} -> {name}{marker}')

    lines: list[str] = []
    if roots:
        lines.append('Roots:')
        lines.extend(f'  {name}' for name in sorted(roots, key=str.lower))
    if edges:
        if lines:
            lines.append('')
        lines.append('Graph (parent -> child, child extends parent):')
        lines.extend(f'  {edge}' for edge in sorted(edges, key=str.lower))
    else:
        if lines:
            lines.append('')
        lines.append('(no workspace inheritance edges)')
    return '\n'.join(lines)


def _format_key_values(values: dict[str, object]) -> str:
    lines: list[str] = []
    for key, value in values.items():
        if value in (None, '', []):
            value = '(none)'
        if isinstance(value, list):
            value = ', '.join(str(item) for item in value) or '(none)'
        lines.append(f'{key}: {value}')
    return '\n'.join(lines)


def _format_selected_workspace(context: dict) -> str:
    workspace = context.get('selected_workspace') or {}
    if not isinstance(workspace, dict) or not workspace.get('name'):
        return 'Docker image default (no host ROS 1 workspace selected)'
    return _format_key_values(
        {
            'name': workspace.get('name'),
            'path': workspace.get('path'),
            'extends': workspace.get('extends') or [],
            'configured image': workspace.get('image'),
            'runtime built': workspace.get('runtime_built'),
            'registry active': workspace.get('active'),
        }
    )


def _format_selected_image_match(context: dict) -> str:
    return _format_key_values(
        {
            'selected Docker image': context.get('selected_image'),
            'active workspace': context.get('active_workspace_name')
            or 'Docker image default',
            'workspace match': context.get('workspace_match'),
            'host workspace mount': context.get('host_workspace_mount'),
        }
    )


def _format_workspace_graph(context: dict) -> str:
    workspaces = context.get('workspaces') or []
    registry_path = context.get('workspace_registry_path') or '(unknown)'
    master_folder = context.get('workspace_master_folder') or '(none)'
    lines = [
        f'Registry: {registry_path}',
        f'Master folder: {master_folder}',
        '',
        'Workspaces:',
    ]
    if not workspaces:
        lines.append('  (none)')
    else:
        for item in workspaces:
            marker = ' [active]' if item.get('active') else ''
            lines.append(f"  {item.get('name')}{marker}")
            lines.append(f"    path: {item.get('path') or '(none)'}")
            lines.append(
                '    extends: '
                + (', '.join(item.get('extends') or []) or '(none)')
            )
            lines.append(f"    image: {item.get('image') or '(none)'}")
            lines.append(f"    runtime built: {item.get('runtime_built')}")
    lines.extend(['', workspace_graph_ascii(workspaces)])
    return '\n'.join(lines)


def format_bug_report(
    context: dict,
    included_keys: set[str],
    command_outputs: dict[str, str],
    user_notes: str,
) -> str:
    """Return the report text for the selected sections."""
    base_context = default_report_context()
    merged = {**base_context, **context}
    lines = [
        'Mobipick Labs Docker GUI Bug Report',
        f"Generated: {merged.get('generated_at')}",
        '',
    ]

    def add_section(title: str, body: str) -> None:
        lines.extend([f'## {title}', body.strip() or '(empty)', ''])

    if 'ubuntu_version' in included_keys:
        add_section('Ubuntu Version', command_outputs.get('ubuntu_version', 'Collecting...'))
    if 'nvidia_smi' in included_keys:
        add_section('nvidia-smi Output', command_outputs.get('nvidia_smi', 'Collecting...'))
    if 'gui_version' in included_keys:
        add_section('GUI Version', str(merged.get('gui_version') or get_version()))
    if 'docker_images' in included_keys:
        add_section(
            'Docker Images Containing mobipick',
            command_outputs.get('docker_images', 'Collecting...'),
        )
    if 'selected_workspace' in included_keys:
        add_section('Selected ROS 1 Workspace', _format_selected_workspace(merged))
    if 'selected_image_match' in included_keys:
        add_section('Selected Docker Image / Workspace Match', _format_selected_image_match(merged))
    if 'workspace_graph' in included_keys:
        add_section('Available ROS 1 Workspaces', _format_workspace_graph(merged))
    if 'setup_diagnostics' in included_keys:
        add_section(
            'Setup Checks',
            str(merged.get('setup_diagnostics') or '(No setup diagnostics)'),
        )
    if 'log_tab' in included_keys:
        add_section('Log Tab', str(merged.get('log_tab_text') or '(Log tab is empty)'))
    if 'user_notes' in included_keys:
        add_section('User Input', user_notes or '(not provided)')

    return '\n'.join(lines).rstrip() + '\n'


class BugReportDialog(QDialog):
    """Dialog that collects diagnostics and previews a bug report."""

    def __init__(
        self,
        context_provider: Callable[[], dict],
        parent: QWidget | None = None,
        *,
        initial_notes: str = '',
        initial_checked_keys: set[str] | None = None,
    ):
        super().__init__(parent)
        self.setWindowTitle('Bug Report')
        self.setWindowModality(Qt.NonModal)
        self.resize(980, 680)

        self._context_provider = context_provider
        self._context = context_provider()
        self._command_outputs = {
            key: 'Collecting...'
            for key in COMMAND_SECTION_KEYS
        }
        self._processes: dict[str, QProcess] = {}
        self._checkboxes: dict[str, QCheckBox] = {}
        initial_checked_keys = set(initial_checked_keys or set())

        root = QHBoxLayout(self)
        root.setSpacing(12)

        left = QWidget()
        left.setMinimumWidth(260)
        left.setMaximumWidth(360)
        left_layout = QVBoxLayout(left)
        left_layout.addWidget(QLabel('Include in report'))

        for section in BUG_REPORT_SECTIONS:
            checkbox = QCheckBox(section.label)
            checkbox.setChecked(
                section.default_checked
                or section.key in initial_checked_keys
            )
            checkbox.toggled.connect(self._update_preview)
            self._checkboxes[section.key] = checkbox
            left_layout.addWidget(checkbox)

        check_all_button = QPushButton('Check All')
        check_all_button.clicked.connect(self._check_all)
        left_layout.addWidget(check_all_button)

        left_layout.addWidget(QLabel('Error description'))
        self.notes_edit = QTextEdit()
        self.notes_edit.setPlaceholderText('Describe what happened and how to reproduce it.')
        self.notes_edit.setAcceptRichText(False)
        self.notes_edit.setMinimumHeight(130)
        self.notes_edit.textChanged.connect(self._on_notes_changed)
        left_layout.addWidget(self.notes_edit)
        if initial_notes:
            self.notes_edit.blockSignals(True)
            self.notes_edit.setPlainText(initial_notes)
            self.notes_edit.blockSignals(False)
            notes_checkbox = self._checkboxes.get('user_notes')
            if notes_checkbox:
                notes_checkbox.setChecked(True)
        left_layout.addStretch(1)

        right = QWidget()
        right_layout = QVBoxLayout(right)
        preview_label = QLabel('Report preview')
        right_layout.addWidget(preview_label)

        self.preview_edit = QTextEdit()
        self.preview_edit.setReadOnly(True)
        self.preview_edit.setAcceptRichText(False)
        self.preview_edit.setLineWrapMode(QTextEdit.NoWrap)
        monospace = QFont('monospace')
        monospace.setStyleHint(QFont.Monospace)
        self.preview_edit.setFont(monospace)
        self.preview_edit.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        right_layout.addWidget(self.preview_edit, 1)

        button_row = QHBoxLayout()
        self.status_label = QLabel('')
        button_row.addWidget(self.status_label, 1)

        refresh_button = QPushButton('Refresh')
        refresh_button.clicked.connect(self.refresh)
        button_row.addWidget(refresh_button)

        copy_button = QPushButton('Copy')
        copy_button.clicked.connect(self.copy_to_clipboard)
        button_row.addWidget(copy_button)
        self._copy_button = copy_button

        save_button = QPushButton('Save')
        save_button.clicked.connect(self.save_to_file)
        button_row.addWidget(save_button)

        mail_button = QPushButton('Email')
        mail_button.clicked.connect(self.open_email)
        button_row.addWidget(mail_button)

        github_button = QPushButton('GitHub Issue')
        github_button.clicked.connect(self.open_github_issue)
        button_row.addWidget(github_button)

        close_button = QPushButton('Close')
        close_button.clicked.connect(self.close)
        button_row.addWidget(close_button)

        right_layout.addLayout(button_row)

        root.addWidget(left)
        root.addWidget(right, 1)

        self._update_preview()
        self._start_collectors()

    def report_text(self) -> str:
        """Return the current report preview text."""
        return format_bug_report(
            self._context,
            self._included_keys(),
            self._command_outputs,
            self.notes_edit.toPlainText(),
        )

    def refresh(self) -> None:
        """Refresh main-window state and rerun command collectors."""
        self._context = self._context_provider()
        self._command_outputs.update(
            {key: 'Collecting...' for key in COMMAND_SECTION_KEYS}
        )
        self._update_preview()
        self._start_collectors()

    def copy_to_clipboard(self) -> None:
        QApplication.clipboard().setText(self.report_text())
        self.status_label.setText('Copied to clipboard.')
        self._copy_button.setText('Copied')
        QTimer.singleShot(1500, lambda: self._copy_button.setText('Copy'))

    def save_to_file(self) -> None:
        default_name = (
            'mobipick_bug_report_'
            + datetime.now().strftime('%Y%m%d_%H%M%S')
            + '.txt'
        )
        path, _ = QFileDialog.getSaveFileName(
            self,
            'Save Bug Report',
            default_name,
            'Text Files (*.txt);;Markdown Files (*.md);;All Files (*)',
        )
        if not path:
            return
        try:
            Path(path).write_text(self.report_text(), encoding='utf-8')
        except OSError as exc:
            QMessageBox.critical(self, 'Bug Report', f'Failed to save report:\n{exc}')
            return
        self.status_label.setText(f'Saved to {path}')

    def open_email(self) -> None:
        subject = 'Mobipick Labs Docker GUI bug report'
        body = self.report_text()
        url = QUrl(
            f'mailto:{BUG_REPORT_EMAIL}'
            f'?subject={quote(subject, safe="")}'
            f'&body={quote(body, safe="")}'
        )
        if not open_external_url(url):
            QMessageBox.warning(
                self,
                'Bug Report',
                'Unable to open the default email application.',
            )

    def open_github_issue(self) -> None:
        subject = 'Mobipick Labs Docker GUI bug report'
        body = self.report_text()
        url = QUrl(
            f'{BUG_REPORT_GITHUB_ISSUE_URL}'
            f'?title={quote(subject, safe="")}'
            f'&body={quote(body, safe="")}'
        )
        if not open_external_url(url):
            QMessageBox.warning(
                self,
                'Bug Report',
                'Unable to open the GitHub issue page.',
            )

    def _included_keys(self) -> set[str]:
        return {
            key
            for key, checkbox in self._checkboxes.items()
            if checkbox.isChecked()
        }

    def _check_all(self) -> None:
        for checkbox in self._checkboxes.values():
            checkbox.setChecked(True)
        self._update_preview()

    def _on_notes_changed(self) -> None:
        notes_checkbox = self._checkboxes.get('user_notes')
        if notes_checkbox and self.notes_edit.toPlainText().strip():
            notes_checkbox.setChecked(True)
        self._update_preview()

    def _update_preview(self) -> None:
        self.preview_edit.setPlainText(self.report_text())

    def _start_collectors(self) -> None:
        for process in self._processes.values():
            if process.state() != QProcess.NotRunning:
                process.kill()
        self._processes.clear()
        self._run_capture(
            'ubuntu_version',
            'bash',
            [
                '-lc',
                (
                    'if command -v lsb_release >/dev/null 2>&1; then '
                    'lsb_release -ds; '
                    'elif [ -r /etc/os-release ]; then '
                    '. /etc/os-release; printf "%s\\n" "${PRETTY_NAME:-unknown}"; '
                    'else uname -a; fi'
                ),
            ],
        )
        self._run_capture('nvidia_smi', 'nvidia-smi', [])
        self._run_capture(
            'docker_images',
            'docker',
            [
                'images',
                '--format',
                '{{.Repository}}:{{.Tag}}\t{{.ID}}\t{{.CreatedSince}}\t{{.Size}}',
            ],
            transform=filter_mobipick_docker_images,
        )

    def _run_capture(
        self,
        key: str,
        program: str,
        args: list[str],
        *,
        transform: Callable[[str], str] | None = None,
    ) -> None:
        process = QProcess(self)
        process.setProgram(program)
        process.setArguments(args)
        process.finished.connect(
            lambda exit_code, _status, k=key, p=process, tr=transform: (
                self._on_process_finished(k, p, exit_code, tr)
            )
        )
        process.errorOccurred.connect(
            lambda _error, k=key, p=process: self._on_process_error(k, p)
        )
        self._processes[key] = process
        process.start()

    def _on_process_finished(
        self,
        key: str,
        process: QProcess,
        exit_code: int,
        transform: Callable[[str], str] | None,
    ) -> None:
        if self._processes.get(key) is not process:
            return
        stdout = bytes(process.readAllStandardOutput()).decode(errors='replace')
        stderr = bytes(process.readAllStandardError()).decode(errors='replace')
        if exit_code == 0:
            text = stdout.strip() or '(command produced no output)'
            if transform:
                text = transform(text)
        else:
            details = (stderr or stdout).strip()
            text = f'command failed with exit code {exit_code}'
            if details:
                text += f':\n{details}'
        self._command_outputs[key] = text
        self._processes.pop(key, None)
        self._update_preview()

    def _on_process_error(self, key: str, process: QProcess) -> None:
        if self._processes.get(key) is not process:
            return
        program = process.program() or key
        self._command_outputs[key] = f'command could not be started: {program}'
        self._processes.pop(key, None)
        self._update_preview()


__all__ = [
    'BUG_REPORT_EMAIL',
    'BUG_REPORT_GITHUB_ISSUE_URL',
    'BUG_REPORT_SECTIONS',
    'BugReportDialog',
    'BugReportSection',
    'default_report_context',
    'filter_mobipick_docker_images',
    'format_bug_report',
    'workspace_graph_ascii',
]
