from __future__ import annotations

import copy
import html
import json
import os
import re
import shlex
import signal
import subprocess
import sys
import time
import uuid
from collections import deque
from datetime import datetime
from fnmatch import fnmatchcase
from pathlib import Path
from typing import Callable, Match, Optional
from urllib.parse import urlsplit

import yaml
from PyQt5.QtCore import QEvent, QIODevice, QPoint, QProcess, QProcessEnvironment, QTimer, Qt
from PyQt5.QtGui import QColor, QGuiApplication, QPixmap, QTextCursor, QTextDocument
from PyQt5.QtWidgets import (
    QAction,
    QAbstractItemView,
    QApplication,
    QCheckBox,
    QComboBox,
    QDialog,
    QDialogButtonBox,
    QDoubleSpinBox,
    QFileDialog,
    QFormLayout,
    QHBoxLayout,
    QHeaderView,
    QLabel,
    QLineEdit,
    QListWidget,
    QListWidgetItem,
    QMainWindow,
    QMenu,
    QMessageBox,
    QPushButton,
    QScrollArea,
    QSizePolicy,
    QTabBar,
    QTableWidget,
    QTableWidgetItem,
    QTabWidget,
    QTextEdit,
    QToolTip,
    QVBoxLayout,
    QWidget,
)

from .ansi import CSI_SEQ_RE, OSC_SEQ_RE, ansi_to_html
from .bug_report import BugReportDialog
from .config import (
    BUTTON_CONFIG_FILE,
    CONFIG,
    DEFAULT_BUTTON_COMMANDS,
    DEFAULT_YAML_PATH,
    PROJECT_ROOT,
    SCRIPT_CLEAN,
    USER_DATA_DIR,
    USER_CONFIG_FILE,
    WINDOW_LAYOUT_FILE,
    load_docker_cp_config,
    load_docker_cp_user_config,
    load_button_layout,
    load_launch_sequence_plan,
    save_button_layout,
    save_docker_cp_config,
    save_launch_sequence_plan,
    save_user_config_update,
    user_configuration_paths,
    writable_button_config_path,
    writable_docker_cp_config_path,
    writable_launch_sequence_path,
    writable_workspace_button_config_path,
    writable_workspace_docker_cp_config_path,
)
from .documentation_dialog import DocumentationDialog
from .external_links import open_external_url
from .settings_transfer import export_settings, import_settings

CONTAINER_SCRIPTS_DIR = str(
    CONFIG.get('process', {}).get('container_scripts_dir', '/scripts_430ofkjl04fsw')
)
from .process_tab import ProcessTab
from .setup_wizard import ImageSetupWizard, SetupWizardSelection
from .version import get_version
from .window_layout import WindowLayoutManager
from .workspace_dialog import WorkspaceManagerDialog
from .workspaces import RosWorkspace, WorkspaceRegistry

_SIGINT_TRIGGERED = False


def trigger_sigint():
    """Signal the GUI to start its shutdown sequence."""
    global _SIGINT_TRIGGERED
    _SIGINT_TRIGGERED = True


def _text_width(font_metrics, text: str) -> int:
    """Return the display width for text across supported PyQt5 versions."""
    if hasattr(font_metrics, 'horizontalAdvance'):
        return font_metrics.horizontalAdvance(text)
    return font_metrics.width(text)


def _configure_expanding_toolbar_button(button: QPushButton) -> None:
    """Let toolbar buttons grow with the window while keeping text readable."""
    button.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Preferred)


class ButtonProfileTable(QTableWidget):
    """Toolbar button editor table with priority-based column sizing."""

    FIELD_ORDER = ['key', 'label', 'command', 'service', 'tooltip']
    MIN_WIDTHS = {
        'key': 96,
        'label': 132,
        'command': 280,
        'service': 112,
        'tooltip': 72,
    }
    MAX_DESIRED_WIDTHS = {
        'key': 220,
        'label': 280,
        'command': 760,
        'service': 180,
        'tooltip': 240,
    }
    EXPAND_WEIGHTS = {
        'key': 1,
        'label': 2,
        'command': 7,
        'service': 1,
        'tooltip': 0,
    }

    def __init__(
        self,
        columns: list[tuple[str, str]],
        parent: QWidget | None = None,
    ):
        super().__init__(0, len(columns), parent)
        self._column_fields = [field for field, _label in columns]
        self.setHorizontalHeaderLabels([label for _field, label in columns])
        self.setHorizontalScrollMode(QAbstractItemView.ScrollPerPixel)
        header = self.horizontalHeader()
        header.setStretchLastSection(False)
        header.setSectionResizeMode(QHeaderView.Fixed)
        header.setMinimumSectionSize(48)

    def resizeEvent(self, event):  # noqa: N802 - Qt API
        super().resizeEvent(event)
        self.apply_column_widths()

    def apply_column_widths(self) -> None:
        if self.columnCount() <= 0:
            return
        widths = self._responsive_column_widths()
        for column, width in enumerate(widths):
            self.setColumnWidth(column, width)

    def _responsive_column_widths(self) -> list[int]:
        desired = self._desired_column_widths()
        minimum = [
            min(desired[index], self.MIN_WIDTHS.get(field, 90))
            for index, field in enumerate(self._column_fields)
        ]
        available = max(self.viewport().width(), sum(minimum))
        widths = list(minimum)
        remaining = available - sum(widths)

        for field in self.FIELD_ORDER:
            if remaining <= 0:
                break
            if field not in self._column_fields:
                continue
            index = self._column_fields.index(field)
            growth = max(0, desired[index] - widths[index])
            add = min(growth, remaining)
            widths[index] += add
            remaining -= add

        if remaining > 0:
            weighted_fields = [
                field
                for field in self.FIELD_ORDER
                if field in self._column_fields
                and self.EXPAND_WEIGHTS.get(field, 0) > 0
            ]
            total_weight = sum(
                self.EXPAND_WEIGHTS[field]
                for field in weighted_fields
            )
            for field in weighted_fields:
                index = self._column_fields.index(field)
                add = remaining * self.EXPAND_WEIGHTS[field] // total_weight
                widths[index] += add
            used = sum(widths)
            if used < available:
                widths[self._column_fields.index('command')] += available - used
        return widths

    def _desired_column_widths(self) -> list[int]:
        metrics = self.fontMetrics()
        widths: list[int] = []
        for column, field in enumerate(self._column_fields):
            header = self.horizontalHeaderItem(column)
            header_text = header.text() if header else field
            content_width = _text_width(metrics, header_text)
            for row in range(self.rowCount()):
                item = self.item(row, column)
                if item:
                    content_width = max(
                        content_width,
                        _text_width(metrics, item.text()),
                    )
            desired = content_width + 32
            widths.append(
                min(
                    max(desired, self.MIN_WIDTHS.get(field, 90)),
                    self.MAX_DESIRED_WIDTHS.get(field, 360),
                )
            )
        return widths


class ImageBlacklistDialog(QDialog):
    """Edit Docker image ignore patterns with a live result preview."""

    def __init__(
        self,
        patterns: list[str],
        discovery_filters: list[str],
        image_refs: list[str],
        matcher: Callable[[str, str], bool],
        parent: QWidget | None = None,
    ):
        super().__init__(parent)
        self._matcher = matcher
        self._image_refs = list(dict.fromkeys(ref for ref in image_refs if ref))

        self.setWindowTitle('Docker Image Filters')
        self.resize(1040, 640)

        layout = QVBoxLayout(self)
        intro = QLabel(
            'Discovery filters choose which local images are offered by the '
            'GUI. Ignore patterns then remove unwanted matches.'
        )
        intro.setWordWrap(True)
        layout.addWidget(intro)

        body = QHBoxLayout()
        layout.addLayout(body, 1)

        editor_column = QVBoxLayout()
        body.addLayout(editor_column, 1)

        editor_column.addWidget(QLabel('Discovery filters'))
        self.discovery_filter_editor = QTextEdit()
        self.discovery_filter_editor.setAcceptRichText(False)
        self.discovery_filter_editor.setPlainText('\n'.join(discovery_filters))
        self.discovery_filter_editor.setPlaceholderText(
            'mobipick\nx_mobipick_labs'
        )
        self.discovery_filter_editor.setMinimumWidth(320)
        self.discovery_filter_editor.setMaximumHeight(140)
        self.discovery_filter_editor.setSizePolicy(
            QSizePolicy.Expanding,
            QSizePolicy.Fixed,
        )
        editor_column.addWidget(self.discovery_filter_editor)

        filter_actions = QHBoxLayout()
        add_filter = QPushButton('Add Repository Filter')
        add_filter.clicked.connect(self._add_selected_repository_filter)
        filter_actions.addWidget(add_filter)
        filter_actions.addStretch(1)
        editor_column.addLayout(filter_actions)

        editor_column.addWidget(QLabel('Ignored refs and patterns'))
        self.pattern_editor = QTextEdit()
        self.pattern_editor.setAcceptRichText(False)
        self.pattern_editor.setPlainText('\n'.join(patterns))
        self.pattern_editor.setPlaceholderText('*n8n*\nrepo/image:tag')
        self.pattern_editor.setMinimumWidth(320)
        self.pattern_editor.setSizePolicy(
            QSizePolicy.Expanding,
            QSizePolicy.Expanding,
        )
        editor_column.addWidget(self.pattern_editor, 1)

        image_row = QHBoxLayout()
        self.image_combo = QComboBox()
        self.image_combo.setMinimumWidth(280)
        self.image_combo.setSizePolicy(
            QSizePolicy.Expanding,
            QSizePolicy.Fixed,
        )
        if self._image_refs:
            self.image_combo.addItems(self._image_refs)
        else:
            self.image_combo.addItem('No matching local Docker images found')
            self.image_combo.setEnabled(False)
        image_row.addWidget(self.image_combo, 1)

        add_exact = QPushButton('Add Image')
        add_exact.clicked.connect(self._add_selected_image)
        image_row.addWidget(add_exact)
        editor_column.addLayout(image_row)

        add_repo = QPushButton('Add Repository Pattern')
        add_repo.clicked.connect(self._add_selected_repository_pattern)
        editor_column.addWidget(add_repo)

        preview_column = QVBoxLayout()
        body.addLayout(preview_column, 2)

        self.summary_label = QLabel()
        self.summary_label.setWordWrap(True)
        preview_column.addWidget(self.summary_label)

        self.preview_table = QTableWidget(0, 4)
        self.preview_table.setHorizontalHeaderLabels(
            ['Result', 'Image', 'Discovery filter', 'Ignore pattern']
        )
        self.preview_table.setEditTriggers(QAbstractItemView.NoEditTriggers)
        self.preview_table.setSelectionBehavior(QAbstractItemView.SelectRows)
        self.preview_table.verticalHeader().setVisible(False)
        header = self.preview_table.horizontalHeader()
        header.setSectionResizeMode(0, QHeaderView.ResizeToContents)
        header.setSectionResizeMode(1, QHeaderView.Stretch)
        header.setSectionResizeMode(2, QHeaderView.ResizeToContents)
        header.setSectionResizeMode(3, QHeaderView.ResizeToContents)
        preview_column.addWidget(self.preview_table, 1)

        buttons = QDialogButtonBox(
            QDialogButtonBox.Ok | QDialogButtonBox.Cancel
        )
        buttons.accepted.connect(self.accept)
        buttons.rejected.connect(self.reject)
        layout.addWidget(buttons)

        self.pattern_editor.textChanged.connect(self._refresh_preview)
        self.discovery_filter_editor.textChanged.connect(self._refresh_preview)
        self._refresh_preview()

    def patterns(self) -> list[str]:
        return self._normalize_patterns(self.pattern_editor.toPlainText())

    def discovery_filters(self) -> list[str]:
        return self._normalize_patterns(
            self.discovery_filter_editor.toPlainText()
        )

    @staticmethod
    def _normalize_patterns(text: str) -> list[str]:
        patterns: list[str] = []
        for item in re.split(r'[\n,]+', text or ''):
            pattern = item.strip()
            if pattern and pattern not in patterns:
                patterns.append(pattern)
        return patterns

    def _selected_image_ref(self) -> str:
        if not self.image_combo.isEnabled():
            return ''
        return str(self.image_combo.currentText() or '').strip()

    def _add_selected_image(self) -> None:
        self._append_pattern(self._selected_image_ref())

    def _add_selected_repository_pattern(self) -> None:
        self._append_pattern(self._repository_pattern_for_selected_image())

    def _add_selected_repository_filter(self) -> None:
        self._append_discovery_filter(
            self._repository_filter_for_selected_image()
        )

    def _repository_pattern_for_selected_image(self) -> str:
        image_ref = self._selected_image_ref()
        if not image_ref:
            return ''
        repository = self._repository_for_image_ref(image_ref)
        return f'{repository}:*' if repository else ''

    def _repository_filter_for_selected_image(self) -> str:
        return self._repository_for_image_ref(self._selected_image_ref())

    @staticmethod
    def _repository_for_image_ref(image_ref: str) -> str:
        last_slash = image_ref.rfind('/')
        last_colon = image_ref.rfind(':')
        return (
            image_ref[:last_colon]
            if last_colon > last_slash
            else image_ref
        )

    def _append_pattern(self, pattern: str) -> None:
        pattern = str(pattern or '').strip()
        if not pattern:
            return
        patterns = self.patterns()
        if pattern not in patterns:
            patterns.append(pattern)
        self.pattern_editor.setPlainText('\n'.join(patterns))
        cursor = self.pattern_editor.textCursor()
        cursor.movePosition(QTextCursor.End)
        self.pattern_editor.setTextCursor(cursor)

    def _append_discovery_filter(self, discovery_filter: str) -> None:
        discovery_filter = str(discovery_filter or '').strip()
        if not discovery_filter:
            return
        filters = self.discovery_filters()
        if discovery_filter not in filters:
            filters.append(discovery_filter)
        self.discovery_filter_editor.setPlainText('\n'.join(filters))
        cursor = self.discovery_filter_editor.textCursor()
        cursor.movePosition(QTextCursor.End)
        self.discovery_filter_editor.setTextCursor(cursor)

    def _refresh_preview(self) -> None:
        patterns = self.patterns()
        discovery_filters = self.discovery_filters()
        rows: list[tuple[str, str, str, str]] = []
        used_count = 0
        ignored_count = 0
        hidden_count = 0
        for image_ref in self._image_refs:
            filter_match = next(
                (
                    discovery_filter
                    for discovery_filter in discovery_filters
                    if discovery_filter.lower() in image_ref.lower()
                ),
                '',
            )
            matched_pattern = next(
                (
                    pattern
                    for pattern in patterns
                    if self._matcher(image_ref, pattern)
                ),
                '',
            )
            if matched_pattern:
                rows.append(('Ignored', image_ref, filter_match, matched_pattern))
                ignored_count += 1
            elif discovery_filters and not filter_match:
                rows.append(('Hidden', image_ref, '', ''))
                hidden_count += 1
            else:
                rows.append(('Used', image_ref, filter_match, ''))
                used_count += 1

        self.summary_label.setText(
            f'{used_count} image(s) will be used; '
            f'{ignored_count} image(s) will be ignored; '
            f'{hidden_count} image(s) will be hidden by discovery filters.'
        )
        self.preview_table.setRowCount(len(rows))
        for row, (
            result,
            image_ref,
            filter_match,
            matched_pattern,
        ) in enumerate(rows):
            result_item = QTableWidgetItem(result)
            if result == 'Ignored':
                result_item.setForeground(QColor('#9a3412'))
            elif result == 'Hidden':
                result_item.setForeground(QColor('#525252'))
            else:
                result_item.setForeground(QColor('#166534'))
            self.preview_table.setItem(row, 0, result_item)
            self.preview_table.setItem(row, 1, QTableWidgetItem(image_ref))
            self.preview_table.setItem(row, 2, QTableWidgetItem(filter_match))
            self.preview_table.setItem(row, 3, QTableWidgetItem(matched_pattern))


class ButtonProfileDialog(QDialog):
    """Edit the configurable top-row command buttons."""

    COLUMNS = [
        ('key', 'Key'),
        ('label', 'Label'),
        ('command', 'Command'),
        ('service', 'Service'),
        ('tooltip', 'Tooltip'),
    ]
    BOOL_FIELDS: set[str] = set()
    REQUIRED_KEYS = {'sim', 'rviz'}
    RESERVED_KEYS = {'roscore', 'terminal'}
    KEY_RE = re.compile(r'^[A-Za-z0-9_.-]+$')

    def __init__(
        self,
        entries: list[dict],
        source_path: Path,
        save_path: Path,
        parent: QWidget | None = None,
    ):
        super().__init__(None)
        self.setWindowTitle('Configure Toolbar Buttons')
        self.setWindowFlag(Qt.Window, True)
        self.resize(960, 560)
        self._owner = parent
        self._source_path = source_path
        self._save_path = save_path

        root = QVBoxLayout(self)
        note = QLabel(
            'Roscore and Terminal are always present and are not editable here. '
            'Every listed button runs its command in its own tab. Sim and RViz '
            'cannot be removed.'
        )
        note.setWordWrap(True)
        root.addWidget(note)

        path_label = QLabel(
            f'Loaded from: {source_path}\nSaves to: {save_path}'
        )
        path_label.setTextInteractionFlags(Qt.TextSelectableByMouse)
        root.addWidget(path_label)

        self.table = ButtonProfileTable(self.COLUMNS)
        self.table.setSelectionBehavior(QAbstractItemView.SelectRows)
        self.table.setSelectionMode(QAbstractItemView.SingleSelection)
        root.addWidget(self.table, 1)

        for entry in entries:
            self._append_row(entry)
        self.table.apply_column_widths()

        actions = QHBoxLayout()
        add_button = QPushButton('Add Command')
        add_button.clicked.connect(self._add_command_row)
        actions.addWidget(add_button)
        remove_button = QPushButton('Remove Selected')
        remove_button.clicked.connect(self._remove_selected_row)
        actions.addWidget(remove_button)
        up_button = QPushButton('Move Up')
        up_button.clicked.connect(lambda: self._move_selected_row(-1))
        actions.addWidget(up_button)
        down_button = QPushButton('Move Down')
        down_button.clicked.connect(lambda: self._move_selected_row(1))
        actions.addWidget(down_button)
        load_button = QPushButton('Load Profile')
        load_button.clicked.connect(self._load_profile)
        actions.addWidget(load_button)
        export_button = QPushButton('Export Profile')
        export_button.clicked.connect(self._export_profile)
        actions.addWidget(export_button)
        actions.addStretch(1)
        root.addLayout(actions)

        buttons = QDialogButtonBox(
            QDialogButtonBox.Save | QDialogButtonBox.Cancel
        )
        buttons.accepted.connect(self.accept)
        buttons.rejected.connect(self.reject)
        root.addWidget(buttons)

    def _append_row(self, entry: dict) -> None:
        row = self.table.rowCount()
        self.table.insertRow(row)
        key_text = str(entry.get('key') or '').strip()
        locked = key_text in self.REQUIRED_KEYS
        for column, (field, _label) in enumerate(self.COLUMNS):
            if field in self.BOOL_FIELDS:
                item = QTableWidgetItem()
                item.setFlags(Qt.ItemIsEnabled | Qt.ItemIsSelectable | Qt.ItemIsUserCheckable)
                item.setCheckState(
                    Qt.Checked if bool(entry.get(field, False)) else Qt.Unchecked
                )
            else:
                value = entry.get(field, '')
                item = QTableWidgetItem('' if value is None else str(value))
                flags = Qt.ItemIsEnabled | Qt.ItemIsSelectable
                if not (
                    locked
                    and field in {'key', 'kind', 'action'}
                ):
                    flags |= Qt.ItemIsEditable
                item.setFlags(flags)
            if column == 0:
                item.setData(Qt.UserRole, copy.deepcopy(entry))
            self.table.setItem(row, column, item)
        self.table.apply_column_widths()

    def _selected_row(self) -> int:
        indexes = self.table.selectionModel().selectedRows()
        return indexes[0].row() if indexes else -1

    def _field_column(self, field: str) -> int:
        return next(
            index
            for index, (candidate, _label) in enumerate(self.COLUMNS)
            if candidate == field
        )

    def _row_key(self, row: int) -> str:
        item = self.table.item(row, self._field_column('key'))
        return item.text().strip() if item else ''

    def _next_command_key(self) -> str:
        existing = {
            self._row_key(row)
            for row in range(self.table.rowCount())
        }
        index = 1
        while f'command_{index}' in existing:
            index += 1
        return f'command_{index}'

    def _add_command_row(self) -> None:
        key = self._next_command_key()
        self._append_row(
            {
                'key': key,
                'label': key.replace('_', ' ').title(),
                'kind': 'command',
                'action': '',
                'command': '',
                'requires_roscore': True,
                'reuse_tab': False,
                'world_config_required': False,
                'world_arg_name': 'world_config',
                'host': False,
                'pass_ros_master_uri': False,
                'service': '',
            }
        )
        self.table.selectRow(self.table.rowCount() - 1)

    def _replace_rows(self, entries: list[dict]) -> None:
        self.table.setRowCount(0)
        for entry in entries:
            self._append_row(entry)
        self.table.apply_column_widths()

    def _profile_dialog_start_dir(self) -> str:
        for path in (self._save_path, self._source_path):
            try:
                if path.parent:
                    return str(path.parent)
            except AttributeError:
                continue
        return str(Path.home())

    def _validated_layout(self) -> list[dict] | None:
        entries = self.button_layout()
        error = self._validation_error(entries)
        if error:
            QMessageBox.warning(self, 'Toolbar Buttons', error)
            return None
        return entries

    def _export_profile(self) -> None:
        entries = self._validated_layout()
        if entries is None:
            return
        path, _selected_filter = QFileDialog.getSaveFileName(
            self,
            'Export Toolbar Button Profile',
            self._profile_dialog_start_dir(),
            'YAML files (*.yaml *.yml);;All files (*)',
        )
        if not path:
            return
        target = Path(path).expanduser()
        if not target.suffix:
            target = target.with_suffix('.yaml')
        try:
            saved_path = save_button_layout(target, entries)
        except OSError as exc:
            QMessageBox.warning(
                self,
                'Toolbar Buttons',
                f'Failed to export toolbar button profile:\n{exc}',
            )
            return
        QMessageBox.information(
            self,
            'Toolbar Buttons',
            f'Exported toolbar button profile to:\n{saved_path}',
        )

    def _load_profile(self) -> None:
        path, _selected_filter = QFileDialog.getOpenFileName(
            self,
            'Load Toolbar Button Profile',
            self._profile_dialog_start_dir(),
            'YAML files (*.yaml *.yml);;All files (*)',
        )
        if not path:
            return
        source = Path(path).expanduser()
        try:
            entries = load_button_layout(source)
        except Exception as exc:
            QMessageBox.warning(
                self,
                'Toolbar Buttons',
                f'Failed to load toolbar button profile:\n{exc}',
            )
            return
        self._replace_rows(entries)
        self.table.selectRow(0)

    def _remove_selected_row(self) -> None:
        row = self._selected_row()
        if row < 0:
            return
        key = self._row_key(row)
        if key in self.REQUIRED_KEYS:
            QMessageBox.information(
                self,
                'Toolbar Buttons',
                'Sim and RViz cannot be removed.',
            )
            return
        self.table.removeRow(row)

    def _move_selected_row(self, direction: int) -> None:
        row = self._selected_row()
        target = row + direction
        if row < 0 or target < 0 or target >= self.table.rowCount():
            return
        values = [self._row_snapshot(row), self._row_snapshot(target)]
        self.table.removeRow(max(row, target))
        self.table.removeRow(min(row, target))
        insert_first = min(row, target)
        ordered = values if direction < 0 else values[::-1]
        for offset, entry in enumerate(ordered):
            self._insert_snapshot(insert_first + offset, entry)
        self.table.selectRow(target)
        self.table.apply_column_widths()

    def _insert_snapshot(self, row: int, entry: dict) -> None:
        self.table.insertRow(row)
        self._write_snapshot_to_row(row, entry)

    def _write_snapshot_to_row(self, row: int, entry: dict) -> None:
        key_text = str(entry.get('key') or '').strip()
        locked = key_text in self.REQUIRED_KEYS
        for column, (field, _label) in enumerate(self.COLUMNS):
            if field in self.BOOL_FIELDS:
                item = QTableWidgetItem()
                item.setFlags(Qt.ItemIsEnabled | Qt.ItemIsSelectable | Qt.ItemIsUserCheckable)
                item.setCheckState(
                    Qt.Checked if bool(entry.get(field, False)) else Qt.Unchecked
                )
            else:
                item = QTableWidgetItem(str(entry.get(field, '') or ''))
                flags = Qt.ItemIsEnabled | Qt.ItemIsSelectable
                if not (
                    locked
                    and field in {'key', 'kind', 'action'}
                ):
                    flags |= Qt.ItemIsEditable
                item.setFlags(flags)
            if column == 0:
                item.setData(Qt.UserRole, copy.deepcopy(entry))
            self.table.setItem(row, column, item)
        self.table.apply_column_widths()

    def _row_snapshot(self, row: int) -> dict:
        base_item = self.table.item(row, 0)
        base = copy.deepcopy(base_item.data(Qt.UserRole) if base_item else {})
        if not isinstance(base, dict):
            base = {}
        original_key = str(base.get('key') or '').strip()
        for column, (field, _label) in enumerate(self.COLUMNS):
            item = self.table.item(row, column)
            if field in self.BOOL_FIELDS:
                base[field] = bool(item and item.checkState() == Qt.Checked)
            else:
                base[field] = item.text().strip() if item else ''
        current_key = str(base.get('key') or '').strip()
        if (
            original_key
            and current_key
            and current_key != original_key
            and current_key not in self.REQUIRED_KEYS
        ):
            base['kind'] = 'command'
            base['action'] = ''
        return base

    def button_layout(self) -> list[dict]:
        return [
            self._row_snapshot(row)
            for row in range(self.table.rowCount())
        ]

    def _validation_error(self, entries: list[dict]) -> str:
        keys: list[str] = []
        for entry in entries:
            key = str(entry.get('key') or '').strip()
            if not key:
                return 'Every button needs a key.'
            if not self.KEY_RE.fullmatch(key):
                return (
                    f'Button key "{key}" may contain only letters, numbers, '
                    'dot, underscore, and hyphen.'
                )
            if key in self.RESERVED_KEYS:
                return 'Roscore and Terminal are fixed buttons and cannot be edited here.'
            if key in keys:
                return f'Button key "{key}" is duplicated.'
            keys.append(key)
            kind = str(entry.get('kind') or 'builtin').strip().lower()
            entry['kind'] = kind
            if key in self.REQUIRED_KEYS:
                required_action = key
                if kind != 'builtin' or str(entry.get('action') or key) != required_action:
                    return 'Sim and RViz must keep their builtin actions.'
                entry['action'] = required_action
            if not str(entry.get('command') or '').strip():
                return f'Button "{key}" needs a command.'
        missing = sorted(self.REQUIRED_KEYS - set(keys))
        if missing:
            return f'Missing required button(s): {", ".join(missing)}.'
        return ''

    def accept(self) -> None:
        entries = self.button_layout()
        error = self._validation_error(entries)
        if error:
            QMessageBox.warning(self, 'Toolbar Buttons', error)
            return
        super().accept()


class AutoLaunchWizard(QDialog):
    """Collect and persist an auto-launch timeline."""

    def __init__(
        self,
        buttons: list[tuple[str, str]],
        timeline: list[dict],
        save_path: Path,
        recording_start_delay_seconds: float = 0.0,
        parent: QWidget | None = None,
    ):
        super().__init__(parent)
        self.setWindowTitle('Configure Auto Launch')
        self._rows: list[tuple[str, QCheckBox, QDoubleSpinBox]] = []

        existing = {
            str(entry.get('button')): float(entry.get('at_seconds', 0.0))
            for entry in timeline
            if isinstance(entry, dict) and entry.get('button') is not None
        }

        root = QVBoxLayout(self)
        root.addWidget(QLabel('Select launch steps and start delays.'))

        for index, (key, label) in enumerate(buttons):
            row = QHBoxLayout()
            checkbox = QCheckBox(label)
            checkbox.setProperty('button_key', key)
            checked = key in existing
            if not existing and key != 'terminal':
                checked = True
            checkbox.setChecked(checked)

            delay = QDoubleSpinBox()
            delay.setRange(0.0, 3600.0)
            delay.setDecimals(1)
            delay.setSingleStep(1.0)
            delay.setSuffix(' s')
            delay.setValue(existing.get(key, float(index * 2)))
            delay.setEnabled(checkbox.isChecked())
            checkbox.toggled.connect(delay.setEnabled)

            row.addWidget(checkbox, 1)
            row.addWidget(QLabel('Delay:'))
            row.addWidget(delay)
            root.addLayout(row)
            self._rows.append((key, checkbox, delay))

        recording_row = QHBoxLayout()
        recording_row.addWidget(QLabel('Extra recording start delay:'), 1)
        self._recording_delay = QDoubleSpinBox()
        self._recording_delay.setRange(0.0, 3600.0)
        self._recording_delay.setDecimals(1)
        self._recording_delay.setSingleStep(1.0)
        self._recording_delay.setSuffix(' s')
        self._recording_delay.setValue(max(0.0, float(recording_start_delay_seconds or 0)))
        self._recording_delay.setToolTip(
            'Additional time to wait after the last launch/layout delay '
            'before Auto Launch recording starts.'
        )
        recording_row.addWidget(self._recording_delay)
        root.addLayout(recording_row)

        path_label = QLabel(f'Saves to: {save_path}')
        path_label.setTextInteractionFlags(Qt.TextSelectableByMouse)
        root.addWidget(path_label)

        self._button_box = QDialogButtonBox(QDialogButtonBox.Save | QDialogButtonBox.Cancel)
        self._button_box.accepted.connect(self.accept)
        self._button_box.rejected.connect(self.reject)
        root.addWidget(self._button_box)

    def timeline(self) -> list[dict]:
        """Return selected launch steps sorted by delay."""
        entries = [
            {
                'button': key,
                'at_seconds': delay.value(),
            }
            for key, checkbox, delay in self._rows
            if checkbox.isChecked()
        ]
        return sorted(entries, key=lambda entry: (entry['at_seconds'], entry['button']))

    def recording_start_delay_seconds(self) -> float:
        """Return extra delay before recording starts after the launch timeline."""
        return max(0.0, float(self._recording_delay.value()))

    def accept(self):
        if not self.timeline():
            QMessageBox.warning(
                self,
                'Auto Launch',
                'Select at least one launch step before saving.',
            )
            return
        super().accept()


class DockerCpConfigDialog(QDialog):
    """Edit persistent docker cp path mappings."""

    IMAGE_SETUP_PREFIX = '__image__:'
    DEFAULT_CONTAINER_PATH = (
        '/root/catkin_ws/src/mobipick_labs/tables_demo_bringup/config/'
        'pick_n_place.rviz'
    )
    CONTAINER_CONFIG_SUFFIX = (
        'src/mobipick_labs/tables_demo_bringup/config/pick_n_place.rviz'
    )

    def __init__(
        self,
        effective_config: dict[str, dict[str, list[dict]]],
        user_config: dict[str, dict[str, list[dict]]],
        selected_workspace: str,
        workspace_names: list[str],
        save_path: Path,
        container_options_provider: Callable[..., list[tuple[str, str]]] | None = None,
        container_path_provider: Callable[[str, str], str] | None = None,
        host_start_provider: Callable[[str], Path] | None = None,
        parent: QWidget | None = None,
    ):
        super().__init__(parent)
        self.setWindowTitle('Configure Docker cp Paths')
        self.resize(900, 620)
        self._effective_config = copy.deepcopy(effective_config or {})
        self._user_config = copy.deepcopy(user_config or {})
        self._selected_workspace = (selected_workspace or '').strip()
        self._workspace_names = [
            str(name).strip()
            for name in (workspace_names or [])
            if str(name).strip()
        ]
        self._container_options_provider = container_options_provider
        self._container_path_provider = container_path_provider
        self._host_start_provider = host_start_provider
        self._loading_profile = False

        root = QVBoxLayout(self)

        profile_row = QHBoxLayout()
        profile_row.addWidget(QLabel('Workspace:'))
        self.profile_combo = QComboBox()
        for label, key in self._profile_items():
            self.profile_combo.addItem(label, key)
        self.profile_combo.currentIndexChanged.connect(self._load_selected_profile)
        profile_row.addWidget(self.profile_combo, 1)
        root.addLayout(profile_row)

        self.tabs = QTabWidget()
        self.host_to_container_table = self._make_table(
            'Host source path',
            'Container destination path',
        )
        self.container_to_host_table = self._make_table(
            'Container source path',
            'Host destination path',
        )
        self.tabs.addTab(
            self._table_page(self.host_to_container_table),
            'Host to Container',
        )
        self.tabs.addTab(
            self._table_page(self.container_to_host_table),
            'Container to Host',
        )
        root.addWidget(self.tabs, 1)

        self.preview = QTextEdit()
        self.preview.setReadOnly(True)
        self.preview.setMinimumHeight(110)
        root.addWidget(self.preview)

        path_label = QLabel(f'Saves to: {save_path}')
        path_label.setTextInteractionFlags(Qt.TextSelectableByMouse)
        root.addWidget(path_label)

        self._button_box = QDialogButtonBox(
            QDialogButtonBox.Save | QDialogButtonBox.Cancel
        )
        self._button_box.accepted.connect(self.accept)
        self._button_box.rejected.connect(self.reject)
        root.addWidget(self._button_box)

        self._load_selected_profile()

    def _profile_items(self) -> list[tuple[str, str]]:
        items = [('Docker image default', 'default')]
        items.extend((name, name) for name in self._workspace_names)
        seen: set[str] = set()
        result: list[tuple[str, str]] = []
        for label, key in items:
            if key in seen:
                continue
            result.append((label, key))
            seen.add(key)
        selected = self._selected_workspace or 'default'
        if selected in seen:
            for index, (_label, key) in enumerate(result):
                if key == selected:
                    result.insert(0, result.pop(index))
                    break
        return result

    def _make_table(self, first_header: str, second_header: str) -> QTableWidget:
        table = QTableWidget(0, 2)
        table.setHorizontalHeaderLabels([first_header, second_header])
        table.setHorizontalScrollMode(QAbstractItemView.ScrollPerPixel)
        header = table.horizontalHeader()
        header.setStretchLastSection(False)
        header.setMinimumSectionSize(180)
        header.setSectionResizeMode(0, QHeaderView.Stretch)
        header.setSectionResizeMode(1, QHeaderView.Stretch)
        table.itemChanged.connect(lambda _item: self._update_preview())
        return table

    def _table_page(self, table: QTableWidget) -> QWidget:
        page = QWidget()
        layout = QVBoxLayout(page)
        layout.addWidget(table)

        row = QHBoxLayout()
        add_button = QPushButton('Add Row')
        add_button.clicked.connect(
            lambda _checked=False, tbl=table: (
                self._add_row_from_dialog(tbl)
            )
        )
        row.addWidget(add_button)
        remove_button = QPushButton('Remove Selected')
        remove_button.clicked.connect(
            lambda _checked=False, tbl=table: self._remove_selected_rows(tbl)
        )
        row.addWidget(remove_button)
        row.addStretch(1)
        layout.addLayout(row)
        return page

    def _add_row_from_dialog(self, table: QTableWidget) -> None:
        host_first = table is self.host_to_container_table
        dialog = DockerCpPathDialog(
            host_first=host_first,
            host_start_path=self._host_start_path(
                self._selected_workspace_key()
            ),
            container_path=self._default_container_path(
                self._selected_workspace_key()
            ),
            container_options_provider=self._container_options_for_selected_workspace,
            container_path_provider=self._container_path_provider,
            parent=self,
        )
        if dialog.exec_() != QDialog.Accepted:
            return
        first, second = dialog.paths()
        self._add_row(table, first, second)

    def _container_options_for_selected_workspace(self) -> list[tuple[str, str]]:
        if not self._container_options_provider:
            return []
        workspace_key = self._selected_workspace_key()
        try:
            return self._container_options_provider(workspace_key)
        except TypeError:
            return self._container_options_provider()

    @classmethod
    def _default_container_path(cls, workspace_name: str = '') -> str:
        workspace = str(workspace_name or '').strip() or 'clean_mobipick_labs_ws'
        preferred_text = f'{workspace}/{cls.CONTAINER_CONFIG_SUFFIX}'
        preferred = Path(preferred_text)
        candidates = [
            preferred,
            Path.home() / 'ros_ws' / preferred,
        ]
        if any(candidate.exists() for candidate in candidates):
            return preferred_text
        return cls.DEFAULT_CONTAINER_PATH

    def _host_start_path(self, workspace_name: str) -> Path:
        if self._host_start_provider:
            return self._host_start_provider(workspace_name)
        return Path.home()

    def _selected_workspace_key(self) -> str:
        data = self.profile_combo.currentData()
        return str(data or 'default').strip() or 'default'

    def _profile_section(self, key: str) -> dict[str, list[dict]]:
        if key in self._user_config:
            return copy.deepcopy(self._user_config.get(key) or {})
        return copy.deepcopy(self._effective_config.get(key) or {})

    def _load_selected_profile(self) -> None:
        self._loading_profile = True
        key = self._selected_workspace_key()
        section = self._profile_section(key)
        if not section and key != 'default':
            section = self._profile_section('default')
        self._set_table_entries(
            self.host_to_container_table,
            section.get('host_to_container', []),
            host_first=True,
        )
        self._set_table_entries(
            self.container_to_host_table,
            section.get('container_to_host', []),
            host_first=False,
        )
        self._loading_profile = False
        self._update_preview()

    def _store_current_profile(self) -> bool:
        if getattr(self, '_loading_profile', False):
            return False
        key = self._selected_workspace_key()
        if not key:
            return False
        try:
            section = self._current_section()
        except ValueError:
            return False
        self._user_config[key] = section
        return True

    def _set_table_entries(
        self,
        table: QTableWidget,
        entries: list[dict],
        *,
        host_first: bool,
    ) -> None:
        table.blockSignals(True)
        table.setRowCount(0)
        for entry in entries or []:
            if not isinstance(entry, dict):
                continue
            host = str(entry.get('host') or '').strip()
            container = str(entry.get('container') or '').strip()
            if not host or not container:
                continue
            if host_first:
                self._add_row(table, host, container)
            else:
                self._add_row(table, container, host)
        table.blockSignals(False)

    def _add_row(
        self,
        table: QTableWidget,
        first: str = '',
        second: str = '',
    ) -> None:
        row = table.rowCount()
        table.insertRow(row)
        table.setItem(row, 0, QTableWidgetItem(first))
        table.setItem(row, 1, QTableWidgetItem(second))
        self._update_preview()

    def _remove_selected_rows(self, table: QTableWidget) -> None:
        rows = {index.row() for index in table.selectedIndexes()}
        for row in sorted(rows, reverse=True):
            table.removeRow(row)
        self._update_preview()

    def _table_entries(
        self,
        table: QTableWidget,
        *,
        host_first: bool,
    ) -> list[dict[str, str]]:
        entries: list[dict[str, str]] = []
        for row in range(table.rowCount()):
            first_item = table.item(row, 0)
            second_item = table.item(row, 1)
            first = first_item.text().strip() if first_item else ''
            second = second_item.text().strip() if second_item else ''
            if not first and not second:
                continue
            if not first or not second:
                raise ValueError('Every docker cp row needs both paths.')
            host, container = (first, second) if host_first else (second, first)
            entries.append({'host': host, 'container': container})
        return entries

    def _current_section(self) -> dict[str, list[dict[str, str]]]:
        return {
            'host_to_container': self._table_entries(
                self.host_to_container_table,
                host_first=True,
            ),
            'container_to_host': self._table_entries(
                self.container_to_host_table,
                host_first=False,
            ),
        }

    def _update_preview(self) -> None:
        key = self._selected_workspace_key()
        try:
            section = self._current_section()
        except ValueError:
            self.preview.setPlainText(
                'Complete both paths in each row to preview commands.'
            )
            return
        lines = [f'Workspace: {key or "default"}']
        for entry in section['host_to_container']:
            lines.append(
                f'docker cp {entry["host"]} <container>:{entry["container"]}'
            )
        for entry in section['container_to_host']:
            lines.append(
                f'docker cp <container>:{entry["container"]} {entry["host"]}'
            )
        if len(lines) == 1:
            lines.append('No docker cp commands configured for this profile.')
        self.preview.setPlainText('\n'.join(lines))
        self._store_current_profile()

    def docker_cp_config(self) -> dict[str, dict[str, list[dict]]]:
        self._store_current_profile()
        return copy.deepcopy(self._user_config)

    def accept(self):
        key = self._selected_workspace_key()
        if not key:
            QMessageBox.warning(
                self,
                'Docker cp Paths',
                'Select a workspace before saving.',
            )
            return
        try:
            section = self._current_section()
        except ValueError as exc:
            QMessageBox.warning(self, 'Docker cp Paths', str(exc))
            return
        self._user_config[key] = section
        self._store_current_profile()
        super().accept()


class DockerCpPathDialog(QDialog):
    """Collect one docker cp mapping with file pickers where available."""

    def __init__(
        self,
        *,
        host_first: bool,
        host_start_path: str | Path | None = None,
        container_path: str,
        container_options_provider: Callable[[], list[tuple[str, str]]] | None = None,
        container_path_provider: Callable[[str, str], str] | None = None,
        parent: QWidget | None = None,
    ):
        super().__init__(parent)
        self._host_first = host_first
        self._host_start_path = (
            Path(host_start_path).expanduser()
            if host_start_path
            else Path.home()
        )
        self._container_options_provider = container_options_provider
        self._container_path_provider = container_path_provider
        self.setWindowTitle(
            'Add Host to Container Path'
            if host_first
            else 'Add Container to Host Path'
        )

        root = QVBoxLayout(self)
        form = QFormLayout()
        form.setFieldGrowthPolicy(QFormLayout.AllNonFixedFieldsGrow)

        self.host_path_edit = QLineEdit()
        self.host_path_edit.setPlaceholderText('Select an existing host file')
        self._configure_path_edit(self.host_path_edit)
        host_row = QHBoxLayout()
        host_row.addWidget(self.host_path_edit, 1)
        host_browse = QPushButton('Browse')
        host_browse.clicked.connect(self._browse_host_path)
        host_row.addWidget(host_browse)

        self.container_path_edit = QLineEdit(container_path)
        self.container_path_edit.setPlaceholderText('Container file path')
        self._configure_path_edit(self.container_path_edit)
        container_row = QHBoxLayout()
        container_row.addWidget(self.container_path_edit, 1)
        container_browse = QPushButton('Browse')
        container_browse.clicked.connect(self._browse_container_path)
        container_row.addWidget(container_browse)
        default_button = QPushButton('Default')
        default_button.clicked.connect(
            lambda _checked=False: self.container_path_edit.setText(
                container_path
            )
        )
        container_row.addWidget(default_button)

        if host_first:
            form.addRow('Host source file:', host_row)
            form.addRow('Container destination:', container_row)
        else:
            form.addRow('Container source:', container_row)
            form.addRow('Host destination file:', host_row)

        root.addLayout(form)
        buttons = QDialogButtonBox(QDialogButtonBox.Ok | QDialogButtonBox.Cancel)
        buttons.accepted.connect(self.accept)
        buttons.rejected.connect(self.reject)
        root.addWidget(buttons)
        self._fit_to_path_texts()

    def _configure_path_edit(self, edit: QLineEdit) -> None:
        edit.setMinimumWidth(520)
        edit.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        edit.textChanged.connect(lambda _text: self._fit_to_path_texts())

    def _fit_to_path_texts(self) -> None:
        metrics = self.fontMetrics()
        longest = max(
            (
                self.host_path_edit.text(),
                self.container_path_edit.text(),
                self.host_path_edit.placeholderText(),
                self.container_path_edit.placeholderText(),
            ),
            key=lambda text: _text_width(metrics, text),
        )
        desired = _text_width(metrics, longest) + 360
        width = min(max(760, desired), 1400)
        self.setMinimumWidth(width)
        if self.width() < width:
            self.resize(width, max(self.height(), self.sizeHint().height()))

    def _browse_host_path(self) -> None:
        current = self.host_path_edit.text().strip()
        start = str(self._host_browse_start(current, self._host_start_path))
        selected, _filter = QFileDialog.getOpenFileName(
            self,
            'Select Host File',
            start,
        )
        if selected:
            self.host_path_edit.setText(selected)

    @staticmethod
    def _host_browse_start(current: str, default_start: Path) -> Path:
        if current:
            parent = Path(current).expanduser().parent
            if parent.is_dir():
                return parent.resolve()
        start = Path(default_start).expanduser()
        if start.is_file():
            start = start.parent
        if start.is_dir():
            return start.resolve()
        return Path.home()

    def _browse_container_path(self) -> None:
        if self._container_options_provider and self._container_path_provider:
            dialog = DockerCpContainerSelectDialog(
                self._container_options_provider(),
                self,
            )
            if dialog.exec_() != QDialog.Accepted:
                return
            container_ref = dialog.container_ref()
            if container_ref:
                selected = self._container_path_provider(
                    container_ref,
                    self.container_path_edit.text().strip(),
                )
                if selected:
                    self.container_path_edit.setText(selected)
                return

        selected, _filter = QFileDialog.getOpenFileName(
            self,
            'Select Container File',
            self._container_browse_start(),
        )
        if selected:
            self.container_path_edit.setText(
                self._container_path_from_selection(Path(selected))
            )

    @staticmethod
    def _container_browse_start(workspace_name: str = '') -> str:
        preferred = Path(DockerCpConfigDialog._default_container_path(workspace_name))
        candidates = [
            preferred,
            Path.home() / 'ros_ws' / preferred,
        ]
        for candidate in candidates:
            if candidate.is_file():
                return str(candidate.resolve())
            if candidate.parent.is_dir():
                return str(candidate.parent.resolve())
        return str(Path.home())

    @staticmethod
    def _container_path_from_selection(path: Path) -> str:
        suffix = Path(DockerCpConfigDialog.CONTAINER_CONFIG_SUFFIX)
        parts = path.parts
        suffix_parts = suffix.parts
        for index in range(0, len(parts) - len(suffix_parts)):
            if parts[index + 1:index + 1 + len(suffix_parts)] == suffix_parts:
                workspace = parts[index]
                return f'{workspace}/{DockerCpConfigDialog.CONTAINER_CONFIG_SUFFIX}'
        return str(path)

    def paths(self) -> tuple[str, str]:
        host = self.host_path_edit.text().strip()
        container = self.container_path_edit.text().strip()
        return (host, container) if self._host_first else (container, host)

    def accept(self) -> None:
        host = self.host_path_edit.text().strip()
        container = self.container_path_edit.text().strip()
        if not host or not container:
            QMessageBox.warning(
                self,
                'Docker cp Paths',
                'Choose both the host file and container path.',
            )
            return
        host_path = Path(os.path.expanduser(os.path.expandvars(host)))
        if not host_path.is_file():
            QMessageBox.warning(
                self,
                'Docker cp Paths',
                'Choose an existing host file.',
            )
            return
        super().accept()


class DockerCpContainerSelectDialog(QDialog):
    """Choose a running container for container-side path setup."""

    def __init__(
        self,
        options: list[tuple[str, str]],
        parent: QWidget | None = None,
    ):
        super().__init__(parent)
        self.setWindowTitle('Select Setup Container')
        self.resize(520, 140)

        root = QVBoxLayout(self)
        root.addWidget(QLabel('Container used to browse or verify paths:'))
        self.container_combo = QComboBox()
        for label, ref in options or []:
            clean_ref = str(ref or '').strip()
            if clean_ref:
                self.container_combo.addItem(str(label or clean_ref), clean_ref)
        root.addWidget(self.container_combo)

        buttons = QDialogButtonBox(QDialogButtonBox.Ok | QDialogButtonBox.Cancel)
        buttons.accepted.connect(self.accept)
        buttons.rejected.connect(self.reject)
        root.addWidget(buttons)

    def container_ref(self) -> str:
        return str(self.container_combo.currentData() or '').strip()

    def accept(self) -> None:
        if not self.container_ref():
            QMessageBox.warning(
                self,
                'Docker cp Paths',
                'Select a running container or type the path manually.',
            )
            return
        super().accept()


class DockerCpContainerPathDialog(QDialog):
    """Browse existing container paths while allowing arbitrary destinations."""

    def __init__(
        self,
        *,
        container_ref: str,
        start_path: str,
        list_provider: Callable[[str, str], list[dict[str, object]]],
        parent: QWidget | None = None,
    ):
        super().__init__(parent)
        self._container_ref = container_ref
        self._list_provider = list_provider
        self.setWindowTitle('Select Container File')
        self.resize(900, 560)

        root = QVBoxLayout(self)
        root.addWidget(QLabel(f'Container: {container_ref}'))

        path_row = QHBoxLayout()
        self.path_edit = QLineEdit(start_path)
        self.path_edit.setMinimumWidth(620)
        self.path_edit.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        path_row.addWidget(self.path_edit, 1)
        up_button = QPushButton('Up')
        up_button.clicked.connect(self._go_up)
        path_row.addWidget(up_button)
        refresh_button = QPushButton('Refresh')
        refresh_button.clicked.connect(self._refresh)
        path_row.addWidget(refresh_button)
        root.addLayout(path_row)

        self.entries = QListWidget()
        self.entries.itemDoubleClicked.connect(self._activate_item)
        root.addWidget(self.entries, 1)

        buttons = QDialogButtonBox(QDialogButtonBox.Ok | QDialogButtonBox.Cancel)
        buttons.accepted.connect(self.accept)
        buttons.rejected.connect(self.reject)
        root.addWidget(buttons)

        self._refresh()

    def selected_path(self) -> str:
        return self.path_edit.text().strip()

    def _refresh(self) -> None:
        self.entries.clear()
        for entry in self._list_provider(
            self._container_ref,
            self.path_edit.text().strip(),
        ):
            error = str(entry.get('error') or '').strip()
            if error:
                item = QListWidgetItem(error)
                item.setForeground(QColor('firebrick'))
                item.setFlags(item.flags() & ~Qt.ItemIsEnabled)
                self.entries.addItem(item)
                continue
            path = str(entry.get('path') or '').strip()
            if not path:
                continue
            is_dir = bool(entry.get('is_dir'))
            name = str(entry.get('name') or Path(path).name or path)
            label = f'[dir] {name}' if is_dir else name
            item = QListWidgetItem(label)
            item.setData(Qt.UserRole, {'path': path, 'is_dir': is_dir})
            self.entries.addItem(item)
        if self.entries.count() == 0:
            item = QListWidgetItem('No files found in this directory.')
            item.setForeground(QColor('gray'))
            item.setFlags(item.flags() & ~Qt.ItemIsEnabled)
            self.entries.addItem(item)

    def _activate_item(self, item: QListWidgetItem) -> None:
        data = item.data(Qt.UserRole) or {}
        path = str(data.get('path') or '').strip()
        if not path:
            return
        self.path_edit.setText(path)
        if bool(data.get('is_dir')):
            self._refresh()

    def _go_up(self) -> None:
        current = self.path_edit.text().strip() or '/'
        parent = str(Path(current).parent)
        self.path_edit.setText(parent if parent else '/')
        self._refresh()


class WorkspaceMatchDialog(QDialog):
    """Edit image-to-workspace compatibility profile entries."""

    def __init__(
        self,
        images: list[str],
        workspaces: list[str],
        matches: dict[str, list[str]],
        parent=None,
    ):
        super().__init__(parent)
        self.setWindowTitle('Workspace Matches')
        self.resize(520, 460)
        self._matches = {
            image: list(dict.fromkeys(values))
            for image, values in matches.items()
        }
        self._checkboxes: dict[str, QCheckBox] = {}

        layout = QVBoxLayout(self)
        form = QFormLayout()
        self.image_combo = QComboBox()
        for image in images:
            self.image_combo.addItem(image, image)
        self.image_combo.currentIndexChanged.connect(self._load_image)
        form.addRow('Docker image:', self.image_combo)
        layout.addLayout(form)

        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        content = QWidget()
        self.checkbox_layout = QVBoxLayout(content)
        self._add_checkbox('Docker image default', 'Docker image default')
        for workspace in workspaces:
            self._add_checkbox(workspace, workspace)
        self.checkbox_layout.addStretch(1)
        scroll.setWidget(content)
        layout.addWidget(scroll)

        buttons = QDialogButtonBox(
            QDialogButtonBox.Save | QDialogButtonBox.Cancel
        )
        buttons.accepted.connect(self.accept)
        buttons.rejected.connect(self.reject)
        layout.addWidget(buttons)
        self._load_image()

    def _add_checkbox(self, label: str, value: str) -> None:
        checkbox = QCheckBox(label)
        checkbox.stateChanged.connect(self._store_current_image)
        self._checkboxes[value] = checkbox
        self.checkbox_layout.addWidget(checkbox)

    def _current_image(self) -> str:
        return str(self.image_combo.currentData() or '').strip()

    def _store_current_image(self) -> None:
        image = self._current_image()
        if not image:
            return
        self._matches[image] = [
            value
            for value, checkbox in self._checkboxes.items()
            if checkbox.isChecked()
        ]

    def _load_image(self) -> None:
        image = self._current_image()
        selected = set(self._matches.get(image, []))
        for value, checkbox in self._checkboxes.items():
            checkbox.blockSignals(True)
            checkbox.setChecked(value in selected)
            checkbox.blockSignals(False)

    def accept(self) -> None:
        self._store_current_image()
        super().accept()

    def matches(self) -> dict[str, list[str]]:
        return copy.deepcopy(self._matches)


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
        self._restore_window_state(window_cfg)

        self._killing = False
        self._last_search = ''
        self._yaml_path = None
        self._custom_counter = 0
        self._timers_cfg = CONFIG['timers']
        self._images_cfg = CONFIG['images']
        self._image_profiles = self._normalize_image_profiles(
            self._images_cfg.get('profiles', [])
        )
        self._workspace_warning_cfg = CONFIG.get(
            'workspace_mismatch_warning',
            {},
        )
        self._workspace_mismatch_session_exceptions = (
            self._normalize_workspace_mismatch_exceptions(
                self._workspace_warning_cfg.get('silenced_exceptions', [])
            )
        )
        self._selected_image = self._images_cfg.get('default', '')
        self._image_choices: list[str] = []
        self._related_patterns: list[str] = []
        self._worlds_cfg = CONFIG['worlds']
        self._default_world = self._worlds_cfg.get('default', 'moelk_tables')
        self._selected_world = self._default_world
        self._scripts_dir = PROJECT_ROOT / 'scripts'
        self._script_choices: list[str] = []
        self._script_active_tab_key: str | None = None
        self._roscore_container_name = 'mobipick-roscore'
        self._roscore_running_cached = False
        self._roscore_stopping = False
        self._roscore_last_start_ts: float | None = None
        self._ros_cfg = CONFIG.get('ros', {})
        configured_master = self._normalize_ros_master_uri(
            self._ros_cfg.get('remote_master_uri', '')
        )
        self._remote_master_uri_value = (
            configured_master or 'http://mobipick-os-sensor:11311'
        )
        self._remote_ros_service = str(
            self._ros_cfg.get('remote_service', 'mobipick_remote_cmd')
        ).strip() or 'mobipick_remote_cmd'
        remote_default = self._ros_cfg.get(
            'remote_enabled_by_default',
            False,
        )
        if isinstance(remote_default, str):
            self._remote_master_enabled_value = (
                remote_default.strip().lower() in {'1', 'true', 'yes', 'on'}
            )
        else:
            self._remote_master_enabled_value = bool(remote_default)
        self._terminal_cfg = CONFIG.get('terminal', {})
        drop_to_host_user_default = self._terminal_cfg.get('drop_to_host_user', True)
        if drop_to_host_user_default is None:
            self._terminal_drop_to_host_user_default = True
        elif isinstance(drop_to_host_user_default, str):
            lowered = drop_to_host_user_default.strip().lower()
            self._terminal_drop_to_host_user_default = lowered not in {'0', 'false', 'no', 'off'}
        else:
            self._terminal_drop_to_host_user_default = bool(drop_to_host_user_default)
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
        self._project_root = PROJECT_ROOT
        self._workspace_load_error = ''
        workspace_registry = WorkspaceRegistry(
            container_workspace_root=(
                Path(
                    CONFIG['process']['qprocess_env'][
                        'MOBIPICK_HOST_HOME'
                    ]
                )
                / 'ros_ws'
            ),
        )
        try:
            workspace_registry.load()
        except Exception as exc:
            self._workspace_load_error = (
                f'Failed to load workspace registry '
                f'{workspace_registry.path}: {exc}'
            )
        self._workspace_registry = workspace_registry
        self._empty_workspace_dir = (
            self._workspace_registry.path.parent / 'empty_workspace'
        )
        try:
            self._empty_workspace_dir.mkdir(parents=True, exist_ok=True)
        except OSError:
            self._empty_workspace_dir = PROJECT_ROOT / 'empty_workspace'
        self._workspace_dialog: WorkspaceManagerDialog | None = None
        self._setup_wizard_dialog: ImageSetupWizard | None = None
        self._setup_wizard_auto_scheduled = False
        self._docker_cp_config = load_docker_cp_config(
            self._workspace_docker_cp_config_path()
        )
        self._synced_container_refs: set[str] = set()
        self._toggle_states: dict[str, str] = {}
        self._last_log_origin: dict[str, str] = {}
        self._gui_log_color = str(CONFIG['log'].get('gui_log_color', '#ff00ff'))
        self._command_log_color = str(CONFIG['log'].get('command_log_color', '#4da3ff'))
        self._default_image_dialog_shown = False
        self._bug_report_dialog: BugReportDialog | None = None
        self._documentation_dialog: DocumentationDialog | None = None
        self._config_paths_dialog: QDialog | None = None
        self._config_path_contents_dialog: QDialog | None = None
        self._view_actions: dict[str, QAction] = {}
        self._active_menu_tooltip_action: QAction | None = None
        self._create_menu_bar()

        # sim state
        self._sim_container_name = 'mobipick-run'
        self._xhost_sources: set[str] = set()
        self._sim_running_cached = False  # event driven sim state

        self.tasks: dict[str, ProcessTab] = {}
        self._bg_procs: list[QProcess] = []
        self._cleanup_done = False
        self._exit_in_progress = False
        self._exit_dialog: Optional[QMessageBox] = None
        self._docker_stop_timeout = self._normalize_stop_timeout(CONFIG['exit'].get('docker_stop_timeout'))
        self._button_layout = load_button_layout(
            self._workspace_button_config_path()
        )
        self._launch_plan = load_launch_sequence_plan(
            self._workspace_button_config_path(),
            self._workspace_launch_config_path(),
        )
        self._launch_retry_ms = max(0, int(self._launch_plan.get('retry_delay_ms', 750) or 0))
        self._launch_max_retry = max(0, int(self._launch_plan.get('max_retry_attempts', 6) or 0))
        self._button_widgets: dict[str, QPushButton] = {}
        self._config_buttons: dict[str, dict] = {}
        self._config_button_order: list[str] = []
        self._auto_launch_running = False
        self._auto_launch_stopping = False
        self._auto_launch_timers: list[QTimer] = []
        self._auto_launch_active_keys: list[str] = []
        self._auto_launch_run_count = 0
        self._window_layout_cfg = CONFIG.get('window_layout', {})
        raw_auto_apply = self._window_layout_cfg.get('auto_apply', True)
        if isinstance(raw_auto_apply, str):
            self._window_layout_auto_apply = raw_auto_apply.strip().lower() not in {'0', 'false', 'no', 'off'}
        else:
            self._window_layout_auto_apply = bool(raw_auto_apply)
        self._window_layout_path_template = (
            self._window_layout_cfg.get('state_file') or str(WINDOW_LAYOUT_FILE)
        )
        self._window_layout_path = self._workspace_window_layout_path()
        self._window_layout_delay_ms = self._compute_window_layout_delay_ms()
        self._window_layout_manager = WindowLayoutManager(
            state_file=self._window_layout_path,
            log_info=self._log_info,
            log_warning=lambda msg: self._append_gui_html('log', f'<i>{html.escape(msg)}</i>'),
            log_debug=lambda msg: self._console_log(3, msg),
            apply_delay_ms=self._window_layout_delay_ms,
        )
        self._window_layout_manager.record_baseline(exclude_titles={self.windowTitle()})
        self._window_layout_dialog: QDialog | None = None
        self._recording_cfg = CONFIG.get('recording', {})
        self._recording_default_checked = bool(
            self._recording_cfg.get('enabled_by_default', False)
        )
        self._recording_remember_output_dir = bool(
            self._recording_cfg.get('remember_output_dir', False)
        )
        self._recording_show_control_window = bool(
            self._recording_cfg.get('show_control_window', True)
        )
        self._recording_output_root = self._resolve_recording_output_root()
        self._recording_workspace_name = self._normalize_workspace_name(self._recording_cfg.get('workspace_name'))
        active_workspace = self._workspace_registry.active_workspace()
        if active_workspace:
            self._recording_workspace_name = active_workspace.name
        self._screen_resolution = ''
        self._recording_counter = self._load_recording_counter()
        self._recording_resolutions = self._load_recording_resolutions()
        self._recording_default_resolution = self._select_default_resolution()
        self._recording_start_timer: QTimer | None = None
        self._recording_proc: QProcess | None = None
        self._recording_session: dict | None = None
        self._recording_window: QDialog | None = None
        self._recording_stop_button: QPushButton | None = None
        self._recording_path_label: QLabel | None = None
        self._recording_indicator_timer: QTimer | None = None
        self._recording_indicator_on = False

        central = QWidget()
        self.setCentralWidget(central)
        root = QVBoxLayout(central)

        workspace_row = QHBoxLayout()
        workspace_row.addWidget(QLabel('ROS 1 workspace:'))
        self.workspace_combo = QComboBox()
        self.workspace_combo.setSizePolicy(
            QSizePolicy.Expanding,
            QSizePolicy.Preferred,
        )
        self.workspace_combo.currentIndexChanged.connect(
            self._on_workspace_changed
        )
        workspace_row.addWidget(self.workspace_combo)
        self.manage_workspaces_button = QPushButton('Configure Workspaces')
        self.manage_workspaces_button.clicked.connect(
            self._open_workspace_manager
        )
        workspace_row.addWidget(self.manage_workspaces_button)
        root.addLayout(workspace_row)

        self.ros_master_controls = QWidget()
        ros_master_row = QHBoxLayout(self.ros_master_controls)
        ros_master_row.setContentsMargins(0, 0, 0, 0)
        self.remote_master_checkbox = QCheckBox('Use remote ROS master')
        self.remote_master_checkbox.setChecked(
            self._remote_master_enabled_value
        )
        self.remote_master_checkbox.setToolTip(
            'Run ROS tool containers with host networking and connect them '
            'to the selected external ROS 1 master.'
        )
        ros_master_row.addWidget(self.remote_master_checkbox)
        ros_master_row.addWidget(QLabel('ROS_MASTER_URI:'))
        self.remote_master_input = QLineEdit(
            self._remote_master_uri_value
        )
        self.remote_master_input.setPlaceholderText(
            'http://mobipick-os-sensor:11311'
        )
        self.remote_master_input.setEnabled(
            self._remote_master_enabled_value
        )
        self.remote_master_input.setToolTip(
            'ROS 1 master used by RViz, RQt, scripts, terminals, and custom '
            'commands while remote mode is enabled.'
        )
        ros_master_row.addWidget(self.remote_master_input)
        root.addWidget(self.ros_master_controls)
        self.remote_master_checkbox.toggled.connect(
            self._on_remote_master_toggled
        )
        self.remote_master_input.editingFinished.connect(
            self._on_remote_master_uri_edited
        )

        # top controls
        top = QHBoxLayout()
        self._top_controls_layout = top
        self.roscore_button = QPushButton()
        _configure_expanding_toolbar_button(self.roscore_button)
        self.roscore_button.clicked.connect(self._on_roscore_toggle_clicked)
        top.addWidget(self.roscore_button)
        self._button_widgets['roscore'] = self.roscore_button

        self._build_configurable_buttons(top)

        self.terminal_button = QPushButton()
        _configure_expanding_toolbar_button(self.terminal_button)
        self.terminal_button.clicked.connect(self._on_terminal_toggle_clicked)
        top.addWidget(self.terminal_button)
        self._button_widgets['terminal'] = self.terminal_button

        self.terminal_root_checkbox = QCheckBox('Run as root')
        self.terminal_root_checkbox.setToolTip('When checked, new terminals run as root inside the container.')
        self.terminal_root_checkbox.setChecked(not self._terminal_drop_to_host_user_default)
        top.addWidget(self.terminal_root_checkbox)

        spacer = QWidget()
        spacer.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Preferred)
        top.addWidget(spacer)
        root.addLayout(top)
        self._populate_workspace_combo()

        self.clear_button = QPushButton('Clear Current Tab')
        self.clear_button.clicked.connect(self.clear_current_tab)

        self.clear_all_button = QPushButton('Clear All Tabs')
        self.clear_all_button.clicked.connect(self.clear_all_tabs)

        self.commit_current_tab_button = QPushButton('Commit Current Tab')
        self.commit_current_tab_button.setToolTip('Create a docker image from the container backing the current tab')
        self.commit_current_tab_button.clicked.connect(self.commit_current_tab)

        self.manage_images_button = QPushButton('Manage Images')
        self.manage_images_button.setToolTip('Remove docker images that match the configured filters')
        self.manage_images_button.clicked.connect(self.manage_images)

        self.setup_wizard_button = QPushButton('Setup Wizard')
        self.setup_wizard_button.setToolTip('Configure Docker images and first-run setup')
        self.setup_wizard_button.clicked.connect(
            lambda _checked=False: self._open_setup_wizard()
        )

        self.build_custom_image_button = QPushButton('Build Custom Image')
        self.build_custom_image_button.setToolTip('Build a host-user development Docker image')
        self.build_custom_image_button.clicked.connect(
            lambda _checked=False: self._open_custom_image_builder()
        )

        self.execute_docker_cp_button = QPushButton('Execute Docker cp')
        self.execute_docker_cp_button.setToolTip('Copy configured paths from the active container to the host')
        self.execute_docker_cp_button.clicked.connect(self.execute_docker_cp_from_container)

        self.window_layout_button = QPushButton('Window Layout')
        self.window_layout_button.setToolTip('Open helper to save window positions for wmctrl replay')
        self.window_layout_button.clicked.connect(self._on_window_layout_clicked)

        self.save_current_button = QPushButton('Save Current Log')
        self.save_current_button.clicked.connect(self.save_current_log)

        self.load_log_button = QPushButton('Load Log')
        self.load_log_button.clicked.connect(self.load_log_file)

        self.save_all_button = QPushButton('Save All Logs')
        self.save_all_button.clicked.connect(self.save_all_logs)

        # actions row
        actions = QHBoxLayout()

        self.auto_launch_button = QPushButton()
        self.auto_launch_button.clicked.connect(self._on_auto_launch_toggle_clicked)
        auto_launch_tooltip = str(self._launch_plan.get('button', {}).get('tooltip') or '').strip()
        self._auto_launch_base_tooltip = auto_launch_tooltip
        if auto_launch_tooltip:
            self.auto_launch_button.setToolTip(auto_launch_tooltip)
        actions.addWidget(self.auto_launch_button)
        self._button_widgets['auto_launch'] = self.auto_launch_button

        self.recording_controls = QWidget()
        recording_row = QHBoxLayout(self.recording_controls)
        recording_row.setContentsMargins(0, 0, 0, 0)
        self.record_checkbox = QCheckBox('Record Auto Launch')
        self.record_checkbox.setToolTip(
            'Record the Auto Launch run: screen video plus run logs. '
            'Recording starts after you press Auto Launch and its timeline '
            'and window-layout delay finish.'
        )
        self.record_checkbox.setChecked(self._recording_default_checked)
        self.record_checkbox.toggled.connect(self._on_record_checkbox_toggled)
        recording_row.addWidget(self.record_checkbox)
        self.recording_indicator = QLabel('REC off')
        self.recording_indicator.setMinimumWidth(170)
        self.recording_indicator.setAlignment(Qt.AlignCenter)
        recording_row.addWidget(self.recording_indicator)
        self.recording_options_button = QPushButton('Recording Options')
        self.recording_options_button.clicked.connect(
            self._open_recording_options
        )
        recording_row.addWidget(self.recording_options_button)
        self.record_resolution_combo = QComboBox()
        self.record_resolution_combo.setInsertPolicy(QComboBox.NoInsert)
        self.record_resolution_combo.addItems(self._recording_resolutions)
        if self._recording_default_resolution in self._recording_resolutions:
            self.record_resolution_combo.setCurrentIndex(
                self._recording_resolutions.index(self._recording_default_resolution)
            )
        else:
            self.record_resolution_combo.setCurrentIndex(0)
        self.record_resolution_combo.setToolTip('Select the video resolution used for Auto Launch recording.')
        recording_row.addWidget(self.record_resolution_combo)
        actions.addWidget(self.recording_controls)
        self._update_recording_location_tooltip()
        self._set_recording_indicator(
            'armed' if self.record_checkbox.isChecked() else 'off'
        )

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

        root.addLayout(actions)

        self.script_controls = QWidget()
        scripts_row = QHBoxLayout(self.script_controls)
        scripts_row.setContentsMargins(0, 0, 0, 0)
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
        root.addWidget(self.script_controls)

        self.run_script_button.setEnabled(False)
        self.script_combo.setEnabled(False)

        self._ensure_scripts_dir()

        self._update_related_patterns()
        self._load_available_images()

        # custom command row
        self.command_controls = QWidget()
        cmdrow = QHBoxLayout(self.command_controls)
        cmdrow.setContentsMargins(0, 0, 0, 0)
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
        root.addWidget(self.command_controls)
        self._apply_view_action_visibility()

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

        self._create_workspace_tabs()

        self._apply_env_to_all_tabs()
        self._refresh_script_options()
        self._window_layout_manager.load_layout()

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
        if self._workspace_load_error:
            self._console_log(1, self._workspace_load_error)
        self._schedule_first_run_setup_wizard()

        app_instance = QApplication.instance()
        if app_instance:
            app_instance.aboutToQuit.connect(self._ensure_cleanup_before_exit)

    # ---------- Menu bar ----------

    def _create_menu_bar(self) -> None:
        menu_bar = self.menuBar()

        workspace_menu = self._add_menu(menu_bar, 'Workspace')
        self._add_menu_action(
            workspace_menu,
            'Configure Workspaces',
            self._open_workspace_manager,
        )
        self._add_menu_action(
            workspace_menu,
            'Configure Workspace Matches',
            self._open_workspace_match_editor,
            tooltip='Edit which Docker images match which ROS workspaces',
        )
        self._add_menu_action(
            workspace_menu,
            'Build Active Workspace',
            self._build_active_workspace,
        )

        settings_menu = self._add_menu(menu_bar, 'Settings')
        self._add_menu_action(
            settings_menu,
            'Export All Settings...',
            self._export_all_settings,
            tooltip='Save portable GUI settings to one YAML file',
        )
        self._add_menu_action(
            settings_menu,
            'Import All Settings...',
            self._import_all_settings,
            tooltip='Load portable GUI settings from one YAML file',
        )
        settings_menu.addSeparator()
        self._add_menu_action(
            settings_menu,
            'Show Configuration Paths',
            self._show_configuration_paths,
            tooltip='Show writable config and data paths managed by the GUI',
        )

        tools_menu = self._add_menu(menu_bar, 'Tools')
        self._add_menu_action(
            tools_menu,
            'Configure Toolbar Buttons',
            self._open_button_profile_dialog,
            tooltip='Edit the active workspace toolbar button profile',
        )
        tools_menu.addSeparator()

        docker_menu = self._add_menu(tools_menu, 'Docker')
        self._add_menu_action(docker_menu, 'Manage Images', self.manage_images)
        self._add_menu_action(
            docker_menu,
            'Setup Wizard',
            lambda _checked=False: self._open_setup_wizard(),
        )
        self._add_menu_action(
            docker_menu,
            'Configure Image Filters',
            self._open_image_blacklist_dialog,
            tooltip='Choose which Docker images are shown or ignored',
        )
        self._add_menu_action(
            docker_menu,
            'Build Custom Image',
            lambda _checked=False: self._open_custom_image_builder(),
            tooltip='Build a host-user development Docker image',
        )
        self._add_menu_action(
            docker_menu,
            'Commit Current Tab',
            self.commit_current_tab,
        )
        self._add_menu_action(
            docker_menu,
            'Execute Docker cp',
            self.execute_docker_cp_from_container,
            tooltip='Copy configured paths from the active container to the host',
        )
        self._add_menu_action(
            docker_menu,
            'Configure Docker cp Paths',
            self._open_docker_cp_config_dialog,
            tooltip='Edit persistent docker cp paths for default or image-specific profiles',
        )

        layout_menu = self._add_menu(tools_menu, 'Layout')
        self._add_menu_action(
            layout_menu,
            'Window Layout',
            self._on_window_layout_clicked,
            tooltip='Open helper to save window positions for wmctrl replay',
        )

        automation_menu = self._add_menu(tools_menu, 'Automation')
        self._add_menu_action(
            automation_menu,
            'Configure Auto Launch',
            self._open_auto_launch_wizard,
        )
        tools_menu.addSeparator()
        self._add_menu_action(
            tools_menu,
            'Update Status',
            self._on_refresh_clicked,
            tooltip='Refresh Docker container status',
        )

        view_menu = self._add_menu(menu_bar, 'View')
        self._view_actions['recording'] = self._add_checkable_menu_action(
            view_menu,
            'Recording Controls',
            self._set_recording_controls_visible,
        )
        self._view_actions['scripts'] = self._add_checkable_menu_action(
            view_menu,
            'Script Controls',
            self._set_script_controls_visible,
        )
        self._view_actions['commands'] = self._add_checkable_menu_action(
            view_menu,
            'Command Controls',
            self._set_command_controls_visible,
        )
        self._view_actions['remote_master'] = self._add_checkable_menu_action(
            view_menu,
            'Remote ROS Master',
            self._set_remote_master_controls_visible,
        )
        view_menu.addSeparator()
        self._add_menu_action(view_menu, 'Refresh Images', self._reload_images)

        logs_menu = self._add_menu(menu_bar, 'Logs')
        self._add_menu_action(logs_menu, 'Save Current Log', self.save_current_log)
        self._add_menu_action(logs_menu, 'Load Log', self.load_log_file)
        self._add_menu_action(logs_menu, 'Save All Logs', self.save_all_logs)
        logs_menu.addSeparator()
        self._add_menu_action(logs_menu, 'Clear Current Tab', self.clear_current_tab)
        self._add_menu_action(logs_menu, 'Clear All Tabs', self.clear_all_tabs)

        help_menu = self._add_menu(menu_bar, 'Help')
        self._add_menu_action(
            help_menu,
            'Documentation',
            self._open_documentation_dialog,
        )
        self._add_menu_action(
            help_menu,
            'File Bug Report...',
            self._open_bug_report_dialog,
        )
        self._add_menu_action(help_menu, 'About', self._show_about_dialog)

    def _add_menu(self, parent, text: str):
        menu = parent.addMenu(text)
        menu.setMouseTracking(True)
        menu.installEventFilter(self)
        return menu

    def eventFilter(self, watched, event):  # noqa: N802 - Qt API
        if isinstance(watched, QMenu):
            if event.type() == QEvent.MouseMove:
                self._update_menu_tooltip(watched, event)
            elif event.type() in (QEvent.Leave, QEvent.Hide):
                self._hide_menu_tooltip()
        return super().eventFilter(watched, event)

    def _update_menu_tooltip(self, menu: QMenu, event) -> None:
        action = menu.actionAt(event.pos())
        tooltip = ''
        if action is not None:
            tooltip = str(action.property('mobipick_menu_tooltip') or '')
        if action is None or not tooltip:
            self._hide_menu_tooltip()
            return
        if action is self._active_menu_tooltip_action and QToolTip.isVisible():
            return

        self._active_menu_tooltip_action = action
        action_rect = menu.actionGeometry(action)
        tooltip_pos = event.globalPos() + QPoint(12, 16)
        QToolTip.showText(tooltip_pos, tooltip, menu, action_rect)

    def _hide_menu_tooltip(self) -> None:
        self._active_menu_tooltip_action = None
        QToolTip.hideText()

    def _add_menu_action(
        self,
        menu,
        text: str,
        slot,
        *,
        tooltip: str = '',
    ) -> QAction:
        action = QAction(text, self)
        if tooltip:
            action.setProperty('mobipick_menu_tooltip', tooltip)
            action.setToolTip(tooltip)
            action.setStatusTip(tooltip)
        action.triggered.connect(
            lambda _checked=False, callback=slot: callback()
        )
        menu.addAction(action)
        return action

    def _add_checkable_menu_action(self, menu, text: str, slot) -> QAction:
        action = QAction(text, self)
        action.setCheckable(True)
        action.setChecked(False)
        action.toggled.connect(slot)
        menu.addAction(action)
        return action

    def _export_all_settings(self) -> None:
        selected, _ = QFileDialog.getSaveFileName(
            self,
            'Export Mobipick GUI settings',
            str(Path.home() / 'mobipick-gui-settings.yaml'),
            'YAML files (*.yaml *.yml)',
        )
        if not selected:
            return
        try:
            export_settings(Path(selected), self._workspace_registry)
        except (OSError, ValueError) as exc:
            QMessageBox.critical(self, 'Export Settings', str(exc))
            return
        QMessageBox.information(
            self,
            'Export Settings',
            f'Settings exported to:\n{selected}',
        )

    def _import_all_settings(self) -> None:
        if self._workspace_processes_running():
            QMessageBox.warning(
                self,
                'Import Settings',
                'Stop running workspace processes before importing settings.',
            )
            return
        selected, _ = QFileDialog.getOpenFileName(
            self,
            'Import Mobipick GUI settings',
            str(Path.home()),
            'YAML files (*.yaml *.yml);;All files (*)',
        )
        if not selected:
            return
        start = self._workspace_registry.master_folder or str(Path.home())
        master = QFileDialog.getExistingDirectory(
            self,
            'Choose the workspace master folder for imported settings',
            start,
        )
        if not master:
            return
        answer = QMessageBox.question(
            self,
            'Import Settings',
            'Importing will replace the configured workspace list and active '
            'workspace. All current tabs and log output will be discarded. '
            'Workspace files on disk will not be changed.\n\n'
            'Continue?',
        )
        if answer != QMessageBox.Yes:
            return
        try:
            import_settings(
                Path(selected),
                self._workspace_registry,
                master_folder=Path(master),
            )
        except (OSError, ValueError, yaml.YAMLError) as exc:
            QMessageBox.critical(self, 'Import Settings', str(exc))
            return
        self._apply_imported_workspace_settings()
        QMessageBox.information(
            self,
            'Import Settings',
            'Settings imported. Restart the GUI to apply general GUI '
            'configuration overrides.',
        )

    def _show_configuration_paths(self) -> None:
        if self._config_paths_dialog:
            self._config_paths_dialog.raise_()
            self._config_paths_dialog.activateWindow()
            return
        dialog = QDialog(self)
        dialog.setWindowTitle('Configuration Paths')
        layout = QVBoxLayout(dialog)
        legend = QLabel(
            'These paths are affected by the GUI. Manual editing is not '
            'recommended.'
        )
        legend.setWordWrap(True)
        layout.addWidget(legend)

        paths = QTableWidget()
        paths.setColumnCount(4)
        paths.setHorizontalHeaderLabels(['Setting', 'Path', 'Copy', 'Show'])
        paths.setEditTriggers(QAbstractItemView.NoEditTriggers)
        paths.setSelectionBehavior(QAbstractItemView.SelectRows)
        paths.setSelectionMode(QAbstractItemView.SingleSelection)
        paths.verticalHeader().setVisible(False)
        header = paths.horizontalHeader()
        header.setSectionResizeMode(0, QHeaderView.ResizeToContents)
        header.setSectionResizeMode(1, QHeaderView.Stretch)
        header.setSectionResizeMode(2, QHeaderView.ResizeToContents)
        header.setSectionResizeMode(3, QHeaderView.ResizeToContents)
        rows = self._configuration_path_rows()
        paths.setRowCount(len(rows))
        for row, (label, raw_path) in enumerate(rows):
            path = Path(raw_path).expanduser()
            paths.setItem(row, 0, QTableWidgetItem(label))
            paths.setItem(row, 1, QTableWidgetItem(str(path)))
            copy_button = QPushButton('Copy')
            show_button = QPushButton('Show')
            copy_button.clicked.connect(
                lambda _checked=False, item_path=path: (
                    self._copy_configuration_path(item_path)
                )
            )
            show_button.clicked.connect(
                lambda _checked=False, item_label=label, item_path=path: (
                    self._show_single_configuration_path_content(
                        item_label,
                        item_path,
                    )
                )
            )
            paths.setCellWidget(row, 2, copy_button)
            paths.setCellWidget(row, 3, show_button)
        paths.resizeColumnsToContents()
        layout.addWidget(paths)

        buttons = QDialogButtonBox(QDialogButtonBox.Close)
        copy_button = buttons.addButton(
            'Copy All Paths',
            QDialogButtonBox.ActionRole,
        )
        show_button = buttons.addButton(
            'Show All Contents',
            QDialogButtonBox.ActionRole,
        )
        copy_button.clicked.connect(self._copy_configuration_paths)
        show_button.clicked.connect(self._show_configuration_path_contents)
        buttons.rejected.connect(dialog.close)
        layout.addWidget(buttons)

        dialog.finished.connect(self._on_configuration_paths_closed)
        self._config_paths_dialog = dialog
        self._resize_configuration_paths_dialog(dialog, paths)
        dialog.show()

    def _resize_configuration_paths_dialog(
        self,
        dialog: QDialog,
        paths: QTableWidget,
    ) -> None:
        desired_width = (
            paths.verticalHeader().width()
            + sum(
                max(paths.columnWidth(column), paths.sizeHintForColumn(column))
                for column in range(paths.columnCount())
            )
            + paths.frameWidth() * 2
            + 56
        )
        screen = QGuiApplication.screenAt(self.frameGeometry().center())
        if screen is None:
            screen = QGuiApplication.primaryScreen()
        if screen is not None:
            available = screen.availableGeometry()
            desired_width = min(desired_width, max(760, available.width() - 80))
        dialog.resize(max(760, desired_width), 420)

    def _configuration_path_rows(self) -> list[tuple[str, Path]]:
        rows = user_configuration_paths(
            workspace_registry_path=self._workspace_registry.path,
            window_layout_template=self._window_layout_path_template,
        )
        active = self._workspace_registry.active_workspace()
        if active:
            if active.button_config:
                rows.append(
                    ('Active workspace button profile', Path(active.button_config))
                )
            if active.launch_config:
                rows.append(
                    ('Active workspace auto launch', Path(active.launch_config))
                )
        return rows

    def _configuration_paths_text(self) -> str:
        rows = self._configuration_path_rows()
        width = max(len(label) for label, _path in rows) if rows else 0
        return '\n'.join(
            f'{label:{width}}  {Path(path).expanduser()}'
            for label, path in rows
        )

    def _copy_configuration_paths(self) -> None:
        QApplication.clipboard().setText(self._configuration_paths_text())

    @staticmethod
    def _copy_configuration_path(path: Path) -> None:
        QApplication.clipboard().setText(str(Path(path).expanduser()))

    def _show_configuration_path_contents(self) -> None:
        self._open_configuration_path_contents_dialog(
            'Configuration Path Contents',
            self._configuration_path_contents_text(),
        )

    def _show_single_configuration_path_content(
        self,
        label: str,
        path: Path,
    ) -> None:
        path = Path(path).expanduser()
        text = (
            f'## {label}\n{path}\n\n'
            f'{self._read_configuration_path_content(path)}'
        )
        self._open_configuration_path_contents_dialog(
            f'{label} Contents',
            text,
        )

    def _open_configuration_path_contents_dialog(
        self,
        title: str,
        text: str,
    ) -> None:
        if self._config_path_contents_dialog:
            self._config_path_contents_dialog.close()
        dialog = QDialog(self)
        dialog.setWindowTitle(title)
        dialog.resize(900, 620)
        layout = QVBoxLayout(dialog)

        contents = QTextEdit()
        contents.setReadOnly(True)
        contents.setLineWrapMode(QTextEdit.NoWrap)
        contents.setPlainText(text)
        layout.addWidget(contents)

        buttons = QDialogButtonBox(QDialogButtonBox.Close)
        buttons.rejected.connect(dialog.close)
        layout.addWidget(buttons)

        dialog.finished.connect(self._on_configuration_path_contents_closed)
        self._config_path_contents_dialog = dialog
        dialog.show()

    def _configuration_path_contents_text(self) -> str:
        sections = []
        for label, raw_path in self._configuration_path_rows():
            path = Path(raw_path).expanduser()
            sections.append(f'## {label}\n{path}\n')
            sections.append(self._read_configuration_path_content(path))
        return '\n\n'.join(sections)

    def _read_configuration_path_content(self, path: Path) -> str:
        path_text = str(path)
        if '{workspace}' in path_text or '{workspace_slug}' in path_text:
            template_parent = path.parent
            if template_parent.exists():
                return self._format_directory_listing(template_parent)
            return '(template path; directory has not been created yet)'
        if path.is_dir():
            return self._format_directory_listing(path)
        if path.is_file():
            try:
                text = path.read_text(encoding='utf-8')
            except (OSError, UnicodeDecodeError) as exc:
                return f'(cannot read file: {exc})'
            limit = 200000
            if len(text) > limit:
                return text[:limit] + '\n\n(truncated)'
            return text or '(empty file)'
        return '(path has not been created yet)'

    @staticmethod
    def _format_directory_listing(path: Path) -> str:
        try:
            children = sorted(path.iterdir(), key=lambda item: item.name.lower())
        except OSError as exc:
            return f'(cannot list directory: {exc})'
        if not children:
            return '(empty directory)'
        return '\n'.join(
            f'{"[dir] " if child.is_dir() else "      "}{child.name}'
            for child in children
        )

    def _on_configuration_paths_closed(self, _result: int) -> None:
        self._config_paths_dialog = None

    def _on_configuration_path_contents_closed(self, _result: int) -> None:
        self._config_path_contents_dialog = None

    def _apply_view_action_visibility(self) -> None:
        recording = self._view_actions.get('recording')
        scripts = self._view_actions.get('scripts')
        commands = self._view_actions.get('commands')
        remote_master = self._view_actions.get('remote_master')
        self._set_recording_controls_visible(
            bool(recording and recording.isChecked())
        )
        self._set_script_controls_visible(
            bool(scripts and scripts.isChecked())
        )
        self._set_command_controls_visible(
            bool(commands and commands.isChecked())
        )
        self._set_remote_master_controls_visible(
            bool(remote_master and remote_master.isChecked())
        )

    def _set_recording_controls_visible(self, visible: bool) -> None:
        controls = getattr(self, 'recording_controls', None)
        if controls is not None:
            controls.setVisible(bool(visible))

    def _set_script_controls_visible(self, visible: bool) -> None:
        controls = getattr(self, 'script_controls', None)
        if controls is not None:
            controls.setVisible(bool(visible))

    def _set_command_controls_visible(self, visible: bool) -> None:
        controls = getattr(self, 'command_controls', None)
        if controls is not None:
            controls.setVisible(bool(visible))

    def _set_remote_master_controls_visible(self, visible: bool) -> None:
        controls = getattr(self, 'ros_master_controls', None)
        if controls is not None:
            controls.setVisible(bool(visible))

    def _show_about_dialog(self) -> None:
        dialog = QDialog(self)
        dialog.setWindowTitle('About Mobipick Labs Control')
        dialog.setModal(True)

        layout = QVBoxLayout(dialog)
        layout.setContentsMargins(24, 20, 24, 20)
        layout.setSpacing(14)

        logo_label = QLabel()
        logo_label.setAlignment(Qt.AlignCenter)
        logo = QPixmap(str(PROJECT_ROOT / 'images' / 'dfki_logo.svg'))
        if logo.isNull():
            logo_label.setText('DFKI')
        else:
            logo_label.setPixmap(logo.scaledToWidth(220, Qt.SmoothTransformation))
        layout.addWidget(logo_label)

        details = QLabel(
            '<h2>Mobipick Labs Docker GUI</h2>'
            f'<p>Version: {html.escape(get_version())}</p>'
            '<p>Maintainer:<br>'
            'Mobipick Labs<br>'
            '<a href="mailto:mobipick-labs@dfki.de">'
            'mobipick-labs@dfki.de</a></p>'
            '<p>Mobipick Labs:<br>'
            '<a href="https://github.com/DFKI-NI/mobipick_labs">'
            'https://github.com/DFKI-NI/mobipick_labs</a></p>'
        )
        details.setTextFormat(Qt.RichText)
        details.setOpenExternalLinks(False)
        details.setTextInteractionFlags(Qt.TextBrowserInteraction)
        details.setAlignment(Qt.AlignCenter)
        details.linkActivated.connect(self._open_about_link)
        layout.addWidget(details)

        buttons = QDialogButtonBox(QDialogButtonBox.Ok)
        buttons.accepted.connect(dialog.accept)
        layout.addWidget(buttons)

        dialog.exec_()

    def _open_about_link(self, link: str) -> None:
        if not open_external_url(link):
            QMessageBox.warning(
                self,
                'About',
                'Unable to open the selected link.',
            )

    def _open_documentation_dialog(self) -> None:
        if self._documentation_dialog:
            self._documentation_dialog.raise_()
            self._documentation_dialog.activateWindow()
            return
        dialog = DocumentationDialog(
            PROJECT_ROOT / 'gui_user_documentation.md',
            self,
        )
        dialog.finished.connect(self._on_documentation_dialog_closed)
        self._documentation_dialog = dialog
        dialog.show()

    def _on_documentation_dialog_closed(self, _result: int) -> None:
        self._documentation_dialog = None

    def _open_bug_report_dialog(self) -> None:
        if self._bug_report_dialog:
            self._bug_report_dialog.raise_()
            self._bug_report_dialog.activateWindow()
            return
        dialog = BugReportDialog(self._build_bug_report_context, self)
        dialog.finished.connect(self._on_bug_report_dialog_closed)
        self._bug_report_dialog = dialog
        dialog.show()

    def _on_bug_report_dialog_closed(self, _result: int) -> None:
        self._bug_report_dialog = None

    def _build_bug_report_context(self) -> dict:
        active = self._workspace_registry.active_workspace()
        workspaces = []
        for workspace in self._workspace_registry.workspaces:
            try:
                directory = str(workspace.directory)
            except OSError:
                directory = workspace.path
            workspaces.append(
                {
                    'name': workspace.name,
                    'path': directory,
                    'extends': list(workspace.extends),
                    'image': workspace.image,
                    'runtime_built': self._workspace_registry.is_runtime_built(
                        workspace
                    ),
                    'active': workspace.name == self._workspace_registry.active,
                }
            )

        return {
            'generated_at': datetime.now().astimezone().isoformat(
                timespec='seconds'
            ),
            'gui_version': get_version(),
            'selected_workspace': (
                {
                    'name': active.name,
                    'path': str(active.directory),
                    'extends': list(active.extends),
                    'image': active.image,
                    'runtime_built': self._workspace_registry.is_runtime_built(
                        active
                    ),
                    'active': True,
                }
                if active
                else {}
            ),
            'selected_image': self._selected_image or '(none)',
            'active_workspace_name': self._workspace_registry.active,
            'workspace_match': self._bug_report_workspace_match(),
            'host_workspace_mount': (
                'enabled'
                if self._selected_image
                and self._image_supports_host_workspaces(self._selected_image)
                else 'disabled'
            ),
            'workspace_registry_path': str(self._workspace_registry.path),
            'workspace_master_folder': self._workspace_registry.master_folder,
            'workspaces': workspaces,
            'log_tab_text': self._bug_report_log_tab_text(),
        }

    def _bug_report_workspace_match(self) -> str:
        image_ref = self._selected_image or ''
        active_name = self._workspace_registry.active
        if not image_ref:
            return 'no Docker image selected'
        compatible = self._image_compatible_with_workspace(
            image_ref,
            active_name,
        )
        if compatible is True:
            return 'workspace match'
        if compatible is False:
            return 'image profile does not list the active workspace'
        if active_name and not self._image_supports_host_workspaces(image_ref):
            return 'selected image uses its baked workspace'
        if active_name:
            return 'unknown (no explicit compatibility profile)'
        return 'not applicable (Docker image default workspace)'

    def _bug_report_log_tab_text(self) -> str:
        tab = self.tasks.get('log')
        if not tab:
            return ''
        return tab.output.toPlainText()

    # ---------- ROS workspace management ----------

    def _workspace_button_config_path(self) -> str | None:
        workspace = self._workspace_registry.active_workspace()
        if workspace and workspace.button_config:
            return workspace.button_config
        return (
            str(CONFIG.get('buttons', {}).get('config_file') or '')
            or str(BUTTON_CONFIG_FILE)
        )

    def _workspace_launch_config_path(self) -> str | None:
        workspace = self._workspace_registry.active_workspace()
        if workspace and workspace.launch_config:
            return workspace.launch_config
        return None

    def _resolved_button_config_path(self) -> Path:
        raw_path = Path(self._workspace_button_config_path() or BUTTON_CONFIG_FILE)
        raw_path = raw_path.expanduser()
        if not raw_path.is_absolute():
            raw_path = PROJECT_ROOT / raw_path
        return raw_path

    def _button_profile_save_path(self, source_path: Path) -> Path:
        workspace = self._workspace_registry.active_workspace()
        if workspace is None:
            return writable_button_config_path(source_path)
        return writable_workspace_button_config_path(source_path, workspace.name)

    def _workspace_docker_cp_config_path(self) -> Path:
        workspace = self._workspace_registry.active_workspace()
        if workspace is None:
            return writable_docker_cp_config_path()
        return writable_workspace_docker_cp_config_path(workspace.name)

    def _open_button_profile_dialog(self) -> None:
        if self._workspace_processes_running():
            QMessageBox.warning(
                self,
                'Toolbar Buttons',
                'Stop running workspace processes before editing toolbar buttons.',
            )
            return
        source_path = self._resolved_button_config_path()
        save_path = self._button_profile_save_path(source_path)
        dialog = ButtonProfileDialog(
            self._button_layout_for_editor(),
            source_path,
            save_path,
            self,
        )
        if dialog.exec_() != QDialog.Accepted:
            return
        button_layout = dialog.button_layout()
        try:
            saved_path = save_button_layout(save_path, button_layout)
            self._remember_button_profile_path(
                saved_path,
                source_path,
                button_layout,
            )
        except (OSError, ValueError) as exc:
            QMessageBox.warning(
                self,
                'Toolbar Buttons',
                f'Failed to save toolbar button profile:\n{exc}',
            )
            return

        self._reset_workspace_tabs()
        self._reload_workspace_profile()
        self._create_workspace_tabs()
        self._apply_env_to_all_tabs()
        self._log_info(f'saved toolbar button profile to {saved_path}')

    def _remember_button_profile_path(
        self,
        saved_path: Path,
        source_path: Path,
        entries: list[dict],
    ) -> None:
        workspace = self._workspace_registry.active_workspace()
        if workspace is not None:
            changed = False
            if saved_path != source_path:
                workspace.button_config = str(saved_path)
                changed = True
            if workspace.sim_command and self._entry_command(entries, 'sim'):
                workspace.sim_command = ''
                changed = True
            if changed:
                self._workspace_registry.save()
            if self._workspace_dialog:
                self._workspace_dialog.refresh(self._workspace_registry.active)
            return
        if saved_path == source_path:
            return
        save_user_config_update({
            'buttons': {
                'config_file': str(saved_path),
            },
        })

    @staticmethod
    def _entry_command(entries: list[dict], key: str) -> str:
        for entry in entries:
            if str(entry.get('key') or '').strip() == key:
                return str(entry.get('command') or '').strip()
        return ''

    def _button_layout_for_editor(self) -> list[dict]:
        entries = copy.deepcopy(self._button_layout)
        effective_commands = {
            'sim': self._workspace_sim_command(),
            'tables': self._tables_demo_command(),
            'rviz': self._rviz_command(),
            'rqt': self._rqt_tables_command(),
        }
        for entry in entries:
            key = str(entry.get('key') or '').strip()
            command = effective_commands.get(key)
            if command:
                entry['command'] = command
        return entries

    def _workspace_runtime_env(
        self,
        workspace_name: str | None = None,
        *,
        force_host_workspace: bool = False,
    ) -> dict[str, str]:
        requested_name = (
            self._workspace_registry.active
            if workspace_name is None
            else workspace_name
        )
        if (
            requested_name
            and not force_host_workspace
            and not self._image_supports_host_workspaces(self._selected_image)
        ):
            workspace_name = ''
        return self._workspace_registry.runtime_environment(
            empty_mount_source=self._empty_workspace_dir,
            workspace_name=workspace_name,
        )

    @staticmethod
    def _normalize_image_profiles(entries) -> list[dict]:
        profiles: list[dict] = []
        if not isinstance(entries, list):
            return profiles
        for entry in entries:
            if not isinstance(entry, dict):
                continue
            ref = str(entry.get('ref') or '').strip()
            match = str(entry.get('match') or '').strip()
            if not ref and not match:
                continue
            user = str(
                entry.get('user')
                or entry.get('container_user')
                or ''
            ).strip()
            compatible = entry.get('compatible_workspaces', [])
            if isinstance(compatible, str):
                compatible_items = [
                    item.strip()
                    for item in compatible.split(',')
                    if item.strip()
                ]
            elif isinstance(compatible, list):
                compatible_items = [
                    str(item).strip()
                    for item in compatible
                    if str(item).strip()
                ]
            else:
                compatible_items = []
            profiles.append(
                {
                    'ref': ref,
                    'match': match,
                    'user': user,
                    'supports_host_workspaces': entry.get(
                        'supports_host_workspaces',
                        entry.get('host_workspaces'),
                    ),
                    'compatible_workspaces': compatible_items,
                    'workdir': str(entry.get('workdir') or '').strip(),
                    'entrypoint': str(
                        entry.get('entrypoint')
                        or '/usr/local/bin/entrypoint_user.sh'
                    ).strip(),
                    'description': str(
                        entry.get('description') or ''
                    ).strip(),
                }
            )
        return profiles

    def _image_profile(self, image_ref: str | None = None) -> dict | None:
        ref = str(image_ref if image_ref is not None else self._selected_image)
        ref = ref.strip()
        if not ref:
            return None
        for profile in self._image_profiles:
            if profile.get('ref') == ref:
                return profile
        for profile in self._image_profiles:
            pattern = profile.get('match') or ''
            if pattern and fnmatchcase(ref, pattern):
                return profile
        return None

    def _image_supports_host_workspaces(
        self,
        image_ref: str | None = None,
    ) -> bool:
        profile = self._image_profile(image_ref)
        value = (
            profile.get('supports_host_workspaces')
            if profile
            else self._images_cfg.get('default_supports_host_workspaces')
        )
        if isinstance(value, str):
            return value.strip().lower() in {'1', 'true', 'yes', 'on'}
        return bool(value)

    def _image_container_user(self, image_ref: str | None = None) -> str:
        profile = self._image_profile(image_ref)
        raw_user = (
            profile.get('user')
            if profile and profile.get('user')
            else self._images_cfg.get('default_user', 'root')
        )
        user = str(raw_user or 'root').strip()
        if user.lower() == 'host':
            return str(
                CONFIG['process']['compose_run_env'].get(
                    'MOBIPICK_HOST_USER',
                    '',
                )
            ).strip() or str(os.environ.get('USER') or 'root')
        if user.lower() in {'', 'root'}:
            return 'root'
        return user

    def _image_entrypoint(self, image_ref: str | None = None) -> str:
        profile = self._image_profile(image_ref)
        if profile and profile.get('entrypoint'):
            return str(profile['entrypoint'])
        return '/usr/local/bin/entrypoint_user.sh'

    def _image_workdir(self, image_ref: str | None = None) -> str:
        profile = self._image_profile(image_ref)
        if profile and profile.get('workdir'):
            return str(profile['workdir'])
        return ''

    def _image_runtime_env(
        self,
        workspace_env: dict[str, str] | None = None,
        *,
        image_ref: str | None = None,
    ) -> dict[str, str]:
        workspace_env = workspace_env or self._workspace_runtime_env()
        image = image_ref or self._selected_image
        workdir = self._image_workdir(image)
        if not workdir:
            workdir = workspace_env.get('MOBIPICK_WORKSPACE_PATH') or '/tmp'
        return {
            'MOBIPICK_CONTAINER_USER': self._image_container_user(
                image
            ),
            'MOBIPICK_CONTAINER_ENTRYPOINT': self._image_entrypoint(
                image
            ),
            'MOBIPICK_CONTAINER_WORKDIR': workdir,
        }

    def _image_compatible_with_workspace(
        self,
        image_ref: str,
        workspace_name: str,
    ) -> bool | None:
        profile = self._image_profile(image_ref)
        if not profile:
            return None
        compatible = profile.get('compatible_workspaces') or []
        if '*' in compatible:
            return True
        if workspace_name:
            return workspace_name in compatible
        return (
            ''
            in compatible
            or 'Docker image default' in compatible
            or 'image default' in compatible
        )

    @staticmethod
    def _normalize_workspace_mismatch_exceptions(value) -> list[dict[str, str]]:
        if not isinstance(value, list):
            return []
        normalized: list[dict[str, str]] = []
        for item in value:
            if not isinstance(item, dict):
                continue
            image = str(item.get('image') or '').strip()
            workspace = str(item.get('workspace') or '').strip()
            if not image:
                continue
            if not workspace:
                workspace = 'Docker image default'
            entry = {'image': image, 'workspace': workspace}
            if entry not in normalized:
                normalized.append(entry)
        return normalized

    def _workspace_mismatch_workspace_key(self) -> str:
        return (
            str(self._workspace_registry.active or '').strip()
            or 'Docker image default'
        )

    def _workspace_mismatch_exception_entry(self) -> dict[str, str] | None:
        image = str(self._selected_image or '').strip()
        if not image:
            return None
        workspace = self._workspace_mismatch_workspace_key()
        return {'image': image, 'workspace': workspace}

    def _workspace_mismatch_exception_exists(self) -> bool:
        entry = self._workspace_mismatch_exception_entry()
        if not entry:
            return False
        return entry in self._workspace_mismatch_session_exceptions

    def _workspace_mismatch_warning_reason(self) -> str:
        image = str(self._selected_image or '').strip()
        if not image:
            return ''
        workspace = str(self._workspace_registry.active or '').strip()
        if self._image_compatible_with_workspace(image, workspace) is True:
            return ''
        if not self._image_supports_host_workspaces(image):
            return (
                'The selected Docker image does not mount host workspaces. '
                'Commands will run against the workspace baked into the image.'
            )
        if not workspace:
            return (
                'The selected Docker image does not declare a workspace match '
                'for Docker image default.'
            )
        return (
            'The selected Docker image does not declare a workspace match for '
            'the active ROS 1 workspace.'
        )

    def _remember_workspace_mismatch_exception(self) -> None:
        entry = self._workspace_mismatch_exception_entry()
        if not entry:
            return
        exceptions = list(self._workspace_mismatch_session_exceptions)
        if entry not in exceptions:
            exceptions.append(entry)
        self._workspace_mismatch_session_exceptions = exceptions
        self._workspace_warning_cfg['silenced_exceptions'] = exceptions
        save_user_config_update({
            'workspace_mismatch_warning': {
                'silenced_exceptions': exceptions,
            },
        })

    def _mark_current_image_workspace_match(self) -> None:
        image = str(self._selected_image or '').strip()
        workspace = self._workspace_mismatch_workspace_key()
        self._mark_image_workspace_match(image, workspace)

    def _mark_image_workspace_match(self, image: str, workspace: str) -> None:
        image = str(image or '').strip()
        workspace = str(workspace or '').strip() or 'Docker image default'
        compatible_workspace = '' if workspace == 'Docker image default' else workspace
        if not image:
            return
        if self._image_compatible_with_workspace(
            image,
            compatible_workspace,
        ) is True:
            return
        profiles = copy.deepcopy(self._images_cfg.get('profiles', []) or [])
        match_profile = self._image_profile(image) or {}
        updated = False
        for profile in profiles:
            if not isinstance(profile, dict):
                continue
            if str(profile.get('ref') or '').strip() != image:
                continue
            compatible = self._compatible_workspace_list(
                profile.get('compatible_workspaces')
            )
            if workspace not in compatible:
                compatible.append(workspace)
            profile['compatible_workspaces'] = compatible
            updated = True
            break
        if not updated:
            profile = {
                'ref': image,
                'user': match_profile.get(
                    'user',
                    self._images_cfg.get('default_user', 'root'),
                ),
                'supports_host_workspaces': match_profile.get(
                    'supports_host_workspaces',
                    self._images_cfg.get('default_supports_host_workspaces'),
                ),
                'compatible_workspaces': [workspace],
            }
            for key in ('workdir', 'entrypoint', 'description'):
                value = match_profile.get(key)
                if value not in (None, ''):
                    profile[key] = value
            profiles.append(profile)

        save_user_config_update({'images': {'profiles': profiles}})
        self._images_cfg['profiles'] = profiles
        CONFIG['images']['profiles'] = profiles
        self._image_profiles = self._normalize_image_profiles(
            self._images_cfg.get('profiles', [])
        )
        self._refresh_image_combo_labels()
        self._log_info(
            f'marked {image} as a workspace match for {workspace}'
        )

    def _open_workspace_match_editor(self) -> None:
        images = list(self._image_choices)
        if not images:
            QMessageBox.information(
                self,
                'Workspace Matches',
                'No local Docker images are available to configure.',
            )
            return
        dialog = WorkspaceMatchDialog(
            images,
            [workspace.name for workspace in self._workspace_registry.workspaces],
            self._workspace_match_map(images),
            self,
        )
        if dialog.exec_() != QDialog.Accepted:
            return
        try:
            self._save_workspace_match_map(dialog.matches())
        except Exception as exc:
            QMessageBox.warning(
                self,
                'Workspace Matches',
                f'Failed to save workspace matches:\n{exc}',
            )
            return
        self._refresh_image_combo_labels()
        self._log_info('workspace matches updated')

    def _workspace_match_map(self, images: list[str]) -> dict[str, list[str]]:
        result: dict[str, list[str]] = {}
        for image in images:
            profile = self._image_profile(image) or {}
            compatible = self._compatible_workspace_list(
                profile.get('compatible_workspaces')
            )
            values = []
            for item in compatible:
                if item in {'', 'image default'}:
                    item = 'Docker image default'
                if item not in values:
                    values.append(item)
            result[image] = values
        return result

    def _save_workspace_match_map(
        self,
        matches: dict[str, list[str]],
    ) -> None:
        profiles = copy.deepcopy(self._images_cfg.get('profiles', []) or [])
        for image, workspaces in matches.items():
            profiles = self._profiles_with_workspace_matches(
                profiles,
                image,
                workspaces,
            )
        save_user_config_update({'images': {'profiles': profiles}})
        self._images_cfg['profiles'] = profiles
        CONFIG['images']['profiles'] = profiles
        self._image_profiles = self._normalize_image_profiles(profiles)

    def _profiles_with_workspace_matches(
        self,
        profiles: list,
        image: str,
        workspaces: list[str],
    ) -> list:
        image = str(image or '').strip()
        if not image:
            return profiles
        compatible = [
            str(workspace or '').strip() or 'Docker image default'
            for workspace in workspaces
            if str(workspace or '').strip()
        ]
        compatible = list(dict.fromkeys(compatible))
        match_profile = self._image_profile(image) or {}
        for profile in profiles:
            if not isinstance(profile, dict):
                continue
            if str(profile.get('ref') or '').strip() != image:
                continue
            profile['compatible_workspaces'] = compatible
            return profiles
        profile = {
            'ref': image,
            'user': match_profile.get(
                'user',
                self._images_cfg.get('default_user', 'root'),
            ),
            'supports_host_workspaces': match_profile.get(
                'supports_host_workspaces',
                self._images_cfg.get('default_supports_host_workspaces'),
            ),
            'compatible_workspaces': compatible,
        }
        for key in ('workdir', 'entrypoint', 'description'):
            value = match_profile.get(key)
            if value not in (None, ''):
                profile[key] = value
        profiles.append(profile)
        return profiles

    def _mark_workspace_build_image_match(self, workspace: RosWorkspace) -> bool:
        try:
            self._mark_image_workspace_match(
                self._selected_image,
                workspace.name,
            )
        except Exception as exc:
            QMessageBox.warning(
                self,
                'Build Workspace',
                f'Failed to save image/workspace match:\n{exc}',
            )
            return False
        return True

    @staticmethod
    def _compatible_workspace_list(value) -> list[str]:
        if isinstance(value, str):
            return [
                item.strip()
                for item in value.split(',')
                if item.strip()
            ]
        if isinstance(value, list):
            return [
                str(item).strip()
                for item in value
                if str(item).strip()
            ]
        return []

    def _confirm_workspace_mismatch_warning(self, action_label: str) -> bool:
        reason = self._workspace_mismatch_warning_reason()
        if not reason or self._workspace_mismatch_exception_exists():
            return True

        workspace = self._workspace_mismatch_workspace_key()
        image = self._selected_image or '(none)'
        message = QMessageBox(self)
        message.setIcon(QMessageBox.Warning)
        message.setWindowTitle('Workspace/Image Mismatch')
        message.setText(
            f'{action_label} is about to run without a workspace match.'
        )
        message.setInformativeText(
            f'Active workspace: {workspace}\n'
            f'Docker image: {image}\n\n'
            f'{reason}\n\n'
            'Continue only if this image/workspace combination is intentional.'
        )
        continue_button = message.addButton('Continue', QMessageBox.AcceptRole)
        mark_match_button = message.addButton(
            'Mark as Workspace Match',
            QMessageBox.AcceptRole,
        )
        message.addButton(QMessageBox.Cancel)
        remember = QCheckBox(
            'Do not warn again for this image/workspace pair'
        )
        message.setCheckBox(remember)
        message.setDefaultButton(QMessageBox.Cancel)
        message.exec_()
        clicked = message.clickedButton()
        if clicked == mark_match_button:
            try:
                self._mark_current_image_workspace_match()
            except Exception as exc:
                QMessageBox.warning(
                    self,
                    'Workspace Match',
                    f'Failed to save workspace match:\n{exc}',
                )
                return False
            return True
        if clicked != continue_button:
            return False
        if remember.isChecked():
            try:
                self._remember_workspace_mismatch_exception()
            except Exception as exc:
                self._append_gui_html(
                    'log',
                    '<i>Failed to save workspace mismatch warning '
                    f'exception: {html.escape(str(exc))}</i>',
                )
        else:
            entry = self._workspace_mismatch_exception_entry()
            if entry and entry not in self._workspace_mismatch_session_exceptions:
                self._workspace_mismatch_session_exceptions.append(entry)
        return True

    def _image_choice_label(self, image_ref: str) -> str:
        active_name = self._workspace_registry.active
        compatible = self._image_compatible_with_workspace(
            image_ref,
            active_name,
        )
        if compatible is True:
            return f'{image_ref}  [workspace match]'
        if active_name and not self._image_supports_host_workspaces(image_ref):
            return f'{image_ref}  [image default only]'
        return image_ref

    def _image_choice_tooltip(self, image_ref: str) -> str:
        profile = self._image_profile(image_ref)
        parts = [image_ref]
        if profile and profile.get('description'):
            parts.append(str(profile['description']))
        user = self._image_container_user(image_ref)
        parts.append(f'Container user: {user}')
        if self._image_supports_host_workspaces(image_ref):
            parts.append('Host workspace mount: enabled')
        else:
            parts.append('Host workspace mount: disabled; uses image workspace')
        return '\n'.join(parts)

    def _decorate_image_combo_item(self, index: int, image_ref: str) -> None:
        self.image_combo.setItemData(
            index,
            self._image_choice_tooltip(image_ref),
            Qt.ToolTipRole,
        )
        self.image_combo.setItemData(index, None, Qt.ForegroundRole)
        if self._image_compatible_with_workspace(
            image_ref,
            self._workspace_registry.active,
        ) is True:
            self.image_combo.setItemData(
                index,
                QColor('#1b7f3a'),
                Qt.ForegroundRole,
            )
        elif (
            self._workspace_registry.active
            and not self._image_supports_host_workspaces(image_ref)
        ):
            self.image_combo.setItemData(
                index,
                QColor('#9a6700'),
                Qt.ForegroundRole,
            )

    def _refresh_image_combo_labels(self) -> None:
        if not hasattr(self, 'image_combo') or not self._image_choices:
            return
        self.image_combo.blockSignals(True)
        for index, image_ref in enumerate(self._image_choices):
            self.image_combo.setItemText(
                index,
                self._image_choice_label(image_ref),
            )
            self.image_combo.setItemData(index, image_ref)
            self._decorate_image_combo_item(index, image_ref)
        if self._selected_image in self._image_choices:
            self.image_combo.setCurrentIndex(
                self._image_choices.index(self._selected_image)
            )
        self.image_combo.blockSignals(False)
        self.image_combo.setToolTip(
            self._image_choice_tooltip(self._selected_image)
            if self._selected_image
            else 'No image selected'
        )

    def _workspace_image(self, name: str | None) -> str:
        return self._workspace_registry.image_for(
            name,
            str(self._images_cfg.get('default', '') or ''),
        )

    def _workspace_match_image(
        self,
        name: str | None,
        choices: list[str] | None = None,
    ) -> str:
        workspace_name = str(name or '').strip()
        image_choices = choices if choices is not None else self._image_choices
        for image_ref in image_choices:
            if self._image_compatible_with_workspace(
                image_ref,
                workspace_name,
            ) is True:
                return image_ref
        return ''

    def _populate_workspace_combo(self) -> None:
        selected = self._workspace_registry.active
        self.workspace_combo.blockSignals(True)
        self.workspace_combo.clear()
        self.workspace_combo.addItem('Docker image default', '')
        for workspace in self._workspace_registry.workspaces:
            self.workspace_combo.addItem(
                f'{workspace.name}  [{workspace.directory}]',
                workspace.name,
            )
        index = self.workspace_combo.findData(selected)
        self.workspace_combo.setCurrentIndex(index if index >= 0 else 0)
        self.workspace_combo.blockSignals(False)
        active = self._workspace_registry.active_workspace()
        host_workspace_enabled = self._image_supports_host_workspaces(
            self._selected_image
        )
        if active:
            setup_status = (
                'built'
                if self._workspace_registry.is_runtime_built(active)
                else 'not built; base ROS remains available'
            )
            mount_status = (
                'host workspace will be mounted'
                if host_workspace_enabled
                else 'selected image uses its baked Docker workspace'
            )
            self.workspace_combo.setToolTip(
                f'{active.directory}\n{setup_status}\n{mount_status}'
            )
        else:
            self.workspace_combo.setToolTip(
                'Use the catkin workspace bundled in the selected Docker image.'
            )
        self._refresh_image_combo_labels()

    def _on_workspace_changed(self, index: int) -> None:
        name = str(self.workspace_combo.itemData(index) or '')
        if name == self._workspace_registry.active:
            return
        self._activate_workspace(name)

    def _workspace_processes_running(self) -> bool:
        return bool(
            self._roscore_running_cached
            or self._sim_running_cached
            or self._terminal_running_cached
            or self._recording_is_active()
            or any(tab.is_running() for tab in self.tasks.values())
        )

    def _activate_workspace(self, name: str) -> bool:
        if name == self._workspace_registry.active:
            self._populate_workspace_combo()
            return True
        workspace = self._workspace_registry.get(name) if name else None
        if name and workspace is None:
            QMessageBox.warning(
                self,
                'ROS 1 Workspace',
                f'Workspace "{name}" is not configured.',
            )
            self._populate_workspace_combo()
            return False
        if self._workspace_processes_running():
            QMessageBox.warning(
                self,
                'ROS 1 Workspace',
                'Stop running workspace processes before switching.',
            )
            self._populate_workspace_combo()
            return False

        target_image = (
            self._workspace_match_image(name)
            or self._workspace_image(name)
        )
        if target_image and target_image not in self._image_choices:
            QMessageBox.warning(
                self,
                'ROS 1 Workspace',
                f'The Docker image configured for this workspace is not '
                f'installed:\n\n{target_image}\n\n'
                'Install it or update the workspace settings before switching.',
            )
            self._populate_workspace_combo()
            return False

        current_name = self._workspace_registry.active or 'Docker image default'
        target_name = name or 'Docker image default'
        answer = QMessageBox.question(
            self,
            'Switch ROS 1 Workspace',
            f'Switch from "{current_name}" to "{target_name}"?\n\n'
            'All current tabs and their log output will be discarded and '
            'recreated for the selected workspace.\n\n'
            f'Docker image: {target_image or "(none)"}',
            QMessageBox.Yes | QMessageBox.Cancel,
            QMessageBox.Cancel,
        )
        if answer != QMessageBox.Yes:
            self._populate_workspace_combo()
            if self._workspace_dialog:
                self._workspace_dialog.refresh(
                    self._workspace_registry.active
                )
            return False

        previous_name = self._workspace_registry.active
        self._workspace_registry.active = name
        try:
            self._workspace_registry.save()
        except (OSError, ValueError) as exc:
            self._workspace_registry.active = previous_name
            QMessageBox.critical(self, 'ROS 1 Workspace', str(exc))
            self._populate_workspace_combo()
            return False

        self._reset_workspace_tabs()
        self._reload_workspace_profile()
        self._reload_window_layout_for_workspace()
        if target_image:
            self._select_image(target_image, log_selection=False)
        self._create_workspace_tabs()
        self._apply_env_to_all_tabs()

        if workspace:
            self._recording_workspace_name = workspace.name
            self._log_info(
                f'active ROS 1 workspace: {workspace.name} '
                f'({workspace.directory})'
            )
        else:
            configured_name = self._recording_cfg.get(
                'workspace_name',
                'workspace',
            )
            self._recording_workspace_name = self._normalize_workspace_name(
                configured_name
            )
            self._log_info('using the Docker image default ROS 1 workspace')
        self._populate_workspace_combo()
        if self._workspace_dialog:
            self._workspace_dialog.refresh(name)
        return True

    def _reset_workspace_tabs(self) -> None:
        self._cancel_auto_launch_timers()
        self._cancel_recording_schedule()
        self._auto_launch_running = False
        self._auto_launch_stopping = False
        self._auto_launch_active_keys.clear()
        self._auto_launch_run_count = 0
        self._script_active_tab_key = None
        self._terminal_stream_tab_key = None
        self._custom_counter = 0
        self._last_log_origin.clear()
        self._synced_container_refs.clear()
        self._toggle_states.clear()

        for tab in list(self.tasks.values()):
            index = self.tabs.indexOf(tab.output)
            if index >= 0:
                self.tabs.removeTab(index)
            tab.proc.deleteLater()
            tab.output.deleteLater()
        self.tasks.clear()

    def _create_workspace_tabs(self) -> None:
        self._ensure_tab('roscore', 'Roscore', closable=False)
        for key in self._config_button_order:
            config = self._config_buttons[key]
            self._ensure_tab(
                key,
                str(config.get('label') or key),
                closable=False,
            )
        self._ensure_tab('sim', 'Sim', closable=False)
        self._ensure_tab('tables', 'Tables Demo', closable=False)
        self._ensure_tab('rviz', 'RViz', closable=False)
        self._ensure_tab('rqt', 'RQt Tables', closable=False)
        self._ensure_tab('log', 'Log', closable=False)

    def _refresh_launch_plan_settings(self) -> None:
        self._launch_retry_ms = max(
            0,
            int(self._launch_plan.get('retry_delay_ms', 750) or 0),
        )
        self._launch_max_retry = max(
            0,
            int(self._launch_plan.get('max_retry_attempts', 6) or 0),
        )
        if hasattr(self, '_window_layout_cfg'):
            self._window_layout_delay_ms = self._compute_window_layout_delay_ms()
            manager = getattr(self, '_window_layout_manager', None)
            if manager is not None:
                manager.set_apply_delay_ms(self._window_layout_delay_ms)

    def _reload_workspace_profile(self) -> None:
        old_keys = list(self._config_button_order)
        for key in old_keys:
            button = self._button_widgets.pop(key, None)
            if button:
                self._top_controls_layout.removeWidget(button)
                button.deleteLater()
        self._button_layout = load_button_layout(
            self._workspace_button_config_path()
        )
        self._launch_plan = load_launch_sequence_plan(
            self._workspace_button_config_path(),
            self._workspace_launch_config_path(),
        )
        self._docker_cp_config = load_docker_cp_config(
            self._workspace_docker_cp_config_path()
        )
        self._synced_container_refs.clear()
        self._refresh_launch_plan_settings()
        terminal_index = self._top_controls_layout.indexOf(
            self.terminal_button
        )
        self._build_configurable_buttons(
            self._top_controls_layout,
            insert_at=terminal_index,
        )
        self._auto_launch_base_tooltip = str(
            self._launch_plan.get('button', {}).get('tooltip') or ''
        )
        if getattr(self, 'record_checkbox', None) and self.record_checkbox.isChecked():
            state = 'active' if self._recording_is_active() else 'armed'
        else:
            state = 'off'
        self._set_auto_launch_recording_hint(state)
        self._update_buttons()

    def _open_workspace_manager(self) -> None:
        if self._workspace_dialog:
            self._workspace_dialog.raise_()
            self._workspace_dialog.activateWindow()
            return
        dialog = WorkspaceManagerDialog(
            self._workspace_registry,
            self,
            replace_allowed=lambda: not self._workspace_processes_running(),
            image_choices=self._image_choices,
            image_workspace_path=self._image_workdir(self._selected_image),
        )
        dialog.workspace_activated.connect(self._activate_workspace)
        dialog.build_requested.connect(self._build_workspace)
        dialog.settings_imported.connect(
            self._apply_imported_workspace_settings
        )
        dialog.finished.connect(self._on_workspace_manager_closed)
        self._workspace_dialog = dialog
        dialog.show()

    def _apply_imported_workspace_settings(self) -> None:
        self._reset_workspace_tabs()
        self._reload_workspace_profile()
        target_image = (
            self._workspace_match_image(self._workspace_registry.active)
            or self._workspace_image(self._workspace_registry.active)
        )
        if target_image and target_image in self._image_choices:
            self._select_image(target_image, log_selection=False)
        self._create_workspace_tabs()
        self._apply_env_to_all_tabs()
        active = self._workspace_registry.active_workspace()
        self._recording_workspace_name = (
            active.name
            if active
            else self._normalize_workspace_name(
                self._recording_cfg.get('workspace_name', 'workspace')
            )
        )
        self._populate_workspace_combo()

    def _on_workspace_manager_closed(self, _result: int) -> None:
        self._workspace_dialog = None
        self._populate_workspace_combo()

    # ---------- First-run setup and custom image builds ----------

    def _setup_wizard_cfg(self) -> dict:
        cfg = CONFIG.get('setup_wizard', {})
        return cfg if isinstance(cfg, dict) else {}

    def _schedule_first_run_setup_wizard(self) -> None:
        if self._setup_wizard_auto_scheduled:
            return
        if not self._should_auto_show_setup_wizard():
            return
        self._setup_wizard_auto_scheduled = True
        QTimer.singleShot(0, self._open_setup_wizard)

    def _should_auto_show_setup_wizard(self) -> bool:
        cfg = self._setup_wizard_cfg()
        if not self._bool_config_value(cfg.get('show_on_first_run', True)):
            return False
        if self._bool_config_value(cfg.get('completed', False)):
            return False
        if self._image_choices:
            return False
        platform = os.environ.get('QT_QPA_PLATFORM', '').strip().lower()
        return platform != 'offscreen'

    def _should_offer_setup_for_missing_default_image(self) -> bool:
        cfg = self._setup_wizard_cfg()
        if not self._bool_config_value(cfg.get('show_on_first_run', True)):
            return False
        if self._bool_config_value(cfg.get('completed', False)):
            return False
        platform = os.environ.get('QT_QPA_PLATFORM', '').strip().lower()
        return platform != 'offscreen'

    def _can_offer_setup_wizard(self) -> bool:
        cfg = self._setup_wizard_cfg()
        if not self._bool_config_value(cfg.get('show_on_first_run', True)):
            return False
        platform = os.environ.get('QT_QPA_PLATFORM', '').strip().lower()
        return platform != 'offscreen'

    @staticmethod
    def _bool_config_value(value, default: bool = False) -> bool:
        if value is None:
            return default
        if isinstance(value, str):
            return value.strip().lower() in {'1', 'true', 'yes', 'on'}
        return bool(value)

    def _open_custom_image_builder(self) -> None:
        self._open_setup_wizard(build_custom_default=True)

    def _open_setup_wizard(self, *, build_custom_default: bool = False) -> None:
        if self._setup_wizard_dialog:
            self._setup_wizard_dialog.raise_()
            self._setup_wizard_dialog.activateWindow()
            return
        cfg = self._setup_wizard_cfg()
        public_images = self._normalize_image_list(
            cfg.get('public_images')
        ) or [
            str(self._images_cfg.get('default', '') or '').strip()
        ]
        default_image = str(
            self._images_cfg.get('default')
            or (public_images[0] if public_images else '')
        ).strip()
        host_user = str(
            cfg.get('host_user')
            or CONFIG['process']['compose_run_env'].get(
                'MOBIPICK_HOST_USER',
                '',
            )
        ).strip()
        host_uid = str(
            cfg.get('host_uid')
            or CONFIG['process']['compose_run_env'].get('MOBIPICK_UID', '')
        ).strip()
        host_gid = str(
            cfg.get('host_gid')
            or CONFIG['process']['compose_run_env'].get('MOBIPICK_GID', '')
        ).strip()
        base_image = str(
            cfg.get('development_base_image')
            or 'ozkrelo/x_mobipick_labs:noetic-v1.2'
        ).strip()
        target_image = self._default_custom_image_ref(host_user, cfg)
        source_image = str(
            cfg.get('source_image')
            or target_image
            or base_image
            or default_image
        ).strip()
        active = self._workspace_registry.active
        workspace_names = [
            workspace.name
            for workspace in self._workspace_registry.workspaces
        ]
        wizard = ImageSetupWizard(
            public_images=public_images,
            default_image=default_image,
            host_user=host_user,
            host_uid=host_uid,
            host_gid=host_gid,
            base_image=base_image,
            target_image=target_image,
            workspace_names=workspace_names,
            active_workspace=active,
            build_custom_default=build_custom_default,
            configuration_paths=[
                (label, str(path))
                for label, path in user_configuration_paths(
                    workspace_registry_path=self._workspace_registry.path,
                    window_layout_template=self._window_layout_path_template,
                )
            ],
            source_master_folder=self._default_source_master_folder(),
            source_workspace_name=str(
                cfg.get('source_workspace_name')
                or 'clean_mobipick_labs_ws'
            ).strip(),
            source_repository=str(
                cfg.get('source_repository')
                or 'https://github.com/DFKI-NI/mobipick_labs.git'
            ).strip(),
            source_branch=str(cfg.get('source_branch') or 'noetic').strip(),
            source_image=source_image,
            install_source_default=self._bool_config_value(
                cfg.get('source_install_by_default', False)
            ),
            image_blacklist=self._image_blacklist_patterns(),
            parent=self,
        )
        wizard.accepted.connect(lambda: self._apply_setup_wizard(wizard.selection()))
        wizard.finished.connect(self._on_setup_wizard_closed)
        self._setup_wizard_dialog = wizard
        wizard.show()

    def _default_source_master_folder(self) -> str:
        if self._workspace_registry.master_folder:
            return self._workspace_registry.master_folder
        host_home = str(
            CONFIG['process']['qprocess_env'].get(
                'MOBIPICK_HOST_HOME',
                '',
            )
        ).strip()
        if host_home:
            return str(Path(host_home).expanduser() / 'ros_ws')
        return str(Path.home() / 'ros_ws')

    def _on_setup_wizard_closed(self, _result: int) -> None:
        self._setup_wizard_dialog = None

    def _default_custom_image_ref(self, host_user: str, cfg: dict) -> str:
        repo = str(
            cfg.get('development_image_repository')
            or 'ozkrelo/x_mobipick_labs'
        ).strip()
        template = str(
            cfg.get('development_image_tag_template')
            or '{user}_user_from_1.2'
        )
        safe_user = self._safe_image_tag_part(host_user or 'user')
        try:
            tag = template.format(user=safe_user)
        except (KeyError, ValueError):
            tag = f'{safe_user}_user_from_1.2'
        tag = self._safe_image_tag_part(tag)
        return f'{repo}:{tag}' if repo else tag

    @staticmethod
    def _safe_image_tag_part(value: str) -> str:
        cleaned = re.sub(r'[^A-Za-z0-9_.-]+', '-', str(value).strip())
        cleaned = cleaned.strip('.-')
        return cleaned or 'user'

    @staticmethod
    def _normalize_image_list(value) -> list[str]:
        if isinstance(value, str):
            raw_items = re.split(r'[\n,]+', value)
        elif isinstance(value, list):
            raw_items = value
        else:
            raw_items = []
        images: list[str] = []
        for item in raw_items:
            image = str(item).strip()
            if image and image not in images:
                images.append(image)
        return images

    def _image_blacklist_patterns(self) -> list[str]:
        images_cfg = self.__dict__.get('_images_cfg', {}) or {}
        return self._normalize_image_list(images_cfg.get('blacklist'))

    def _image_discovery_filters(self) -> list[str]:
        images_cfg = self.__dict__.get('_images_cfg', {}) or {}
        return self._normalize_image_list(images_cfg.get('discovery_filters'))

    @staticmethod
    def _image_ref_matches_pattern(image_ref: str, pattern: str) -> bool:
        image_text = str(image_ref or '').strip().lower()
        pattern_text = str(pattern or '').strip().lower()
        if not image_text or not pattern_text:
            return False
        if fnmatchcase(image_text, pattern_text):
            return True
        if not any(char in pattern_text for char in '*?['):
            return pattern_text in image_text
        return False

    def _image_ref_blacklisted(
        self,
        image_ref: str,
        patterns: list[str] | None = None,
    ) -> bool:
        active_patterns = (
            patterns
            if patterns is not None
            else self._image_blacklist_patterns()
        )
        return any(
            self._image_ref_matches_pattern(image_ref, pattern)
            for pattern in active_patterns
        )

    def _open_image_blacklist_dialog(self) -> None:
        records, _error_message = self._discover_filtered_image_records(
            blacklist_patterns=[],
            discovery_filters=[],
        )
        image_refs = [
            record.get('ref', '')
            for record in records
            if record.get('ref')
        ]
        if not image_refs:
            image_refs = list(getattr(self, '_image_choices', []))
        dialog = ImageBlacklistDialog(
            self._image_blacklist_patterns(),
            self._image_discovery_filters(),
            image_refs,
            self._image_ref_matches_pattern,
            self,
        )
        if dialog.exec_() != QDialog.Accepted:
            return
        patterns = dialog.patterns()
        discovery_filters = dialog.discovery_filters()
        try:
            save_user_config_update(
                {
                    'images': {
                        'blacklist': patterns,
                        'discovery_filters': discovery_filters,
                    }
                }
            )
        except OSError as exc:
            QMessageBox.warning(
                self,
                'Docker Image Filters',
                f'Failed to save image filters:\n{exc}',
            )
            return
        self._images_cfg['discovery_filters'] = discovery_filters
        self._images_cfg['blacklist'] = patterns
        CONFIG['images']['discovery_filters'] = discovery_filters
        CONFIG['images']['blacklist'] = patterns
        self._load_available_images(show_feedback=False)
        self._log_info('docker image filters updated')

    def _apply_setup_wizard(self, selection: SetupWizardSelection) -> None:
        if selection.default_image:
            self._images_cfg['default'] = selection.default_image
        profile = None
        if selection.build_custom_image:
            validation_error = self._validate_custom_image_selection(selection)
            if validation_error:
                QMessageBox.warning(self, 'Build Custom Image', validation_error)
                return
            profile = self._custom_image_profile(selection)
            self._upsert_runtime_image_profile(profile)
        if selection.install_source_workspace:
            validation_error = self._validate_source_workspace_selection(
                selection
            )
            if validation_error:
                QMessageBox.warning(
                    self,
                    'Install mobipick_labs Source',
                    validation_error,
                )
                return

        updates = {
            'setup_wizard': {
                'completed': bool(selection.remember_completion),
                'host_user': selection.host_user,
                'host_uid': selection.host_uid,
                'host_gid': selection.host_gid,
                'public_images': selection.public_images,
                'development_base_image': selection.base_image,
                'development_image_repository': self._split_image_ref(
                    selection.target_image
                )[0],
                'development_image_tag_template': (
                    self._split_image_ref(selection.target_image)[1]
                    or '{user}_user_from_1.2'
                ),
                'source_repository': selection.source_repository,
                'source_branch': selection.source_branch,
                'source_workspace_name': selection.source_workspace_name,
                'source_image': selection.source_image,
                'source_install_by_default': (
                    selection.install_source_workspace
                ),
            },
            'images': {
                'default': selection.default_image,
                'blacklist': list(selection.image_blacklist),
            },
        }
        if profile:
            updates['images']['profiles'] = list(
                self._images_cfg.get('profiles', [])
            )
        try:
            save_user_config_update(updates)
        except OSError as exc:
            QMessageBox.warning(
                self,
                'Setup Wizard',
                f'Failed to save setup settings:\n{exc}',
            )
            return

        self._images_cfg = CONFIG['images']
        self._image_profiles = self._normalize_image_profiles(
            self._images_cfg.get('profiles', [])
        )

        source_started = False

        def start_source_once() -> None:
            nonlocal source_started
            if source_started or not selection.install_source_workspace:
                return
            source_started = True
            self._start_source_workspace_install(selection)

        wait_for_custom_image = (
            selection.install_source_workspace
            and selection.build_custom_image
            and selection.source_image == selection.target_image
        )
        wait_for_public_pull = (
            selection.install_source_workspace
            and not wait_for_custom_image
            and selection.pull_public_images
            and selection.source_image in selection.public_images
        )

        if selection.pull_public_images and selection.public_images:
            if wait_for_public_pull:
                self._start_image_pulls(
                    selection.public_images,
                    on_success=start_source_once,
                )
            else:
                self._start_image_pulls(selection.public_images)
        if selection.build_custom_image:
            if wait_for_custom_image:
                self._start_custom_image_build(
                    selection,
                    on_success=start_source_once,
                )
            else:
                self._start_custom_image_build(selection)
        if selection.install_source_workspace and not (
            wait_for_custom_image or wait_for_public_pull
        ):
            start_source_once()

        if self._image_choices:
            self._load_available_images(show_feedback=False)
        self._log_info('setup wizard settings saved')

    def _validate_custom_image_selection(
        self,
        selection: SetupWizardSelection,
    ) -> str:
        if not selection.host_user:
            return 'Host user is required.'
        if not selection.host_uid.isdigit() or not selection.host_gid.isdigit():
            return 'Host UID and GID must be numeric.'
        if ':' not in selection.base_image:
            return 'Base image must include a tag, for example image:tag.'
        if ':' not in selection.target_image:
            return 'Target image must include a tag, for example image:tag.'
        return ''

    def _validate_source_workspace_selection(
        self,
        selection: SetupWizardSelection,
    ) -> str:
        if not selection.source_master_folder:
            return 'Choose a master folder for the source workspace.'
        if not selection.source_workspace_name:
            return 'Enter a source workspace name.'
        try:
            RosWorkspace(
                name=selection.source_workspace_name,
                path=str(
                    Path(selection.source_master_folder).expanduser()
                    / selection.source_workspace_name
                ),
            ).normalized()
        except ValueError as exc:
            return str(exc)
        if not selection.source_repository:
            return 'Enter the mobipick_labs repository URL.'
        if not selection.source_image:
            return 'Choose the Docker image used to build the source workspace.'
        return ''

    def _custom_image_profile(self, selection: SetupWizardSelection) -> dict:
        compatible = (
            [selection.compatible_workspace]
            if selection.compatible_workspace
            else []
        )
        return {
            'ref': selection.target_image,
            'user': 'host',
            'supports_host_workspaces': True,
            'compatible_workspaces': compatible,
            'description': (
                'Local development image with a user matching the host.'
            ),
        }

    def _upsert_runtime_image_profile(self, profile: dict) -> None:
        profiles = list(self._images_cfg.get('profiles', []) or [])
        ref = str(profile.get('ref') or '').strip()
        updated = False
        for index, item in enumerate(profiles):
            if isinstance(item, dict) and str(item.get('ref') or '').strip() == ref:
                profiles[index] = profile
                updated = True
                break
        if not updated:
            profiles.append(profile)
        self._images_cfg['profiles'] = profiles
        CONFIG['images']['profiles'] = profiles
        self._image_profiles = self._normalize_image_profiles(profiles)

    def _start_image_pulls(
        self,
        images: list[str],
        *,
        on_success: Callable[[], None] | None = None,
    ) -> None:
        key = 'setup-pull-images'
        tab = self._ensure_tab(key, 'Pull Images', closable=True)
        if tab.is_running():
            self._focus_tab(key)
            QMessageBox.information(
                self,
                'Pull Images',
                'An image pull is already running.',
            )
            return
        command = ' && '.join(
            shlex.join(['docker', 'pull', image])
            for image in images
        )
        if not command:
            return

        def _after_pull(code: int, _status) -> None:
            self._load_available_images(show_feedback=False)
            if code == 0 and on_success:
                on_success()

        tab.proc.finished.connect(_after_pull)
        tab.start_program('bash', ['-lc', command])
        self._focus_tab(key)

    def _start_custom_image_build(
        self,
        selection: SetupWizardSelection,
        *,
        on_success: Callable[[], None] | None = None,
    ) -> None:
        key = 'setup-build-image'
        tab = self._ensure_tab(key, 'Build Image', closable=True)
        if tab.is_running():
            self._focus_tab(key)
            QMessageBox.information(
                self,
                'Build Custom Image',
                'A custom image build is already running.',
            )
            return
        try:
            context_dir = self._write_custom_image_build_context(selection)
        except OSError as exc:
            QMessageBox.warning(
                self,
                'Build Custom Image',
                f'Failed to prepare Docker build context:\n{exc}',
            )
            return
        args = [
            'build',
            '--build-arg',
            f'USER={selection.host_user}',
            '--build-arg',
            f'UID={selection.host_uid}',
            '--build-arg',
            f'GID={selection.host_gid}',
            '-t',
            selection.target_image,
            str(context_dir),
        ]

        def _after_build(code: int, _status) -> None:
            self._load_available_images(show_feedback=False)
            if code == 0 and on_success:
                on_success()

        tab.proc.finished.connect(_after_build)
        self._start_program_with_pseudo_terminal(tab, 'docker', args)
        self._focus_tab(key)

    def _start_source_workspace_install(
        self,
        selection: SetupWizardSelection,
    ) -> None:
        key = 'setup-source-workspace'
        tab = self._ensure_tab(key, 'Install Source', closable=True)
        if tab.is_running():
            self._focus_tab(key)
            QMessageBox.information(
                self,
                'Install mobipick_labs Source',
                'A source workspace install is already running.',
            )
            return
        try:
            workspace = self._prepare_source_workspace(selection)
        except (OSError, ValueError) as exc:
            QMessageBox.warning(
                self,
                'Install mobipick_labs Source',
                f'Failed to prepare the source workspace:\n{exc}',
            )
            return
        self._ensure_network(log_key='log')
        workspace_env = self._workspace_runtime_env(
            workspace.name,
            force_host_workspace=True,
        )
        workspace_env.update(
            self._image_runtime_env(
                workspace_env,
                image_ref=selection.source_image,
            )
        )
        workspace_env['MOBIPICK_IMAGE'] = selection.source_image
        tab.set_environment_overrides(workspace_env)
        exec_id = uuid.uuid4().hex
        tab.exec_id = exec_id
        tab.container_name = f'mobipick-source-{exec_id[:10]}'
        command = self._source_workspace_install_command(
            workspace_env['MOBIPICK_WORKSPACE_PATH'],
            repository=selection.source_repository,
            branch=selection.source_branch,
        )
        args = [
            'compose',
            'run',
            '--rm',
            '--name',
            tab.container_name,
            '--label',
            f'mobipick.exec={exec_id}',
            '--label',
            f'mobipick.tab={key}',
            *self._compose_env_args(
                workspace_env,
                container_name=tab.container_name,
            ),
            'mobipick_cmd',
            'bash',
            '-lc',
            self._wrap_line_buffered(command),
        ]

        def _after_source_install(code: int, _status) -> None:
            self._on_source_workspace_install_finished(
                code,
                workspace,
                selection,
            )

        tab.proc.finished.connect(_after_source_install)
        self._start_program_with_pseudo_terminal(tab, 'docker', args)
        self._focus_tab(key)
        self._log_info(
            f'installing mobipick_labs source workspace at '
            f'{workspace.directory}'
        )

    def _prepare_source_workspace(
        self,
        selection: SetupWizardSelection,
    ) -> RosWorkspace:
        master = Path(selection.source_master_folder).expanduser().resolve()
        workspace_dir = master / selection.source_workspace_name
        (workspace_dir / 'src').mkdir(parents=True, exist_ok=True)
        existing = self._workspace_registry.get(selection.source_workspace_name)
        workspace = RosWorkspace(
            name=selection.source_workspace_name,
            path=str(workspace_dir),
            extends=list(existing.extends) if existing else [],
            button_config=existing.button_config if existing else '',
            launch_config=existing.launch_config if existing else '',
            sim_command=existing.sim_command if existing else '',
            image=selection.source_image,
        ).normalized()
        self._workspace_registry.master_folder = str(master)
        self._workspace_registry.upsert(workspace)
        self._workspace_registry.save()
        self._populate_workspace_combo()
        if self._workspace_dialog:
            self._workspace_dialog.refresh(self._workspace_registry.active)
        return workspace

    def _source_workspace_install_command(
        self,
        container_workspace_path: str,
        *,
        repository: str,
        branch: str,
    ) -> str:
        workspace = Path(container_workspace_path)
        src_dir = workspace / 'src'
        repo_dir = src_dir / 'mobipick_labs'
        setup_file = workspace / 'devel' / 'setup.bash'
        quoted_repo = self._sh_quote(repository)
        quoted_branch = self._sh_quote(branch) if branch else ''
        quoted_origin_branch = (
            self._sh_quote(f'origin/{branch}') if branch else ''
        )
        clone_branch = f' --branch {quoted_branch}' if branch else ''
        branch_checkout = (
            'git -C "$repo_dir" fetch --all --prune; '
            f'git -C "$repo_dir" checkout {quoted_branch} '
            f'|| git -C "$repo_dir" checkout -b {quoted_branch} '
            f'{quoted_origin_branch}; '
            f'git -C "$repo_dir" pull --ff-only origin {quoted_branch}'
            if branch
            else 'git -C "$repo_dir" pull --ff-only'
        )
        return (
            'set -e; '
            'export DEBIAN_FRONTEND=noninteractive; '
            'source /opt/ros/noetic/setup.bash; '
            f'workspace={self._sh_quote(str(workspace))}; '
            f'src_dir={self._sh_quote(str(src_dir))}; '
            f'repo_dir={self._sh_quote(str(repo_dir))}; '
            f'setup_file={self._sh_quote(str(setup_file))}; '
            'mkdir -p "$src_dir"; '
            'if [ -d "$repo_dir/.git" ]; then '
            'echo "Updating existing mobipick_labs checkout"; '
            f'{branch_checkout}; '
            'elif [ -e "$repo_dir" ]; then '
            'echo "Path exists but is not a git checkout: $repo_dir" >&2; '
            'exit 1; '
            'else '
            'echo "Cloning mobipick_labs"; '
            f'git clone{clone_branch} {quoted_repo} "$repo_dir"; '
            'fi; '
            'cd "$repo_dir"; '
            './install-deps.sh; '
            './build.sh; '
            'if [ -s "$setup_file" ]; then '
            'source "$setup_file"; '
            'echo "Built and sourced $setup_file"; '
            'else '
            'echo "Build did not create $setup_file" >&2; '
            'exit 1; '
            'fi'
        )

    def _on_source_workspace_install_finished(
        self,
        code: int,
        workspace: RosWorkspace,
        selection: SetupWizardSelection,
    ) -> None:
        if code == 0:
            try:
                self._mark_image_workspace_match(
                    selection.source_image,
                    workspace.name,
                )
            except Exception as exc:
                self._append_gui_html(
                    'log',
                    '<i>Failed to save source workspace image match: '
                    f'{html.escape(str(exc))}</i>',
                )
            self._populate_workspace_combo()
            if self._workspace_dialog:
                self._workspace_dialog.refresh(self._workspace_registry.active)
            self._log_info(
                f'mobipick_labs source workspace is ready: '
                f'{workspace.directory}'
            )
            if selection.activate_source_workspace:
                self._activate_workspace(workspace.name)
        else:
            self._append_gui_html(
                'log',
                '<i>mobipick_labs source workspace install failed; '
                'see the Install Source tab for details.</i>',
            )

    def _write_custom_image_build_context(
        self,
        selection: SetupWizardSelection,
    ) -> Path:
        slug = self._safe_image_tag_part(
            selection.target_image.replace('/', '_').replace(':', '_')
        )
        context_dir = USER_DATA_DIR / 'image_builds' / slug
        context_dir.mkdir(parents=True, exist_ok=True)
        dockerfile = context_dir / 'Dockerfile'
        dockerfile.write_text(
            self._custom_image_dockerfile(selection.base_image),
            encoding='utf-8',
        )
        entrypoint_source = PROJECT_ROOT / 'custom_entrypoint.sh'
        entrypoint_target = context_dir / 'entrypoint_user.sh'
        entrypoint_target.write_text(
            entrypoint_source.read_text(encoding='utf-8'),
            encoding='utf-8',
        )
        return context_dir

    @staticmethod
    def _custom_image_dockerfile(base_image: str) -> str:
        return f"""FROM {base_image}

ARG USER
ARG UID
ARG GID

ENV USER=${{USER}} \\
    HOME=/home/${{USER}}

USER root

RUN set -eux; \\
    : "${{USER:?USER build-arg required}}"; \\
    : "${{UID:?UID build-arg required}}"; \\
    : "${{GID:?GID build-arg required}}"; \\
    if ! getent group "${{GID}}" >/dev/null 2>&1; then \\
        groupadd -g "${{GID}}" "${{USER}}"; \\
    fi; \\
    if ! id -u "${{USER}}" >/dev/null 2>&1; then \\
        useradd -m -u "${{UID}}" -g "${{GID}}" -s /bin/bash "${{USER}}"; \\
    fi

RUN apt-get update && \\
    DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends sudo && \\
    rm -rf /var/lib/apt/lists/* && \\
    echo "${{USER}} ALL=(ALL) NOPASSWD:ALL" > "/etc/sudoers.d/${{USER}}" && \\
    chmod 0440 "/etc/sudoers.d/${{USER}}"

COPY entrypoint_user.sh /usr/local/bin/entrypoint_user.sh
RUN chmod 0755 /usr/local/bin/entrypoint_user.sh

USER ${{USER}}
WORKDIR ${{HOME}}

ENTRYPOINT ["/usr/local/bin/entrypoint_user.sh"]
CMD ["bash"]
"""

    def _build_active_workspace(self) -> None:
        workspace = self._workspace_registry.active_workspace()
        if workspace:
            self._build_workspace(workspace.name)

    def _build_workspace(self, name: str) -> None:
        workspace = self._workspace_registry.get(name)
        if not workspace:
            return
        if not self._image_supports_host_workspaces(self._selected_image):
            QMessageBox.warning(
                self,
                'Build Workspace',
                'The selected Docker image uses its baked workspace and '
                'does not mount host ROS workspaces. Select a development '
                'image with a matching host user before building.',
            )
            return
        if self._workspace_processes_running():
            QMessageBox.warning(
                self,
                'Build Workspace',
                'Stop running workspace processes before building.',
            )
            return
        self._ensure_network(log_key='log')
        key = f'build-{workspace.name}'
        tab = self._ensure_tab(
            key,
            f'Build {workspace.name}',
            closable=True,
        )
        if tab.is_running():
            self._focus_tab(key)
            return
        if not self._mark_workspace_build_image_match(workspace):
            return
        workspace_env = self._workspace_runtime_env(
            workspace.name,
            force_host_workspace=True,
        )
        tab.set_environment_overrides(workspace_env)
        exec_id = uuid.uuid4().hex
        tab.exec_id = exec_id
        tab.container_name = f'mobipick-build-{exec_id[:10]}'
        command = self._workspace_registry.build_command(
            workspace,
            image_workspace_path=self._image_workdir(self._selected_image),
        )
        args = [
            'compose',
            'run',
            '--rm',
            '--name',
            tab.container_name,
            '--label',
            f'mobipick.exec={exec_id}',
            '--label',
            f'mobipick.tab={key}',
            *self._compose_env_args(
                workspace_env,
                container_name=tab.container_name,
            ),
            'mobipick_cmd',
            'bash',
            '-lc',
            self._wrap_line_buffered(command),
        ]
        self._start_program_with_pseudo_terminal(tab, 'docker', args)
        self._focus_tab(key)

    def _workspace_sim_command(self) -> str:
        workspace = self._workspace_registry.active_workspace()
        sim_cfg = getattr(self, '_config_buttons', {}).get('sim', {})
        command = str(sim_cfg.get('command') or '').strip()
        if command and not (
            sim_cfg.get('command_is_default')
            and workspace
            and workspace.sim_command
        ):
            return command
        if workspace and workspace.sim_command:
            return workspace.sim_command
        return DEFAULT_BUTTON_COMMANDS['sim']

    def _rviz_command(self) -> str:
        command = self._profile_button_command('rviz')
        if command:
            return command
        return DEFAULT_BUTTON_COMMANDS['rviz']

    def _tables_demo_command(self) -> str:
        command = self._profile_button_command('tables')
        return command or DEFAULT_BUTTON_COMMANDS['tables']

    def _rqt_tables_command(self) -> str:
        command = self._profile_button_command('rqt')
        return command or DEFAULT_BUTTON_COMMANDS['rqt']

    def _profile_button_command(self, key: str) -> str:
        cfg = getattr(self, '_config_buttons', {}).get(key, {})
        return str(cfg.get('command') or '').strip()

    def _get_button_widget(self, key: str) -> QPushButton | None:
        return self._button_widgets.get(key)

    def _auto_launch_button_cfg(self) -> dict:
        return self._launch_plan.get('button', {}) if isinstance(self._launch_plan, dict) else {}

    def _auto_launch_label(self) -> str:
        cfg = self._auto_launch_button_cfg()
        return str(cfg.get('label') or cfg.get('start_text') or 'Auto Launch')

    def _auto_launch_start_text(self) -> str:
        cfg = self._auto_launch_button_cfg()
        return str(cfg.get('start_text') or cfg.get('label') or 'Start Auto Launch')

    def _auto_launch_stop_text(self) -> str:
        cfg = self._auto_launch_button_cfg()
        return str(cfg.get('stop_text') or cfg.get('label') or 'Stop Auto Launch')

    def _build_configurable_buttons(
        self,
        layout: QHBoxLayout,
        *,
        insert_at: int | None = None,
    ):
        self._config_buttons: dict[str, dict] = {}
        self._config_button_order = []
        for entry in self._button_layout:
            if not isinstance(entry, dict):
                continue
            key = str(entry.get('key', '')).strip()
            if not key or key in {'roscore', 'terminal'}:
                continue
            normalized = {
                'key': key,
                'label': entry.get('label') or entry.get('text') or key,
                'kind': str(entry.get('kind') or entry.get('type') or 'builtin').lower(),
                'action': entry.get('action') or key,
                'command': entry.get('command'),
                'tooltip': entry.get('tooltip'),
                'requires_roscore': bool(entry.get('requires_roscore', True)),
                'reuse_tab': bool(entry.get('reuse_tab', False)),
                'world_arg_name': entry.get('world_arg_name', 'world_config'),
                'world_config_required': entry.get('world_config_required', False),
                'setup': entry.get('setup') or entry.get('pre_command') or entry.get('prologue'),
                'host': bool(entry.get('host', False)),
                'stop_command': entry.get('stop_command'),
                'log_command': entry.get('log_command'),
                'pass_ros_master_uri': entry.get('pass_ros_master_uri', False),
                'service': entry.get('service') or '',
            }
            self._config_buttons[key] = normalized
            self._config_button_order.append(key)
            button = QPushButton(normalized['label'])
            _configure_expanding_toolbar_button(button)
            tooltip = normalized.get('tooltip')
            if tooltip:
                button.setToolTip(str(tooltip))
            button.clicked.connect(
                lambda _checked=False, k=key: self._on_config_button_clicked(k)
            )
            if insert_at is None or insert_at < 0:
                layout.addWidget(button)
            else:
                layout.insertWidget(insert_at, button)
                insert_at += 1
            self._button_widgets[key] = button

    def _on_config_button_clicked(self, key: str):
        button = self._get_button_widget(key)
        config = self._config_buttons.get(key, {})
        self._log_button_click(button, config.get('label') or key)
        if not config:
            return
        kind = str(config.get('kind') or 'builtin').lower()
        if kind == 'command':
            self._run_config_command(config)
        else:
            self._dispatch_builtin_action(config)

    def _config_label(self, config: dict) -> str:
        return str(config.get('label') or config.get('key') or 'Command')

    def _set_config_visual(self, config: dict, state: str, text: str, enabled: bool):
        key = str(config.get('key'))
        if key == 'sim':
            self.set_toggle_visual(state, text, enabled)
            return
        self._set_toggle_state(key, self._get_button_widget(key), state, text, enabled)

    def _reset_config_button_visuals(self):
        for key in self._config_button_order:
            config = self._config_buttons.get(key, {})
            label = self._config_label(config)
            self._set_config_visual(config, 'red', f'Start {label}', True)

    @staticmethod
    def _neutralize_compose_ignore(cmd: str) -> str:
        cmd = cmd.strip()
        if not cmd:
            return cmd
        return f'COMPOSE_IGNORE_ORPHANS= {cmd}'

    def _dispatch_builtin_action(self, config: dict):
        action = str(config.get('action') or config.get('key') or '').strip().lower()
        button = self._get_button_widget(config.get('key', ''))
        if action in {'sim', 'toggle_sim', 'sim_toggle'}:
            if not self._guard_toggle_action('sim', button):
                return
            self.toggle_sim()
        elif action in {'tables', 'tables_demo', 'toggle_tables'}:
            if not self._guard_toggle_action('tables', button):
                return
            self.toggle_tables_demo()
        elif action in {'rviz', 'toggle_rviz'}:
            if not self._guard_toggle_action('rviz', button):
                return
            self.toggle_rviz()
        elif action in {'rqt', 'rqt_tables', 'toggle_rqt'}:
            if not self._guard_toggle_action('rqt', button):
                return
            self.toggle_rqt_tables_demo()
        elif action in {'refresh', 'refresh_status', 'refresh_sim'}:
            self._on_refresh_clicked()
        else:
            QMessageBox.information(self, 'Buttons', f'No builtin action registered for "{action}".')

    def _run_config_command(self, config: dict):
        command = str(config.get('command') or '').strip()
        if not command:
            QMessageBox.information(self, 'Buttons', 'Command is missing for this button.')
            return
        key = str(config.get('key') or '').strip() or 'command'
        button = self._get_button_widget(key)
        if not self._guard_toggle_action(key, button):
            return
        setup = str(config.get('setup') or '').strip()
        run_on_host = bool(config.get('host'))
        stop_command = str(config.get('stop_command') or '').strip()
        log_command = str(config.get('log_command') or '').strip()
        pass_master = bool(config.get('pass_ros_master_uri'))
        label = self._config_label(config)
        tab = self._ensure_tab(key, label, closable=False)
        if tab.is_running():
            self._set_config_visual(config, 'yellow', f'Stopping {label}...', False)
            def _done():
                self._set_config_visual(config, 'red', f'Start {label}', True)
            stop_cmd_for_running = stop_command if run_on_host else None
            if run_on_host and stop_cmd_for_running:
                stop_cmd_for_running = self._neutralize_compose_ignore(stop_cmd_for_running)
            self._stop_custom_tab(tab, on_stopped=_done, stop_command=stop_cmd_for_running)
            return
        if not run_on_host and not self._confirm_workspace_mismatch_warning(label):
            return

        def _apply_env(cmd: str) -> str:
            if not pass_master:
                return cmd
            master = self._current_master_uri()
            if not master:
                return cmd
            return f"ROS_MASTER_URI={self._sh_quote(master)} {cmd}"

        def _run_command():
            key_label = config.get('key', 'button')
            full_command = command
            if config.get('world_config_required'):
                arg_name = str(config.get('world_arg_name') or 'world_config')
                world = self._current_world()
                if world:
                    full_command = f"{command} {arg_name}:={self._sh_quote(world)}"
                else:
                    self._log_info('world_config_required set but no world selected; running without flag')
            if setup:
                composed = f'{setup} && {full_command}'
                full_command = f"bash -lc {self._sh_quote(composed)}"
            full_command = _apply_env(full_command)
            stop_command_full = _apply_env(stop_command) if stop_command else ''
            log_command_full = _apply_env(log_command) if log_command else ''
            if run_on_host:
                full_command = self._neutralize_compose_ignore(full_command)
                if stop_command_full:
                    stop_command_full = self._neutralize_compose_ignore(stop_command_full)
                if log_command_full:
                    log_command_full = self._neutralize_compose_ignore(log_command_full)
            self._log_info(f'running configured command ({key_label}): {full_command}')
            if run_on_host:
                tab.container_name = None
                tab.exec_id = None
                if log_command_full:
                    self._sp_run(['bash', '-lc', full_command], log_key=tab.key, check=False)
                    tab.start_program('bash', ['-lc', log_command_full])
                else:
                    tab.start_program('bash', ['-lc', full_command])
            else:
                exec_id = uuid.uuid4().hex
                tab.exec_id = exec_id
                tab.container_name = f'mpcmd-{exec_id[:10]}'
                self._claim_xhost(tab, key, log_key=tab.key)
                service = self._configured_command_service(config)
                wrapped = self._wrap_line_buffered(full_command)
                args = [
                    'compose', 'run', '--rm', '--name', tab.container_name,
                    '--label', f'mobipick.exec={exec_id}', '--label', f'mobipick.tab={key}',
                    *self._compose_env_args(container_name=tab.container_name),
                    service, 'bash', '-lc', wrapped
                ]
                tab.start_program('docker', args)
                self._schedule_host_to_container_copy(tab)
            self._focus_tab(key)
            self._update_stop_custom_enabled()
            self._set_config_visual(config, 'green', f'Stop {label}', True)

        self._set_config_visual(config, 'yellow', f'Starting {label}...', False)
        if config.get('requires_roscore', True):
            self._ensure_roscore_ready(_run_command)
        else:
            _run_command()

    # ---------- Log tab helpers ----------

    def _fmt_args(self, args_or_str) -> str:
        if isinstance(args_or_str, str):
            return args_or_str
        return ' '.join(shlex.quote(s) for s in args_or_str)

    def _compose_env_args(
        self,
        overrides: Optional[dict[str, str]] = None,
        *,
        container_name: str | None = None,
    ) -> list[str]:
        env_args: list[str] = []
        compose_env = dict(CONFIG['process']['compose_run_env'])
        workspace_env = self._workspace_runtime_env()
        effective_workspace_env = dict(workspace_env)
        if overrides:
            for key, value in overrides.items():
                effective_workspace_env[str(key)] = str(value)
        compose_env.update(workspace_env)
        compose_env.update(self._image_runtime_env(effective_workspace_env))
        if self._selected_image:
            compose_env['MOBIPICK_IMAGE'] = self._selected_image
        world = self._current_world()
        if world:
            compose_env['MOBIPICK_WORLD'] = world
        master_uri = self._current_master_uri()
        if master_uri:
            compose_env['ROS_MASTER_URI'] = master_uri
        if overrides:
            for key, value in overrides.items():
                compose_env[str(key)] = str(value)
        for key, value in compose_env.items():
            if key in {
                'MOBIPICK_WORKSPACE_COMPAT_ROOTS',
                'MOBIPICK_WORKSPACE_MOUNT_SOURCE',
            }:
                continue
            env_args.extend(['--env', f'{key}={value}'])
        return env_args

    @staticmethod
    def _normalize_ros_master_uri(value) -> str:
        raw = str(value or '').strip()
        if not raw:
            return ''
        if '://' not in raw:
            raw = f'http://{raw}'
        try:
            parsed = urlsplit(raw)
            port = parsed.port or 11311
        except ValueError:
            return ''
        if (
            parsed.scheme.lower() != 'http'
            or not parsed.hostname
            or parsed.username
            or parsed.password
            or parsed.query
            or parsed.fragment
            or parsed.path not in {'', '/'}
        ):
            return ''
        hostname = parsed.hostname
        if ':' in hostname:
            hostname = f'[{hostname}]'
        return f'http://{hostname}:{port}'

    def _remote_master_enabled(self) -> bool:
        return bool(
            getattr(self, '_remote_master_enabled_value', False)
        )

    def _ros_tool_service(self) -> str:
        if self._remote_master_enabled():
            return self._remote_ros_service
        return 'mobipick_cmd'

    def _configured_command_service(self, config: dict) -> str:
        service = str(config.get('service') or '').strip()
        allowed = {'mobipick', 'mobipick_cmd', self._remote_ros_service}
        if service in allowed:
            return service
        if service:
            label = self._config_label(config)
            self._log_info(
                f'unknown compose service "{service}" for {label}; '
                f'using {self._ros_tool_service()}'
            )
        return self._ros_tool_service()

    def _current_master_uri(self) -> str:
        if self._remote_master_enabled():
            return self._remote_master_uri_value
        if getattr(self, '_roscore_stopping', False):
            return 'http://mobipick:11311'
        if getattr(self, '_roscore_running_cached', False):
            return f'http://{self._roscore_container_name}:11311'
        if 'roscore' in self.tasks and self.tasks['roscore'].is_running():
            self._roscore_running_cached = True
            return f'http://{self._roscore_container_name}:11311'
        return 'http://mobipick:11311'

    def _set_remote_master_checkbox(self, checked: bool) -> None:
        self.remote_master_checkbox.blockSignals(True)
        self.remote_master_checkbox.setChecked(checked)
        self.remote_master_checkbox.blockSignals(False)

    def _on_remote_master_toggled(self, checked: bool) -> None:
        checked = bool(checked)
        if checked == self._remote_master_enabled_value:
            self.remote_master_input.setEnabled(checked)
            return
        if self._workspace_processes_running():
            QMessageBox.warning(
                self,
                'ROS Master',
                'Stop running containers before changing the ROS master.',
            )
            self._set_remote_master_checkbox(
                self._remote_master_enabled_value
            )
            return
        if checked:
            normalized = self._normalize_ros_master_uri(
                self.remote_master_input.text()
            )
            if not normalized:
                QMessageBox.warning(
                    self,
                    'ROS Master',
                    'Enter a valid ROS master URI such as '
                    'http://mobipick-os-sensor:11311.',
                )
                self._set_remote_master_checkbox(False)
                return
            self._remote_master_uri_value = normalized
            self.remote_master_input.setText(normalized)
        self._remote_master_enabled_value = checked
        self.remote_master_input.setEnabled(checked)
        self._apply_env_to_all_tabs()
        self._update_buttons()
        if checked:
            self._log_info(
                'using remote ROS master '
                f'{self._remote_master_uri_value}'
            )
        else:
            self._log_info('using local ROS master')

    def _on_remote_master_uri_edited(self) -> None:
        normalized = self._normalize_ros_master_uri(
            self.remote_master_input.text()
        )
        if not normalized:
            QMessageBox.warning(
                self,
                'ROS Master',
                'Enter a valid ROS master URI such as '
                'http://mobipick-os-sensor:11311.',
            )
            self.remote_master_input.setText(
                self._remote_master_uri_value
            )
            return
        if (
            normalized != self._remote_master_uri_value
            and self._workspace_processes_running()
        ):
            QMessageBox.warning(
                self,
                'ROS Master',
                'Stop running containers before changing the ROS master.',
            )
            self.remote_master_input.setText(
                self._remote_master_uri_value
            )
            return
        self._remote_master_uri_value = normalized
        self.remote_master_input.setText(normalized)
        self._apply_env_to_all_tabs()
        if self._remote_master_enabled():
            self._log_info(f'remote ROS master set to {normalized}')

    def _build_process_environment(self, extra: Optional[dict[str, str]] = None) -> QProcessEnvironment:
        env = QProcessEnvironment.systemEnvironment()
        workspace_env = self._workspace_runtime_env()
        effective_workspace_env = dict(workspace_env)
        if extra:
            for key, value in extra.items():
                effective_workspace_env[str(key)] = str(value)
        for key, value in CONFIG['process']['qprocess_env'].items():
            env.insert(str(key), str(value))
        for key, value in workspace_env.items():
            env.insert(str(key), str(value))
        for key, value in self._image_runtime_env(
            effective_workspace_env
        ).items():
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

    def _terminal_env_overrides(self) -> dict[str, str]:
        if not self._terminal_run_as_root_requested():
            return {}
        return {
            'MOBIPICK_UID': '0',
            'MOBIPICK_GID': '0',
            'MOBIPICK_HOST_USER': 'root',
            'MOBIPICK_HOST_GROUP': 'root',
            'MOBIPICK_HOST_HOME': '/root',
            'MOBIPICK_CONTAINER_USER': 'root',
        }

    def _terminal_run_as_root_requested(self) -> bool:
        checkbox = getattr(self, 'terminal_root_checkbox', None)
        return bool(checkbox and checkbox.isChecked())

    def _prepare_run_env(self, run_kwargs: dict) -> dict:
        env = run_kwargs.get('env')
        if env is None:
            env = os.environ.copy()
        else:
            env = {str(k): str(v) for k, v in env.items()}
        for key, value in CONFIG['process']['qprocess_env'].items():
            env[str(key)] = str(value)
        workspace_env = self._workspace_runtime_env()
        for key, value in workspace_env.items():
            env[str(key)] = str(value)
        for key, value in self._image_runtime_env(workspace_env).items():
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

    @staticmethod
    def _normalize_stop_timeout(value) -> int | None:
        try:
            timeout = int(value)
        except (TypeError, ValueError):
            return None
        if timeout < 0:
            return None
        return timeout

    def _docker_stop_args(self, container_id: str) -> list[str]:
        if self._docker_stop_timeout is None:
            return ['stop', container_id]
        return ['stop', '--time', str(self._docker_stop_timeout), container_id]

    def _docker_stop_display(self, label: str) -> str:
        if self._docker_stop_timeout is None:
            return f'docker stop {label}'
        return f'docker stop --time {self._docker_stop_timeout} {label}'

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

    def _docker_cp_entries(self, direction: str) -> list[dict[str, str]]:
        config = getattr(self, '_docker_cp_config', {}) or {}
        if direction not in {'host_to_container', 'container_to_host'}:
            return []
        if not config:
            return []
        workspace = self._workspace_registry.active_workspace()
        key = workspace.name if workspace else 'default'
        section = config.get(key)
        if section is None and key != 'default':
            section = config.get('default')
        if not isinstance(section, dict):
            return []
        values = section.get(direction)
        if not isinstance(values, list):
            return []
        return [value for value in values if isinstance(value, dict)]

    def _docker_cp_workspace_names(self) -> list[str]:
        return [workspace.name for workspace in self._workspace_registry.workspaces]

    def _docker_cp_host_start_path(self, workspace_name: str) -> Path:
        workspace_key = str(workspace_name or '').strip()
        raw_master = str(self._workspace_registry.master_folder or '').strip()
        master = Path(raw_master).expanduser() if raw_master else None
        if workspace_key and master is not None:
            candidate = master / workspace_key
            if candidate.is_dir():
                return candidate
        workspace = self._workspace_registry.get(workspace_key)
        if workspace and workspace.directory.is_dir():
            return workspace.directory
        if master is not None and master.is_dir():
            return master
        return Path.home()

    def _docker_ps_container_records(self) -> list[dict[str, str]]:
        try:
            cp = subprocess.run(
                [
                    'docker',
                    'ps',
                    '--format',
                    '{{.ID}}\t{{.Image}}\t{{.Names}}',
                ],
                stdout=subprocess.PIPE,
                stderr=subprocess.DEVNULL,
                check=False,
                text=True,
                timeout=8,
            )
        except Exception as exc:
            self._console_log(1, f'Failed to query running containers: {exc}')
            return []
        records: list[dict[str, str]] = []
        for line in (cp.stdout or '').splitlines():
            parts = line.split('\t', 2)
            if len(parts) != 3:
                continue
            container_id, image, name = (part.strip() for part in parts)
            if self._image_ref_blacklisted(image):
                continue
            if container_id:
                records.append(
                    {
                        'id': container_id,
                        'image': image,
                        'name': name,
                    }
                )
        return records

    def _docker_cp_workspace_match_images(self, workspace_name: str) -> list[str]:
        match_image = ''
        try:
            match_image = self._workspace_match_image(workspace_name)
        except Exception:
            match_image = ''
        registry_image = ''
        image_for = getattr(self._workspace_registry, 'image_for', None)
        if callable(image_for):
            registry_image = image_for(workspace_name, '')
        selected_image = ''
        try:
            selected_image = getattr(self, '_selected_image', '')
        except Exception:
            selected_image = ''
        candidates = [
            match_image,
            registry_image,
            selected_image
            if self._image_compatible_with_workspace(
                selected_image,
                workspace_name,
            ) is True
            else '',
        ]
        return list(
            dict.fromkeys(
                image.strip()
                for image in candidates
                if (
                    isinstance(image, str)
                    and image.strip()
                    and not self._image_ref_blacklisted(image)
                )
            )
        )

    def _docker_cp_setup_container_options(
        self,
        workspace_name: str | None = None,
    ) -> list[tuple[str, str]]:
        options: list[tuple[str, str]] = []
        seen: set[str] = set()

        def add_record(label: str, container_ref: str) -> None:
            if not container_ref or container_ref in seen:
                return
            options.append((label, container_ref))
            seen.add(container_ref)

        def add_tab(label: str, tab: ProcessTab | None) -> None:
            if not isinstance(tab, ProcessTab):
                return
            if not getattr(tab, 'container_name', None):
                return
            container_ref = self._container_reference_for_tab(tab)
            if not container_ref:
                return
            add_record(label, container_ref)

        workspace_key = (
            self._workspace_registry.active
            if workspace_name is None
            else str(workspace_name or '').strip()
        )
        if workspace_key == 'default':
            workspace_key = ''
        for image_ref in self._docker_cp_workspace_match_images(workspace_key):
            add_record(
                f'Workspace match image ({image_ref})',
                f'{DockerCpConfigDialog.IMAGE_SETUP_PREFIX}{image_ref}',
            )

        records = self._docker_ps_container_records()
        for record in records:
            image = record.get('image', '')
            if self._image_compatible_with_workspace(image, workspace_key) is True:
                label = (
                    'Workspace match container '
                    f'({record.get("name") or record["id"]}, {image})'
                )
                add_record(label, record['id'])

        current = self.tasks.get(self._current_tab_key() or '')
        if current:
            add_tab('Workspace match container (current tab)', current)
        for key in ('sim', 'roscore', 'rviz', 'tables', 'rqt'):
            add_tab(f'Workspace match container ({key})', self.tasks.get(key))
        for key, tab in self.tasks.items():
            add_tab(f'{key}: {getattr(tab, "container_name", "")}', tab)
        return options

    def _docker_cp_container_path_from_setup(
        self,
        container_ref: str,
        default_path: str,
    ) -> str:
        default_path = str(default_path or '').strip()
        if not container_ref:
            return default_path
        dialog = DockerCpContainerPathDialog(
            container_ref=container_ref,
            start_path=default_path or '/',
            list_provider=self._docker_cp_list_container_paths,
        )
        if dialog.exec_() == QDialog.Accepted and dialog.selected_path():
            return dialog.selected_path()
        return default_path

    def _docker_cp_container_command(
        self,
        container_ref: str,
        script: str,
    ) -> list[str]:
        image_ref = ''
        if container_ref.startswith(DockerCpConfigDialog.IMAGE_SETUP_PREFIX):
            image_ref = container_ref[
                len(DockerCpConfigDialog.IMAGE_SETUP_PREFIX):
            ]
        if image_ref:
            command = [
                'docker',
                'run',
                '--rm',
                '--user',
                'root',
                '--entrypoint',
                'bash',
            ]
            workspace_env = self._docker_cp_workspace_browser_env()
            source = workspace_env.get('MOBIPICK_WORKSPACE_MOUNT_SOURCE', '')
            target = workspace_env.get('MOBIPICK_WORKSPACE_MOUNT_TARGET', '')
            if source and target:
                command.extend(['--volume', f'{source}:{target}:rw'])
            for key, value in workspace_env.items():
                if key in {
                    'MOBIPICK_WORKSPACE_COMPAT_ROOTS',
                    'MOBIPICK_WORKSPACE_MOUNT_SOURCE',
                }:
                    continue
                command.extend(['--env', f'{key}={value}'])
            command.extend([image_ref, '-lc', script])
            return command
        return ['docker', 'exec', container_ref, 'bash', '-lc', script]

    def _docker_cp_workspace_browser_env(self) -> dict[str, str]:
        try:
            return self._workspace_runtime_env(force_host_workspace=True)
        except Exception as exc:
            self._console_log(
                1,
                f'Failed to prepare workspace mount for docker cp browser: {exc}',
            )
            return {}

    def _docker_cp_list_container_paths(
        self,
        container_ref: str,
        path: str,
    ) -> list[dict[str, object]]:
        target = str(path or '/').strip() or '/'
        script = (
            'set -e; '
            f'target={shlex.quote(target)}; '
            'if [ -d "$target" ]; then dir="$target"; '
            'else dir=$(dirname -- "$target"); fi; '
            'printf "__DIR__\\t%s\\n" "$dir"; '
            'if [ -d "$dir" ]; then '
            'for item in "$dir"/* "$dir"/.[!.]* "$dir"/..?*; do '
            '[ -e "$item" ] || continue; '
            'if [ -d "$item" ]; then kind=d; else kind=f; fi; '
            'name=${item##*/}; '
            'printf "%s\\t%s\\t%s\\n" "$kind" "$name" "$item"; '
            'done | sort; '
            'fi'
        )
        entries: list[dict[str, object]] = []
        try:
            cp = subprocess.run(
                self._docker_cp_container_command(container_ref, script),
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                check=False,
                text=True,
                timeout=8,
            )
        except Exception as exc:
            self._console_log(
                1,
                f'Failed to inspect container paths in {container_ref}: {exc}',
            )
            return [{'error': f'Failed to inspect container paths: {exc}'}]
        returncode = getattr(cp, 'returncode', 0)
        if returncode != 0:
            details = (cp.stderr or cp.stdout or '').strip()
            if not details:
                details = f'docker command exited with status {returncode}'
            message = f'Unable to read container path {target}: {details}'
            self._console_log(1, message)
            return [{'error': message}]
        for line in (cp.stdout or '').splitlines():
            if not line.strip():
                continue
            if line.startswith('__DIR__\t'):
                directory = line.split('\t', 1)[1].strip()
                if directory and directory != '/':
                    entries.append(
                        {
                            'name': '..',
                            'path': str(Path(directory).parent),
                            'is_dir': True,
                        }
                    )
                continue
            parts = line.split('\t', 2)
            if len(parts) != 3:
                continue
            kind, name, item_path = parts
            entries.append(
                {
                    'name': name,
                    'path': item_path,
                    'is_dir': kind == 'd',
                }
            )
        return entries

    @staticmethod
    def _expand_host_path(path: str) -> str:
        return os.path.expanduser(os.path.expandvars(path or ''))

    def _container_reference_for_tab(self, tab: ProcessTab) -> str | None:
        container_name = getattr(tab, 'container_name', None)
        exec_id = getattr(tab, 'exec_id', None)
        ids = self._resolve_container_ids(name=container_name, exec_id=exec_id)
        if ids:
            return ids[0]
        return container_name

    def _schedule_host_to_container_copy(self, tab: ProcessTab, attempt: int = 0):
        if attempt > 6:
            return
        if not isinstance(tab, ProcessTab):
            return
        if not getattr(tab, 'container_name', None):
            return
        entries = self._docker_cp_entries('host_to_container')
        if not entries:
            return

        delay_ms = 500 if attempt == 0 else 1000

        def _attempt():
            if not getattr(tab, 'container_name', None):
                return
            container_ref = self._container_reference_for_tab(tab)
            if not container_ref:
                self._schedule_host_to_container_copy(tab, attempt + 1)
                return
            ref_key = f'{container_ref}:{tab.key}'
            if ref_key in self._synced_container_refs:
                return
            commands = self._build_host_to_container_commands(container_ref, entries, tab)
            self._synced_container_refs.add(ref_key)
            if not commands:
                return
            self._append_gui_html(tab.key, '<i>Copying configured host files into container...</i>')
            self._log_host_to_container_commands(tab.key, commands)
            self._run_command_sequence(commands, log_key=tab.key)

        QTimer.singleShot(delay_ms, _attempt)

    def _log_host_to_container_commands(
        self,
        tab_key: str,
        commands: list[list[str]],
    ) -> None:
        for command in commands:
            if len(command) < 4:
                continue
            host_path = command[2]
            container_target = command[3]
            self._append_gui_html(
                tab_key,
                (
                    '<i>Copying configured path from host at '
                    f'{html.escape(host_path)} -&gt; container at '
                    f'{html.escape(container_target)}</i>'
                ),
            )

    def _build_host_to_container_commands(
        self,
        container_ref: str,
        entries: list[dict[str, str]],
        tab: ProcessTab,
    ) -> list[list[str]]:
        commands: list[list[str]] = []
        for entry in entries:
            host_path = self._expand_host_path(entry.get('host', ''))
            container_path = entry.get('container', '')
            if not host_path or not container_path:
                continue
            if not os.path.exists(host_path):
                self._append_gui_html(
                    tab.key,
                    f'<i>docker cp skipped (missing host path): {html.escape(host_path)}</i>',
                )
                continue
            commands.append(['docker', 'cp', host_path, f'{container_ref}:{container_path}'])
        return commands

    def _build_container_to_host_commands(
        self,
        container_ref: str,
        entries: list[dict[str, str]],
        tab_key: str,
    ) -> list[list[str]]:
        commands: list[list[str]] = []
        for entry in entries:
            host_path_raw = entry.get('host', '')
            container_path = entry.get('container', '')
            host_path = self._expand_host_path(host_path_raw)
            if not host_path or not container_path:
                continue
            target = Path(host_path)
            try:
                if host_path.endswith(os.sep):
                    target.mkdir(parents=True, exist_ok=True)
                else:
                    target.parent.mkdir(parents=True, exist_ok=True)
            except Exception as exc:
                self._append_gui_html(
                    tab_key,
                    f'<i>Failed to prepare host path {html.escape(host_path)}: {html.escape(str(exc))}</i>',
                )
                continue
            commands.append(['docker', 'cp', f'{container_ref}:{container_path}', host_path])
        return commands

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

    def _discover_filtered_image_records(
        self,
        blacklist_patterns: list[str] | None = None,
        discovery_filters: list[str] | None = None,
    ) -> tuple[list[dict[str, str]], str | None]:
        images_cfg = self._images_cfg
        raw_filters = (
            self._image_discovery_filters()
            if discovery_filters is None
            else discovery_filters
        )
        filters = [f.lower() for f in raw_filters if f]
        include_none = bool(images_cfg.get('include_none_tag', False))
        active_blacklist = (
            self._image_blacklist_patterns()
            if blacklist_patterns is None
            else blacklist_patterns
        )
        records: list[dict[str, str]] = []
        error_message: str | None = None

        try:
            run_kwargs = {'stdout': subprocess.PIPE, 'stderr': subprocess.PIPE, 'text': True, 'check': False}
            run_kwargs = self._prepare_run_env(run_kwargs)
            cp = subprocess.run(['docker', 'images', '--format', '{{json .}}'], **run_kwargs)
        except Exception as exc:
            error_message = f'Failed to list docker images: {exc}'
            self._console_log(1, error_message)
            return [], error_message

        if cp.returncode not in (0, None):
            error_message = f'docker images returned {cp.returncode}'
            self._console_log(1, error_message)

        output_lines = (cp.stdout or '').splitlines() if isinstance(cp.stdout, str) else []
        for line in output_lines:
            line = line.strip()
            if not line:
                continue
            try:
                entry = json.loads(line)
            except json.JSONDecodeError:
                continue
            if not isinstance(entry, dict):
                continue
            repo = str(entry.get('Repository', '') or '')
            tag = str(entry.get('Tag', '') or '')
            if not include_none and ('<none>' in (repo.strip(), tag.strip())):
                continue
            ref = f'{repo}:{tag}' if tag else repo
            if not ref:
                continue
            if self._image_ref_blacklisted(ref, active_blacklist):
                continue
            if filters and not any(f in ref.lower() for f in filters):
                continue
            normalized_entry = {key: ('' if value is None else str(value)) for key, value in entry.items()}
            normalized_entry['ref'] = ref
            identifier = normalized_entry.get('ID') or normalized_entry.get('Id') or ref
            normalized_entry['identifier'] = identifier
            records.append(normalized_entry)

        return records, error_message

    def _manage_images_detail_level(self) -> str:
        value = str(self._images_cfg.get('manage_dialog_detail', 'simple') or '').strip().lower()
        if value in {'simple', 'medium', 'full'}:
            return value
        return 'simple'

    def _load_available_images(self, show_feedback: bool = False):
        images_cfg = self._images_cfg
        records, error_message = self._discover_filtered_image_records()

        choices = [record.get('ref', '') for record in records if record.get('ref')]
        choices = [choice for choice in dict.fromkeys(choice for choice in choices if choice)]

        if not choices:
            self._selected_image = ''
            self._image_choices = []
            self.image_combo.blockSignals(True)
            self.image_combo.clear()
            self.image_combo.addItem('No images found')
            self.image_combo.setEnabled(False)
            self.image_combo.blockSignals(False)
            self.image_combo.setToolTip('No image selected')
            if self._can_offer_setup_wizard():
                self._console_log(
                    1,
                    'no matching Docker images found; opening setup wizard'
                )
                QTimer.singleShot(0, self._open_setup_wizard)
                return
            self._inform_no_images_and_exit()
            return

        default_image = str(images_cfg.get('default', '') or '').strip()
        default_available = default_image in choices
        if default_available:
            ordered_choices = [default_image] + [choice for choice in choices if choice != default_image]
        else:
            ordered_choices = list(choices)

        workspace_image = (
            self._workspace_match_image(
                self._workspace_registry.active,
                ordered_choices,
            )
            or self._workspace_image(self._workspace_registry.active)
        )
        preferred_image = workspace_image or self._selected_image
        if preferred_image in ordered_choices:
            self._selected_image = preferred_image
        else:
            self._selected_image = ordered_choices[0]

        self._image_choices = ordered_choices

        self.image_combo.blockSignals(True)
        self.image_combo.clear()
        for index, choice in enumerate(ordered_choices):
            self.image_combo.addItem(self._image_choice_label(choice), choice)
            self._decorate_image_combo_item(index, choice)
        index = ordered_choices.index(self._selected_image)
        self.image_combo.setCurrentIndex(index)
        self.image_combo.setEnabled(True)
        self.image_combo.blockSignals(False)
        self.image_combo.setToolTip(
            self._image_choice_tooltip(self._selected_image)
            if self._selected_image
            else 'No image selected'
        )

        if show_feedback and error_message:
            QMessageBox.warning(self, 'Images', error_message)

        if show_feedback:
            self._console_log(2, f'Available images: {", ".join(ordered_choices)}')

        if (
            default_image
            and not default_available
        ):
            if self._should_offer_setup_for_missing_default_image():
                self._console_log(
                    1,
                    'configured default Docker image is missing; '
                    'opening setup wizard',
                )
                QTimer.singleShot(0, self._open_setup_wizard)
            elif os.environ.get('QT_QPA_PLATFORM', '').strip().lower() != 'offscreen':
                self._show_missing_default_image_dialog(default_image)

        self._update_related_patterns()
        self._populate_workspace_combo()
        self._apply_env_to_all_tabs()

    def _apply_env_to_all_tabs(self):
        for tab in self.tasks.values():
            tab.refresh_environment()

    def _inform_no_images_and_exit(self):
        filters = self._images_cfg.get('discovery_filters', [])
        filters_desc = ', '.join(str(item) for item in filters) if filters else '(no filters)'
        config_path = str(USER_CONFIG_FILE)
        message = (
            'No docker images matched the configured discovery filters.\n\n'
            f'Filters: {filters_desc}\n'
            f'Create or update the "images.discovery_filters" setting in '
            f'{config_path} '
            'to change which images are considered.\n\n'
            'The application will now exit.'
        )
        self._console_log(1, message)
        QMessageBox.critical(self, 'Images', message)
        app = QApplication.instance()
        if app is not None:
            QTimer.singleShot(0, app.quit)

    def _show_missing_default_image_dialog(self, image_ref: str):
        if self._default_image_dialog_shown:
            return
        self._default_image_dialog_shown = True

        command = f'docker pull {image_ref}'
        dialog = QDialog(self)
        dialog.setWindowTitle('Default Image Not Installed')

        layout = QVBoxLayout(dialog)
        message = QLabel(
            'The configured default docker image is not available locally.\n'
            f'Run the following command in a terminal to install "{image_ref}":'
        )
        message.setWordWrap(True)
        layout.addWidget(message)

        command_row = QHBoxLayout()
        command_edit = QLineEdit(command)
        command_edit.setReadOnly(True)
        command_edit.setFocusPolicy(Qt.StrongFocus)
        command_edit.selectAll()
        command_row.addWidget(command_edit)

        copy_button = QPushButton('Copy Command')

        def _copy_command():
            QApplication.clipboard().setText(command)
            copy_button.setText('Copied!')
            QTimer.singleShot(1500, lambda: copy_button.setText('Copy Command'))

        copy_button.clicked.connect(_copy_command)
        command_row.addWidget(copy_button)
        layout.addLayout(command_row)

        button_box = QDialogButtonBox(QDialogButtonBox.Ok)
        button_box.accepted.connect(dialog.accept)
        layout.addWidget(button_box)

        dialog.exec_()

    def _select_image(
        self,
        image_ref: str,
        *,
        log_selection: bool = True,
    ) -> bool:
        if image_ref not in self._image_choices:
            return False
        changed = image_ref != self._selected_image
        self._selected_image = image_ref
        self.image_combo.blockSignals(True)
        self.image_combo.setCurrentIndex(self._image_choices.index(image_ref))
        self.image_combo.blockSignals(False)
        self.image_combo.setToolTip(self._image_choice_tooltip(image_ref))
        if changed and log_selection:
            self._console_log(2, f'Selected image: {image_ref}')
        self._update_related_patterns()
        self._apply_env_to_all_tabs()
        self._populate_workspace_combo()
        return True

    def _on_image_changed(self, index: int):
        if not self._image_choices:
            return
        if index < 0 or index >= len(self._image_choices):
            return
        new_image = self._image_choices[index]
        self._select_image(new_image)

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

    def _log_button_click(self, button: QPushButton | None, fallback: str | None = None):
        label = button.text().strip() if button else ''
        if not label:
            label = fallback or 'button'
        self._log_event(f'user clicked {label}')

    def _on_sim_toggle_clicked(self):
        button = self._get_button_widget('sim')
        self._log_button_click(button, 'Sim Toggle')
        if not self._guard_toggle_action('sim', button):
            return
        self.toggle_sim()

    def _on_refresh_clicked(self):
        self._log_button_click(None, 'Update Status')
        self._log_info('refreshing sim status view')
        self.update_sim_status_from_poll(force=True)

    def _on_window_layout_clicked(self):
        self._log_button_click(self.window_layout_button, 'Window Layout')
        dialog = self._ensure_window_layout_dialog()
        dialog.show()
        dialog.raise_()
        dialog.activateWindow()

    def _compute_window_layout_delay_ms(self) -> int:
        raw = self._window_layout_cfg.get('apply_delay_ms', 'auto')
        if isinstance(raw, str):
            value = raw.strip().lower()
            if value in {'', 'auto'}:
                return self._window_layout_delay_from_timeline()
            try:
                return max(0, int(raw))
            except (TypeError, ValueError):
                return 0
        try:
            return max(0, int(raw))
        except (TypeError, ValueError):
            return 0

    def _window_layout_delay_from_timeline(self) -> int:
        try:
            timeline = self._launch_plan.get('timeline', []) if isinstance(self._launch_plan, dict) else []
            max_at = 0.0
            for entry in timeline:
                try:
                    max_at = max(max_at, float(entry.get('at_seconds', 0) or 0))
                except Exception:
                    continue
            if max_at <= 0:
                return 0
            return int(max_at * 1000) + 4000
        except Exception:
            return 0

    def _workspace_window_layout_path(self) -> Path:
        workspace_name = str(self._workspace_registry.active or '').strip()
        workspace_slug = self._normalize_workspace_name(
            workspace_name or 'docker_image_default'
        )
        template = str(self._window_layout_path_template or WINDOW_LAYOUT_FILE)
        if '{workspace' in template:
            formatted = template.format(
                workspace=workspace_slug,
                workspace_slug=workspace_slug,
            )
            return self._resolve_window_layout_path(formatted)

        base_path = self._resolve_window_layout_path(template)
        if base_path.suffix:
            return base_path.with_suffix('') / f'{workspace_slug}{base_path.suffix}'
        return base_path / f'{workspace_slug}.yaml'

    def _resolve_window_layout_path(self, value: str) -> Path:
        layout_path = Path(value).expanduser()
        if not layout_path.is_absolute():
            layout_path = PROJECT_ROOT / layout_path
        return layout_path

    def _reload_window_layout_for_workspace(self) -> None:
        self._window_layout_path = self._workspace_window_layout_path()
        self._window_layout_manager.state_file = self._window_layout_path
        self._window_layout_manager.load_layout()
        if self._window_layout_dialog is not None:
            self._window_layout_dialog.deleteLater()
            self._window_layout_dialog = None

    def _ensure_window_layout_dialog(self) -> QDialog:
        if self._window_layout_dialog is not None:
            return self._window_layout_dialog
        dialog = QDialog(None)  # top-level so minimizing the main GUI leaves it visible
        dialog.setWindowTitle('Window Layout Helper')
        dialog.setWindowFlag(Qt.WindowStaysOnTopHint, True)
        dialog.setWindowModality(Qt.NonModal)
        layout = QVBoxLayout(dialog)
        label = QLabel('Capture and reuse window positions with wmctrl.')
        label.setWordWrap(True)
        layout.addWidget(label)
        path_label = QLabel(str(self._window_layout_path))
        path_label.setTextInteractionFlags(Qt.TextSelectableByMouse)
        layout.addWidget(path_label)
        save_button = QPushButton('Save Window State')
        save_button.clicked.connect(self._on_save_window_state_clicked)
        layout.addWidget(save_button)
        dialog.setLayout(layout)
        dialog.setSizeGripEnabled(False)
        dialog.setMinimumWidth(280)
        dialog.setMaximumWidth(360)
        self._window_layout_dialog = dialog
        return dialog

    def _on_save_window_state_clicked(self):
        exclude_titles = {self.windowTitle()}
        if self._window_layout_dialog:
            exclude_titles.add(self._window_layout_dialog.windowTitle())
        success = self._window_layout_manager.capture_and_save(exclude_titles=exclude_titles)
        if success:
            self._append_gui_html(
                'log',
                f'<i>Window layout saved to {html.escape(str(self._window_layout_path))}</i>',
            )
            if self._window_layout_dialog:
                self._window_layout_dialog.close()
            return
        QMessageBox.warning(
            self,
            'Window Layout',
            'Unable to capture window state. Ensure wmctrl/xprop are installed and windows are visible.',
        )

    def _on_stop_custom_clicked(self):
        self._log_button_click(self.stop_custom_button, 'Stop Command')
        self._log_info('stopping custom command tab')
        self.stop_custom_process()

    def _on_tables_toggle_clicked(self):
        button = self._get_button_widget('tables')
        self._log_button_click(button, 'Tables Demo')
        if not self._guard_toggle_action('tables', button):
            return
        self.toggle_tables_demo()

    def _on_rviz_toggle_clicked(self):
        button = self._get_button_widget('rviz')
        self._log_button_click(button, 'RViz')
        if not self._guard_toggle_action('rviz', button):
            return
        self.toggle_rviz()

    def _on_rqt_toggle_clicked(self):
        button = self._get_button_widget('rqt')
        self._log_button_click(button, 'RQt Tables Demo')
        if not self._guard_toggle_action('rqt', button):
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

    def _on_auto_launch_toggle_clicked(self):
        self._log_button_click(self.auto_launch_button, self._auto_launch_label())
        if not self._guard_toggle_action('auto_launch', self.auto_launch_button):
            return
        self._toggle_auto_launch_stack()

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

        if not self._confirm_workspace_mismatch_warning(f'Script "{script}"'):
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
            self._claim_xhost(tab, key_target, log_key=tab.key)
            inner = f"python3 {CONTAINER_SCRIPTS_DIR}/{self._sh_quote(script)}"
            args = [
                'compose', 'run', '--rm', '--name', tab.container_name,
                '--label', f'mobipick.exec={exec_id}', '--label', f'mobipick.tab={key_target}',
                *self._compose_env_args(container_name=tab.container_name),
                self._ros_tool_service(),
                'bash',
                '-lc',
                self._wrap_line_buffered(inner),
            ]
            tab.start_program('docker', args)
            self._schedule_host_to_container_copy(tab)
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

    # ---------- Recording ----------

    def _resolve_recording_output_root(self) -> Path:
        raw = str(self._recording_cfg.get('output_dir') or '')
        path = Path(raw).expanduser()
        if not path.is_absolute():
            path = PROJECT_ROOT / path
        return path

    def _update_recording_location_tooltip(self) -> None:
        text = (
            'Auto Launch recordings are saved under:\n'
            f'{self._recording_output_root}\n\n'
            'A timestamped folder is created for each recording with the '
            'screen video and saved logs. Recording starts after Auto Launch '
            'finishes its timeline and window layout delay. It stops when '
            'Record Auto Launch is unchecked, Stop Recording '
            'is pressed in the optional pop-out window, Auto Launch is stopped, '
            'Roscore stops, or the GUI exits.'
        )
        if getattr(self, 'record_checkbox', None):
            self.record_checkbox.setToolTip(text)
        if getattr(self, 'recording_options_button', None):
            self.recording_options_button.setToolTip(
                text + '\n\nClick to choose a different recording folder.'
            )
        if getattr(self, 'recording_indicator', None):
            self.recording_indicator.setToolTip(
                'Red flashing means Auto Launch recording is actively running. '
                'REC armed means recording is waiting for you to press Auto Launch.'
            )

    def _on_recording_popup_toggled(self, checked: bool) -> None:
        self._recording_show_control_window = checked
        self._recording_cfg['show_control_window'] = checked
        try:
            save_user_config_update({
                'recording': {'show_control_window': checked},
            })
        except Exception as exc:
            self._append_gui_html(
                'log',
                '<i>Failed to save recording control window preference: '
                f'{html.escape(str(exc))}</i>',
            )
        if checked and self._recording_session:
            video_path = self._recording_session.get('video_path')
            if video_path:
                self._show_recording_window(Path(video_path))
        elif not checked and self._recording_window:
            self._recording_window.hide()

    def _open_recording_options(self) -> None:
        dialog = QDialog(self)
        dialog.setWindowTitle('Recording Options')
        layout = QVBoxLayout(dialog)

        summary = QLabel(
            'Recording captures the Auto Launch run: screen video plus saved '
            'GUI logs. It starts after you press Auto Launch and the launch '
            'timeline/window-layout delay finishes.'
        )
        summary.setWordWrap(True)
        layout.addWidget(summary)

        form = QFormLayout()
        path_row = QHBoxLayout()
        path_edit = QLineEdit(str(self._recording_output_root))
        path_row.addWidget(path_edit)
        browse = QPushButton('Browse')
        path_row.addWidget(browse)
        form.addRow('Save under:', path_row)

        remember = QCheckBox('Remember this folder')
        remember.setChecked(self._recording_remember_output_dir)
        form.addRow('', remember)

        resolution = QComboBox()
        resolution.setInsertPolicy(QComboBox.NoInsert)
        resolution.addItems(self._recording_resolutions)
        current = self._current_recording_resolution()
        index = resolution.findText(current)
        if index >= 0:
            resolution.setCurrentIndex(index)
        form.addRow('Screen resolution:', resolution)

        stop_window = QCheckBox('Show always-on-top Stop Recording window')
        stop_window.setChecked(self._recording_show_control_window)
        form.addRow('', stop_window)
        layout.addLayout(form)

        buttons = QDialogButtonBox(
            QDialogButtonBox.Save | QDialogButtonBox.Cancel
        )
        layout.addWidget(buttons)

        def _browse() -> None:
            selected = QFileDialog.getExistingDirectory(
                dialog,
                'Choose recording folder',
                path_edit.text().strip() or str(self._recording_output_root),
            )
            if selected:
                path_edit.setText(selected)

        def _save() -> None:
            raw = path_edit.text().strip()
            if not raw:
                QMessageBox.warning(
                    dialog,
                    'Recording Options',
                    'Choose a folder for recordings.',
                )
                return
            output_dir = Path(raw).expanduser()
            if not output_dir.is_absolute():
                output_dir = PROJECT_ROOT / output_dir
            selected_resolution = self._normalize_resolution_string(
                resolution.currentText()
            ) or self._select_default_resolution()
            self._recording_output_root = output_dir
            self._recording_counter = self._load_recording_counter()
            self._recording_remember_output_dir = remember.isChecked()
            self._recording_show_control_window = stop_window.isChecked()
            self._recording_cfg['output_dir'] = str(output_dir)
            self._recording_cfg['remember_output_dir'] = (
                self._recording_remember_output_dir
            )
            self._recording_cfg['show_control_window'] = (
                self._recording_show_control_window
            )
            self._recording_cfg['resolution'] = selected_resolution
            combo_index = self.record_resolution_combo.findText(
                selected_resolution
            )
            if combo_index >= 0:
                self.record_resolution_combo.setCurrentIndex(combo_index)
            try:
                save_user_config_update({
                    'recording': {
                        'output_dir': str(output_dir),
                        'remember_output_dir': (
                            self._recording_remember_output_dir
                        ),
                        'show_control_window': (
                            self._recording_show_control_window
                        ),
                        'resolution': selected_resolution,
                    },
                })
            except Exception as exc:
                self._append_gui_html(
                    'log',
                    '<i>Failed to save recording options: '
                    f'{html.escape(str(exc))}</i>',
                )
            self._update_recording_location_tooltip()
            self._log_info(f'recording options saved; output folder: {output_dir}')
            if self._recording_window and not self._recording_show_control_window:
                self._recording_window.hide()
            if self._recording_show_control_window and self._recording_session:
                video_path = self._recording_session.get('video_path')
                if video_path:
                    self._show_recording_window(Path(video_path))
            dialog.accept()

        browse.clicked.connect(_browse)
        buttons.accepted.connect(_save)
        buttons.rejected.connect(dialog.reject)
        dialog.exec_()

    def _set_recording_indicator(self, state: str) -> None:
        if not getattr(self, 'recording_indicator', None):
            return
        if state != 'active':
            self._stop_recording_indicator_flash()
        if state == 'active':
            self.recording_indicator.setText('● REC')
            self.recording_indicator.setToolTip(
                'Auto Launch recording is running. Uncheck Record Auto Launch, stop '
                'Auto Launch, or use the stop window to end it.'
            )
            self._set_auto_launch_recording_hint('active')
            self._start_recording_indicator_flash()
        elif state == 'armed':
            self.recording_indicator.setText('REC armed: press Auto Launch')
            self.recording_indicator.setToolTip(
                'Recording is armed but has not started. Press Auto Launch '
                'to trigger the launch timeline; recording starts after that '
                'timeline and the window-layout delay finish.'
            )
            self._set_auto_launch_recording_hint('armed')
            self.recording_indicator.setStyleSheet(
                'QLabel { color: #6c3b00; background: #fff3cd; '
                'border: 1px solid #f0c36d; border-radius: 4px; '
                'padding: 2px 6px; font-weight: bold; }'
            )
        else:
            self.recording_indicator.setText('REC off')
            self.recording_indicator.setToolTip(
                'Check Record Auto Launch to arm a recording for the next Auto Launch.'
            )
            self._set_auto_launch_recording_hint('off')
            self.recording_indicator.setStyleSheet(
                'QLabel { color: #666666; background: transparent; '
                'border: 1px solid transparent; padding: 2px 6px; }'
            )

    def _set_auto_launch_recording_hint(self, state: str) -> None:
        if not getattr(self, 'auto_launch_button', None):
            return
        base = getattr(self, '_auto_launch_base_tooltip', '') or ''
        if state == 'armed':
            hint = (
                'Recording is armed: click Auto Launch to trigger recording. '
                'Capture starts after the launch timeline and window-layout delay.'
            )
        elif state == 'active':
            hint = 'Recording is active: click Auto Launch to stop the launch stack and recording.'
        else:
            hint = ''
        tooltip = '\n\n'.join(part for part in (base, hint) if part)
        self.auto_launch_button.setToolTip(tooltip)

    def _start_recording_indicator_flash(self) -> None:
        if self._recording_indicator_timer is None:
            timer = QTimer(self)
            timer.timeout.connect(self._flash_recording_indicator)
            self._recording_indicator_timer = timer
        self._recording_indicator_on = True
        self._flash_recording_indicator()
        if not self._recording_indicator_timer.isActive():
            self._recording_indicator_timer.start(500)

    def _stop_recording_indicator_flash(self) -> None:
        timer = self._recording_indicator_timer
        if timer is not None and timer.isActive():
            timer.stop()
        self._recording_indicator_on = False

    def _flash_recording_indicator(self) -> None:
        if not getattr(self, 'recording_indicator', None):
            return
        if self._recording_indicator_on:
            self.recording_indicator.setStyleSheet(
                'QLabel { color: white; background: #d00000; '
                'border: 1px solid #7a0000; border-radius: 4px; '
                'padding: 2px 6px; font-weight: bold; }'
            )
        else:
            self.recording_indicator.setStyleSheet(
                'QLabel { color: #d00000; background: #ffe1e1; '
                'border: 1px solid #d00000; border-radius: 4px; '
                'padding: 2px 6px; font-weight: bold; }'
            )
        self._recording_indicator_on = not self._recording_indicator_on

    def _on_record_checkbox_toggled(self, checked: bool) -> None:
        if not checked:
            self._cancel_recording_schedule()
            if self._recording_is_active():
                self._stop_screen_recording(
                    save_logs=True,
                    reason='Record Auto Launch unchecked; stopping recording',
                )
            else:
                self._set_recording_indicator('off')
                self._log_info('Auto Launch recording disabled')
            return
        if self._recording_remember_output_dir:
            self._set_recording_indicator('armed')
            self._log_recording_armed()
            return
        if self._choose_recording_output_root(
            title='Choose Recording Folder',
            remember_default=False,
        ):
            self._set_recording_indicator('armed')
            self._log_recording_armed()
            return
        self.record_checkbox.blockSignals(True)
        self.record_checkbox.setChecked(False)
        self.record_checkbox.blockSignals(False)
        self._set_recording_indicator('off')
        self._log_info('Auto Launch recording was not enabled because no folder was selected')

    def _choose_recording_output_root(
        self,
        *,
        title: str,
        remember_default: bool,
    ) -> bool:
        selection = self._recording_output_dialog(title, remember_default)
        if selection is None:
            return False
        output_dir, remember = selection
        if not output_dir.is_absolute():
            output_dir = PROJECT_ROOT / output_dir
        self._recording_output_root = output_dir
        self._recording_counter = self._load_recording_counter()
        self._recording_remember_output_dir = remember
        self._recording_cfg['output_dir'] = str(output_dir)
        self._recording_cfg['remember_output_dir'] = remember
        if remember or remember_default:
            try:
                save_user_config_update({
                    'recording': {
                        'output_dir': str(output_dir),
                        'remember_output_dir': remember,
                    },
                })
            except Exception as exc:
                self._append_gui_html(
                    'log',
                    '<i>Failed to save recording folder preference: '
                    f'{html.escape(str(exc))}</i>',
                )
        self._update_recording_location_tooltip()
        self._log_info(f'Auto Launch recordings will be saved under {output_dir}')
        return True

    def _log_recording_armed(self) -> None:
        self._log_info(
            'Auto Launch recording armed; press Auto Launch to trigger it. '
            'Recording starts after Auto Launch finishes its timeline and '
            f'window layout delay. Output folder: '
            f'{self._recording_output_root}'
        )

    def _recording_output_dialog(
        self,
        title: str,
        remember_default: bool,
    ) -> tuple[Path, bool] | None:
        dialog = QDialog(self)
        dialog.setWindowTitle(title)
        layout = QVBoxLayout(dialog)

        label = QLabel(
            'Choose the folder that will contain Auto Launch recording '
            'sessions. Each recording saves screen video and logs in a '
            'timestamped subfolder. Recording is triggered by pressing Auto '
            'Launch, then starts after Auto Launch finishes its timeline and '
            'window layout delay. It stops when Record Auto Launch is '
            'unchecked, Stop Recording is pressed in the optional pop-out '
            'window, Auto Launch is stopped, Roscore stops, or the GUI exits.'
        )
        label.setWordWrap(True)
        layout.addWidget(label)

        path_row = QHBoxLayout()
        path_edit = QLineEdit(str(self._recording_output_root))
        path_edit.setMinimumWidth(420)
        path_row.addWidget(path_edit)
        browse = QPushButton('Browse')
        path_row.addWidget(browse)
        layout.addLayout(path_row)

        remember = QCheckBox('Remember this folder')
        remember.setChecked(remember_default)
        layout.addWidget(remember)

        buttons = QDialogButtonBox(
            QDialogButtonBox.Ok | QDialogButtonBox.Cancel
        )
        layout.addWidget(buttons)

        def _browse() -> None:
            selected = QFileDialog.getExistingDirectory(
                dialog,
                'Choose recording folder',
                path_edit.text().strip() or str(self._recording_output_root),
            )
            if selected:
                path_edit.setText(selected)

        def _accept() -> None:
            raw = path_edit.text().strip()
            if not raw:
                QMessageBox.warning(
                    dialog,
                    'Recording Folder',
                    'Choose a folder for Auto Launch recordings.',
                )
                return
            dialog.accept()

        browse.clicked.connect(_browse)
        buttons.accepted.connect(_accept)
        buttons.rejected.connect(dialog.reject)
        if dialog.exec_() != QDialog.Accepted:
            return None
        return Path(path_edit.text().strip()).expanduser(), remember.isChecked()

    def _load_recording_counter(self) -> int:
        try:
            root = self._recording_output_root
            if not root.exists():
                return 0
            max_idx = 0
            for entry in root.iterdir():
                if not entry.is_dir():
                    continue
                match = re.match(r'^(\d+)_', entry.name)
                if match:
                    try:
                        max_idx = max(max_idx, int(match.group(1)))
                    except ValueError:
                        continue
            return max_idx
        except Exception:
            return 0

    def _normalize_resolution_string(self, value) -> str:
        if not value:
            return ''
        text = str(value).lower().strip()
        match = re.match(r'^(\d{3,5})[xX](\d{3,5})$', text)
        if not match:
            return ''
        width, height = match.groups()
        return f'{int(width)}x{int(height)}'

    def _detect_screen_resolution(self) -> str:
        # Prefer actual monitor resolution via xrandr, then Qt, then configured fallback.
        try:
            cp = subprocess.run(
                ['bash', '-lc', "xrandr | awk '/\\*/ {print $1; exit}'"],
                stdout=subprocess.PIPE,
                stderr=subprocess.DEVNULL,
                text=True,
                check=False,
            )
            out = (cp.stdout or '').strip()
            normalized = self._normalize_resolution_string(out)
            if normalized:
                self._screen_resolution = normalized
                return normalized
        except Exception:
            pass

        try:
            screen = QGuiApplication.primaryScreen()
            if screen is not None:
                geometry = screen.geometry()
                size = geometry.size()
                if size and size.width() > 0 and size.height() > 0:
                    detected = f'{int(size.width())}x{int(size.height())}'
                    normalized = self._normalize_resolution_string(detected)
                    if normalized:
                        self._screen_resolution = normalized
                        return normalized
        except Exception:
            pass

        configured = self._normalize_resolution_string(self._recording_cfg.get('resolution') or '3440x1440')
        self._screen_resolution = configured
        return configured

    def _load_recording_resolutions(self) -> list[str]:
        options: list[str] = []
        detected = self._normalize_resolution_string(self._detect_screen_resolution())
        if detected:
            options.append(detected)
        configured = self._normalize_resolution_string(self._recording_cfg.get('resolution'))
        if configured:
            options.append(configured)
        presets = self._recording_cfg.get('presets') or []
        for entry in presets:
            normalized = self._normalize_resolution_string(entry)
            if normalized:
                options.append(normalized)
        options.extend(['3440x1440', '3840x2160', '2560x1440', '1920x1080', '1600x900', '1280x720'])
        return list(dict.fromkeys([opt for opt in options if opt]))

    def _select_default_resolution(self) -> str:
        detected = self._normalize_resolution_string(self._screen_resolution)
        if detected and detected in self._recording_resolutions:
            return detected
        preferred = self._normalize_resolution_string(self._recording_cfg.get('resolution'))
        if preferred and preferred in self._recording_resolutions:
            return preferred
        return self._recording_resolutions[0] if self._recording_resolutions else '3440x1440'

    def _current_recording_resolution(self) -> str:
        combo = getattr(self, 'record_resolution_combo', None)
        if combo is not None:
            value = self._normalize_resolution_string(combo.currentText())
            if value:
                return self._clamp_resolution_to_screen(value)
        return self._clamp_resolution_to_screen(self._select_default_resolution())

    def _recording_display(self) -> str:
        configured = str(self._recording_cfg.get('display') or '').strip()
        return configured or os.environ.get('DISPLAY') or ':1'

    def _clamp_resolution_to_screen(self, resolution: str) -> str:
        parts = resolution.split('x')
        if len(parts) != 2:
            return resolution
        try:
            width = int(parts[0])
            height = int(parts[1])
        except ValueError:
            return resolution
        screen_res = self._normalize_resolution_string(self._screen_resolution or self._detect_screen_resolution())
        if not screen_res:
            return resolution
        sw, sh = (int(x) for x in screen_res.split('x'))
        clamped_w = min(width, sw)
        clamped_h = min(height, sh)
        clamped = f'{clamped_w}x{clamped_h}'
        if clamped != resolution:
            self._log_info(f'Adjusting recording resolution from {resolution} to {clamped} to fit screen ({screen_res})')
        return clamped

    @staticmethod
    def _normalize_workspace_name(value) -> str:
        raw = str(value or 'workspace')
        slug = re.sub(r'[^a-zA-Z0-9_-]+', '_', raw).strip('_')
        return slug or 'workspace'

    def _recording_start_delay_ms(self) -> int:
        timeline = self._launch_plan.get('timeline') if isinstance(self._launch_plan, dict) else []
        max_at_ms = 0
        for entry in timeline:
            try:
                at_seconds = float(entry.get('at_seconds', 0) or 0)
            except Exception:
                continue
            max_at_ms = max(max_at_ms, int(max(0.0, at_seconds) * 1000))
        layout_delay = max(0, int(self._window_layout_delay_ms))
        try:
            extra_delay_ms = int(
                max(
                    0.0,
                    float(
                        self._launch_plan.get(
                            'recording_start_delay_seconds',
                            0,
                        )
                        or 0
                    ),
                )
                * 1000
            )
        except (TypeError, ValueError):
            extra_delay_ms = 0
        return max(max_at_ms, layout_delay) + extra_delay_ms

    def _schedule_recording_after_launch(self):
        self._cancel_recording_schedule()
        if not getattr(self, 'record_checkbox', None):
            return
        if not self.record_checkbox.isChecked():
            self._console_log(2, 'Auto Launch recording skipped (checkbox unchecked)')
            return
        if self._recording_is_active():
            return
        delay_ms = self._recording_start_delay_ms()
        if delay_ms <= 0:
            self._start_screen_recording()
            return

        timer = QTimer(self)
        timer.setSingleShot(True)

        def _fire():
            try:
                self._start_screen_recording()
            finally:
                if self._recording_start_timer is timer:
                    self._recording_start_timer = None
                timer.deleteLater()

        timer.timeout.connect(_fire)
        self._recording_start_timer = timer
        timer.start(delay_ms)
        self._log_info(f'scheduling Auto Launch recording in {delay_ms / 1000:.1f}s')

    def _cancel_recording_schedule(self):
        timer = self._recording_start_timer
        if timer is not None:
            try:
                timer.stop()
            except Exception:
                pass
            timer.deleteLater()
        self._recording_start_timer = None

    def _recording_is_active(self) -> bool:
        if self._recording_proc and self._recording_proc.state() != QProcess.NotRunning:
            return True
        return bool(self._recording_session)

    def _ensure_recording_window(self) -> QDialog:
        # recreate if previously closed/deleted
        try:
            if self._recording_window is not None:
                _ = self._recording_window.windowTitle()
                return self._recording_window
        except RuntimeError:
            self._recording_window = None

        dialog = QDialog(None)  # top-level so wmctrl can move it independently
        dialog.setWindowTitle('Recording Control')
        dialog.setWindowFlag(Qt.WindowStaysOnTopHint, True)
        dialog.setWindowFlag(Qt.Tool, True)
        dialog.setWindowModality(Qt.NonModal)
        dialog.setAttribute(Qt.WA_DeleteOnClose, False)
        layout = QVBoxLayout(dialog)
        label = QLabel('Screen recording is active.')
        label.setWordWrap(True)
        layout.addWidget(label)
        self._recording_path_label = QLabel('')
        self._recording_path_label.setTextInteractionFlags(Qt.TextSelectableByMouse)
        layout.addWidget(self._recording_path_label)
        button = QPushButton('Stop Recording')
        button.clicked.connect(self._on_recording_stop_clicked)
        layout.addWidget(button)
        self._recording_stop_button = button
        dialog.setLayout(layout)
        dialog.setMinimumWidth(260)
        dialog.setMaximumWidth(360)

        def _mark_closed():
            self._recording_window = None

        dialog.destroyed.connect(lambda *_: _mark_closed())
        self._recording_window = dialog
        return dialog

    def _show_recording_window(self, video_path: Path):
        dialog = self._ensure_recording_window()
        if self._recording_path_label:
            self._recording_path_label.setText(str(video_path))
        if self._recording_stop_button:
            self._recording_stop_button.setEnabled(True)
        dialog.show()
        dialog.raise_()
        dialog.activateWindow()

    def _on_recording_stop_clicked(self):
        self._log_event('user requested recording stop')
        if self._recording_stop_button:
            self._recording_stop_button.setEnabled(False)
        if self._auto_launch_running:
            self._stop_auto_launch_stack()
            return
        self._stop_screen_recording(save_logs=True, reason='Stopping recording')

    def _start_screen_recording(self):
        if self._recording_is_active():
            return
        try:
            self._recording_output_root.mkdir(parents=True, exist_ok=True)
        except Exception as exc:
            self._append_gui_html('log', f'<i>Failed to prepare recording directory: {html.escape(str(exc))}</i>')
            return
        self._log_info(f'preparing Auto Launch recording under {self._recording_output_root}')
        now = datetime.now()
        next_idx = max(self._recording_counter, self._load_recording_counter()) + 1
        self._recording_counter = next_idx
        base_name = f'{next_idx}_{self._recording_workspace_name}_{now:%d_%m_%H%M%S}'
        base_dir = self._recording_output_root / base_name
        try:
            base_dir.mkdir(parents=True, exist_ok=True)
        except Exception as exc:
            self._append_gui_html('log', f'<i>Failed to create recording folder: {html.escape(str(exc))}</i>')
            return
        video_path = base_dir / f'{base_name}.mp4'
        ffmpeg_log = base_dir / 'ffmpeg.log'
        self._log_info(f'Auto Launch recording session folder: {base_dir}')
        self._recording_session = {
            'base_dir': base_dir,
            'video_path': video_path,
            'logs_dir': base_dir / 'logs',
            'save_logs': False,
        }
        proc = QProcess(self)
        proc.setProgram('ffmpeg')
        proc.setProcessChannelMode(QProcess.SeparateChannels)
        try:
            proc.setStandardOutputFile(str(ffmpeg_log))
            proc.setStandardErrorFile(str(ffmpeg_log), mode=QIODevice.Append)
        except Exception:
            # fallback: let ffmpeg print to stdout/stderr
            pass
        requested_res = self._current_recording_resolution()
        display = self._recording_display()
        args = [
            '-f', 'x11grab',
            '-s', requested_res,
            '-r', '30',
            '-i', display,
            '-vcodec', 'libx264',
            '-pix_fmt', 'yuv420p',
            '-preset', 'veryfast',
            '-threads', '8',
            str(video_path),
        ]
        proc.finished.connect(self._on_recording_finished)
        proc.errorOccurred.connect(lambda _err: self._append_gui_html(
            'log',
            f'<i>Recording error: {html.escape(proc.errorString())}</i>',
        ))
        self._recording_proc = proc
        self._log_info(f'starting Auto Launch recording at {requested_res} to {video_path}')
        self._log_info(f'recording command: {shlex.join(["ffmpeg", *args])}')
        self._log_info(f'ffmpeg output log: {ffmpeg_log}')
        proc.start('ffmpeg', args)
        if not proc.waitForStarted(3000):
            self._append_gui_html(
                'log',
                '<i>Unable to start ffmpeg; recording cancelled. '
                f'Check {html.escape(str(ffmpeg_log))}.</i>',
            )
            self._recording_proc = None
            self._recording_session = None
            if getattr(self, 'record_checkbox', None):
                self.record_checkbox.blockSignals(True)
                self.record_checkbox.setChecked(False)
                self.record_checkbox.blockSignals(False)
            self._set_recording_indicator('off')
            return
        self._set_recording_indicator('active')
        if self._recording_show_control_window:
            self._show_recording_window(video_path)

    def _stop_screen_recording(self, *, save_logs: bool, reason: str | None = None):
        self._cancel_recording_schedule()
        session = self._recording_session
        if session is None and not self._recording_proc:
            return
        if session is not None:
            session['save_logs'] = session.get('save_logs', False) or save_logs
        if reason:
            self._log_info(reason)
        proc = self._recording_proc
        try:
            state = proc.state() if proc else None
        except RuntimeError:
            return
        if proc and state != QProcess.NotRunning:
            if not session.get('stop_requested'):
                session['stop_requested'] = True
                self._request_ffmpeg_stop(proc)
                QTimer.singleShot(
                    5000,
                    lambda: self._terminate_recording_if_running(proc),
                )
            return
        self._finalize_recording_session()

    def _request_ffmpeg_stop(self, proc: QProcess) -> None:
        self._log_info('requesting ffmpeg to finish recording cleanly')
        try:
            proc.write(b'q\n')
            proc.closeWriteChannel()
        except Exception as exc:
            self._append_gui_html(
                'log',
                '<i>Failed to send clean stop to ffmpeg; terminating: '
                f'{html.escape(str(exc))}</i>',
            )
            try:
                proc.terminate()
            except Exception:
                pass

    def _terminate_recording_if_running(self, proc: QProcess | None):
        if proc is None:
            return
        try:
            state = proc.state()
        except RuntimeError:
            return
        if state != QProcess.NotRunning:
            self._append_gui_html(
                'log',
                '<i>ffmpeg did not stop after clean quit request; terminating.</i>',
            )
            try:
                proc.terminate()
            except Exception:
                pass
            QTimer.singleShot(2000, lambda: self._force_kill_recording(proc))

    def _force_kill_recording(self, proc: QProcess | None):
        if proc is None:
            return
        try:
            state = proc.state()
        except RuntimeError:
            return
        if state != QProcess.NotRunning:
            try:
                proc.kill()
            except Exception:
                pass

    def _on_recording_finished(self, code: int, _status):
        if self._recording_session is not None:
            self._recording_session['exit_code'] = code
        self._finalize_recording_session()

    def _finalize_recording_session(self):
        session = self._recording_session
        proc = self._recording_proc
        self._recording_proc = None
        self._recording_session = None
        if self._recording_window:
            self._recording_window.hide()
        if getattr(self, 'record_checkbox', None) and self.record_checkbox.isChecked():
            self._set_recording_indicator('armed')
        else:
            self._set_recording_indicator('off')
        if session is None:
            return
        base_dir = session.get('base_dir')
        video_path = session.get('video_path')
        exit_code = session.get('exit_code')
        video_file = Path(video_path) if video_path else None
        video_exists = bool(video_file and video_file.is_file())
        if video_exists:
            size_kb = video_file.stat().st_size / 1024.0 if video_file else 0.0
            size_text = f' ({size_kb:.0f} KB)' if size_kb > 0 else ''
            self._append_gui_html('log', f'<i>Recording saved to {html.escape(str(video_file))}{html.escape(size_text)}</i>')
        if session.get('save_logs') and base_dir:
            logs_dir = session.get('logs_dir') or base_dir
            saved = self._save_logs_to_directory(Path(logs_dir))
            if saved:
                self._append_gui_html(
                    'log',
                    f'<i>Saved {saved} log file(s) to {html.escape(str(logs_dir))}</i>',
                )
        if not video_exists:
            self._append_gui_html(
                'log',
                '<i>No recording video was produced. Check '
                f'{html.escape(str(base_dir / "ffmpeg.log"))}.</i>',
            )
            QMessageBox.warning(
                self,
                'Recording',
                f'No video file was produced. Check ffmpeg.log in {base_dir}.',
            )
        elif exit_code not in (None, 0):
            # ffmpeg exits non-zero when we terminate it; suppress dialog if video is present.
            self._console_log(2, f'Recording finished with exit code {exit_code} (video saved).')
        if proc:
            try:
                proc.deleteLater()
            except Exception:
                pass

    def _toggle_auto_launch_stack(self):
        if self._auto_launch_running:
            self._stop_auto_launch_stack()
        else:
            self._start_auto_launch_stack()

    def _start_auto_launch_stack(self):
        self._auto_launch_stopping = False
        timeline = self._launch_plan.get('timeline') if isinstance(self._launch_plan, dict) else []
        if not timeline:
            message = QMessageBox(self)
            message.setIcon(QMessageBox.Information)
            message.setWindowTitle('Auto Launch')
            text = (
                'No launch sequence is configured. Configure automation to '
                'create one from the available buttons.'
            )
            if getattr(self, 'record_checkbox', None) and self.record_checkbox.isChecked():
                text += (
                    '\n\nRecord Auto Launch is armed, but recording starts only '
                    'from an Auto Launch run. Configure an Auto Launch '
                    'sequence first, then press Auto Launch again.'
                )
                self._log_info(
                    'Auto Launch recording is armed but cannot start because no '
                    'Auto Launch sequence is configured'
                )
            message.setText(text)
            configure_button = message.addButton('Configure', QMessageBox.ActionRole)
            message.addButton(QMessageBox.Ok)
            message.exec_()
            if message.clickedButton() == configure_button:
                self._open_auto_launch_wizard()
            self._auto_launch_running = False
            self.set_auto_launch_visual('red', self._auto_launch_start_text(), True)
            return

        if not self._confirm_workspace_mismatch_warning(self._auto_launch_label()):
            self._auto_launch_running = False
            self.set_auto_launch_visual('red', self._auto_launch_start_text(), True)
            return

        if self._auto_launch_run_count > 0:
            self.clear_all_tabs()
            self._append_gui_html('log', '<i>Cleared tabs before starting a new auto launch run.</i>')
        self._auto_launch_run_count += 1

        if self._window_layout_auto_apply and self._window_layout_manager:
            self._window_layout_manager.reset_auto_apply()

        self._cancel_auto_launch_timers()
        self._auto_launch_running = True
        self._auto_launch_active_keys = [
            entry.get('button') for entry in timeline if isinstance(entry, dict) and entry.get('button')
        ]
        source = self._launch_plan.get('source', 'configuration')
        self._log_info(f'starting auto launch timeline from {source}')
        self.set_auto_launch_visual('green', self._auto_launch_stop_text(), True)
        for entry in timeline:
            key = entry.get('button') if isinstance(entry, dict) else None
            if not key:
                continue
            try:
                at_seconds = max(0.0, float(entry.get('at_seconds', 0)))
            except (TypeError, ValueError):
                at_seconds = 0.0
            delay_ms = int(at_seconds * 1000)
            if delay_ms <= 0:
                self._trigger_auto_launch_step(key, target_running=True)
            else:
                self._schedule_auto_launch_step(key, delay_ms)
        self._schedule_recording_after_launch()

    def _auto_launch_wizard_buttons(self) -> list[tuple[str, str]]:
        buttons: list[tuple[str, str]] = [('roscore', 'Roscore')]
        for key in self._config_button_order:
            cfg = self._config_buttons.get(key, {})
            label = str(cfg.get('label') or key)
            buttons.append((key, label))
        buttons.append(('terminal', 'Terminal'))
        return list(dict.fromkeys(buttons))

    def _open_auto_launch_wizard(self):
        source = self._launch_plan.get('source') if isinstance(self._launch_plan, dict) else None
        save_path = writable_launch_sequence_path(source)
        recording_start_delay_seconds = (
            self._launch_plan.get('recording_start_delay_seconds', 0.0)
            if isinstance(self._launch_plan, dict)
            else 0.0
        )
        dialog = AutoLaunchWizard(
            self._auto_launch_wizard_buttons(),
            self._launch_plan.get('timeline', []) if isinstance(self._launch_plan, dict) else [],
            save_path,
            recording_start_delay_seconds,
            self,
        )
        if dialog.exec_() != QDialog.Accepted:
            return

        timeline = dialog.timeline()
        shutdown_order = [entry['button'] for entry in reversed(timeline)]
        recording_delay = dialog.recording_start_delay_seconds()
        try:
            saved_path = save_launch_sequence_plan(
                save_path,
                timeline,
                shutdown_order,
                self._auto_launch_button_cfg(),
                recording_delay,
            )
        except OSError as exc:
            QMessageBox.warning(
                self,
                'Auto Launch',
                f'Failed to save auto launch configuration:\n{exc}',
            )
            return

        self._launch_plan = load_launch_sequence_plan(
            self._workspace_button_config_path(),
            saved_path,
        )
        self._refresh_launch_plan_settings()
        self.set_auto_launch_visual('red', self._auto_launch_start_text(), True)
        self._log_info(f'saved auto launch configuration to {saved_path}')

    def _stop_auto_launch_stack(self):
        if self._auto_launch_stopping:
            return
        self._auto_launch_stopping = True
        self._cancel_auto_launch_timers()
        self._cancel_recording_schedule()
        order = self._auto_launch_shutdown_order()
        self._auto_launch_running = False
        if order:
            self.set_auto_launch_visual('yellow', 'Stopping Auto Launch...', False)
            self._flush_ui_events()
            for key in order:
                self._trigger_auto_launch_step(key, target_running=False)
        if self._recording_is_active():
            self._stop_screen_recording(save_logs=True, reason='Stopping recording after auto launch toggle')
        if self._roscore_stopping:
            return
        self._finalize_auto_launch_stop()

    def _finalize_auto_launch_stop(self):
        self._auto_launch_running = False
        self._auto_launch_stopping = False
        self._auto_launch_active_keys.clear()
        self.set_auto_launch_visual('red', self._auto_launch_start_text(), True)

    def _flush_ui_events(self):
        app = QApplication.instance()
        if app:
            app.processEvents()

    def _auto_launch_shutdown_order(self) -> list[str]:
        plan_order = []
        if isinstance(self._launch_plan, dict):
            raw = self._launch_plan.get('shutdown_order') or []
            if isinstance(raw, list):
                plan_order = [str(item).strip() for item in raw if str(item).strip()]
            if not plan_order:
                plan_order = [
                    entry.get('button')
                    for entry in self._launch_plan.get('timeline', [])
                    if isinstance(entry, dict) and entry.get('button')
                ]
        skip = set(self._launch_plan.get('shutdown_skip', [])) if isinstance(self._launch_plan, dict) else set()
        plan_order = [entry for entry in plan_order if entry and entry not in skip]
        if not self._auto_launch_active_keys:
            return plan_order
        ordered = [key for key in plan_order if key in self._auto_launch_active_keys and key not in skip]
        remaining = [
            key for key in self._auto_launch_active_keys
            if key and key not in ordered and key not in skip
        ]
        return list(dict.fromkeys(ordered + remaining))

    def _schedule_auto_launch_step(self, key: str, delay_ms: int):
        timer = QTimer(self)
        timer.setSingleShot(True)

        def _fire():
            try:
                self._trigger_auto_launch_step(key, target_running=True)
            finally:
                if timer in self._auto_launch_timers:
                    self._auto_launch_timers.remove(timer)
                timer.deleteLater()

        timer.timeout.connect(_fire)
        self._auto_launch_timers.append(timer)
        timer.start(max(0, int(delay_ms)))

    def _trigger_auto_launch_step(self, key: str, *, target_running: bool, attempt: int = 0):
        if not key:
            return
        if target_running and not self._auto_launch_running:
            return

        if self._toggle_states.get(key) == 'yellow':
            if attempt >= self._launch_max_retry:
                self._log_info(f'auto launch: skipping {key} (busy state)')
                return
            delay = self._launch_retry_ms or 500
            QTimer.singleShot(
                delay,
                lambda k=key, a=attempt + 1: self._trigger_auto_launch_step(k, target_running=target_running, attempt=a),
            )
            return

        self._ensure_button_state(key, target_running)

    def _ensure_button_state(self, key: str, target_running: bool):
        running = self._is_button_running(key)
        if target_running and running:
            return
        if not target_running and not running:
            return
        verb = 'starting' if target_running else 'stopping'
        self._log_info(f'auto launch: {verb} {key}')
        self._dispatch_auto_launch_toggle(key)

    def _dispatch_auto_launch_toggle(self, key: str) -> bool:
        if key == 'roscore':
            self.toggle_roscore()
            return True
        if key == 'terminal':
            self.toggle_terminal()
            return True
        if key in self._config_buttons:
            self._on_config_button_clicked(key)
            return True
        self._log_info(f'auto launch: no action found for "{key}"')
        return False

    def _cancel_auto_launch_timers(self):
        for timer in list(self._auto_launch_timers):
            try:
                timer.stop()
            except Exception:
                pass
            timer.deleteLater()
        self._auto_launch_timers.clear()

    def _is_button_running(self, key: str) -> bool:
        if key == 'roscore':
            return self.is_roscore_running()
        if key == 'sim':
            return self.is_sim_running()
        tab = self.tasks.get(key)
        if tab:
            return tab.is_running()
        return False

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
            if path.is_file() and entry.endswith('.py') and entry != 'enter_host_shell.py':
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
        if not (
            key.startswith('custom')
            or key.startswith('loadedlog')
            or key.startswith('terminal')
            or key.startswith('build-')
        ):
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

    def _grant_x(self, source: str, *, log_key: str | None = None):
        if source in self._xhost_sources:
            return
        if not self._xhost_sources:
            self._sp_run(['xhost', '+local:root'], check=False, log_key=log_key or 'sim')
        self._xhost_sources.add(source)

    def _revoke_x(self, source: str | None = None, *, log_key: str | None = None):
        if source is None:
            if not self._xhost_sources:
                return
            self._xhost_sources.clear()
            self._sp_run(['xhost', '-local:root'], check=False, log_key=log_key or 'sim')
            return
        if source not in self._xhost_sources:
            return
        self._xhost_sources.remove(source)
        if not self._xhost_sources:
            self._sp_run(['xhost', '-local:root'], check=False, log_key=log_key or 'sim')

    def _claim_xhost(self, tab: ProcessTab, token: str, *, log_key: str | None = None):
        tab.xhost_token = token
        self._grant_x(token, log_key=log_key or tab.key)

    def _release_xhost(self, tab: ProcessTab, *, log_key: str | None = None):
        token = getattr(tab, 'xhost_token', None)
        if not token:
            return
        tab.xhost_token = None
        self._revoke_x(token, log_key=log_key or tab.key)

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
            if not self._confirm_workspace_mismatch_warning('Roscore'):
                return
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

    def _ensure_roscore_ready(
        self,
        callback: Callable[[], None],
        *,
        attempt: int = 0,
        allow_autostart: bool = True,
    ):
        if self._remote_master_enabled():
            callback()
            return

        delay_ms = self._roscore_delay_ms()
        last_start = self._roscore_last_start_ts

        roscore_tab = self.tasks.get('roscore')
        roscore_active = self._roscore_running_cached or (
            roscore_tab.is_running() if roscore_tab else False
        )

        if roscore_active:
            if delay_ms > 0 and last_start is not None:
                elapsed_ms = int((time.monotonic() - last_start) * 1000)
                remaining = delay_ms - elapsed_ms
                if remaining > 0:
                    QTimer.singleShot(
                        remaining,
                        lambda: self._ensure_roscore_ready(
                            callback,
                            attempt=attempt + 1,
                            allow_autostart=allow_autostart,
                        ),
                    )
                    return
            callback()
            return

        if not allow_autostart:
            callback()
            return

        if self._roscore_stopping:
            self._log_info('roscore is shutting down; retrying shortly...')
            QTimer.singleShot(
                delay_ms or 1000,
                lambda: self._ensure_roscore_ready(
                    callback,
                    attempt=attempt + 1,
                    allow_autostart=allow_autostart,
                ),
            )
            return

        try:
            if self.is_roscore_running():
                self._roscore_running_cached = True
                self.set_roscore_visual('green', 'Stop Roscore', enabled=True)
                QTimer.singleShot(
                    delay_ms or 0,
                    lambda: self._ensure_roscore_ready(
                        callback,
                        attempt=attempt + 1,
                        allow_autostart=allow_autostart,
                    ),
                )
                return
        except Exception:
            pass

        self._log_info('roscore not running; starting automatically')
        self.bring_up_roscore()
        QTimer.singleShot(
            delay_ms or 1000,
            lambda: self._ensure_roscore_ready(
                callback,
                attempt=attempt + 1,
                allow_autostart=allow_autostart,
            ),
        )

    def bring_up_roscore(self):
        if self._remote_master_enabled():
            self._log_info(
                'remote ROS master mode is active; local roscore was not '
                'started'
            )
            return
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
            *self._compose_env_args(container_name=self._roscore_container_name),
            'mobipick_cmd', 'bash', '-lc', self._wrap_line_buffered(inner)
        ]
        tab.start_program('docker', args)
        self._schedule_host_to_container_copy(tab)
        self._focus_tab('roscore')

    def shutdown_roscore(self):
        if self._roscore_stopping:
            return
        self._roscore_stopping = True
        self._stop_auto_launch_stack()
        self._log_info('stopping roscore master')
        self.set_roscore_visual('yellow', 'Shutting down...', enabled=False)

        sim_tab = self.tasks.get('sim')
        sim_running = bool(sim_tab and sim_tab.is_running())
        if not sim_running:
            sim_running = bool(self._sim_running_cached)

        if sim_running:
            self._killing = True
            self.set_toggle_visual('yellow', 'Shutting down...', False)
        else:
            self._killing = False
            self._disable_toggle_preserving_visual('sim', self._get_button_widget('sim'))
        tables_running = 'tables' in self.tasks and self.tasks['tables'].is_running()
        rviz_running = 'rviz' in self.tasks and self.tasks['rviz'].is_running()
        rqt_running = 'rqt' in self.tasks and self.tasks['rqt'].is_running()
        script_running = False
        if self._script_active_tab_key and self._script_active_tab_key in self.tasks:
            script_running = self.tasks[self._script_active_tab_key].is_running()

        if tables_running:
            self.set_tables_visual('yellow', 'Shutting down...', False)
        else:
            self._disable_toggle_preserving_visual('tables', self._get_button_widget('tables'))

        if rviz_running:
            self.set_rviz_visual('yellow', 'Shutting down...', False)
        else:
            self._disable_toggle_preserving_visual('rviz', self._get_button_widget('rviz'))

        if rqt_running:
            self.set_rqt_visual('yellow', 'Shutting down...', False)
        else:
            self._disable_toggle_preserving_visual('rqt', self._get_button_widget('rqt'))

        if script_running:
            self.set_script_visual('yellow', 'Shutting down...', False)
        else:
            self._disable_toggle_preserving_visual('script', self.run_script_button)

        for cfg_key in self._config_button_order:
            cfg = self._config_buttons.get(cfg_key, {})
            tab_obj = self.tasks.get(cfg_key)
            running = bool(tab_obj and tab_obj.is_running())
            button = self._get_button_widget(cfg_key)
            if running:
                self._set_config_visual(cfg, 'yellow', 'Shutting down...', False)
            else:
                self._disable_toggle_preserving_visual(cfg_key, button)

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
                self._reset_config_button_visuals()
                self._script_active_tab_key = None
                self.set_script_visual('red', 'Run Script', bool(self._script_choices))
                self._terminal_stopping = False
                self._terminal_running_cached = False
                if self._terminal_stream_tab_key and self._terminal_stream_tab_key in self.tasks:
                    self._append_gui_html(self._terminal_stream_tab_key, '<i>Terminal session closed.</i>')
                    self._terminal_stream_tab_key = None
                self.set_terminal_visual('red', 'Open Terminal', True)
                self._update_stop_custom_enabled()
                self._finalize_auto_launch_stop()

            if commands:
                self._run_command_sequence(commands, on_finished=_finalize, log_key=tab.key)
            else:
                _finalize()

        delay = int(self._timers_cfg.get('custom_tab_sigint_delay_ms', 1000))
        QTimer.singleShot(delay, _cleanup)

    # event driven bring up
    def bring_up_sim(self):
        if self._remote_master_enabled():
            self._log_info(
                'simulation is disabled while remote ROS master mode is active'
            )
            return
        if self._killing:
            return
        if not self._confirm_workspace_mismatch_warning('Simulation'):
            return
        self.set_toggle_visual('yellow', 'Starting Sim...', False)

        def _start_sim():
            world = self._current_world()
            self._log_info(f'starting simulation stack (world {world})')
            tab = self._ensure_tab('sim', 'Sim', closable=False)
            tab.container_name = self._sim_container_name  # ensure sim tab is addressable
            exec_id = uuid.uuid4().hex
            tab.exec_id = exec_id

            self._claim_xhost(tab, 'sim', log_key=tab.key)

            args = [
                'compose', 'run', '--rm', '--name', self._sim_container_name,
                '--label', f'mobipick.exec={exec_id}', '--label', f'mobipick.tab={tab.key}',
                *self._compose_env_args(),
                'mobipick',
                'bash',
                '-lc',
                self._wrap_line_buffered(self._workspace_sim_command()),
            ]
            tab.start_program('docker', args)
            self._schedule_host_to_container_copy(tab)
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
            stop_cmd = self._docker_stop_display(label)
            self._append_gui_html(tab.key, f'<i>{html.escape(stop_cmd)}</i>')
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
                self._release_xhost(tab, log_key=tab.key)
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
            commands.append(self._safe_docker_cmd(*self._docker_stop_args(cid)))
        return commands

    def _docker_stop_if_exists(self, name: str | None, tab: ProcessTab | None = None, exec_id: str | None = None) -> list[list[str]]:
        commands: list[list[str]] = []
        ids = self._resolve_container_ids(name=name, exec_id=exec_id)
        for cid in ids:
            commands.append(self._safe_docker_cmd(*self._docker_stop_args(cid)))
            if tab:
                stop_cmd = self._docker_stop_display(cid)
                self._append_gui_html(tab.key, f'<i>{html.escape(stop_cmd)}</i>')
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
                    commands.append(self._safe_docker_cmd(*self._docker_stop_args(cid)))

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

    def _set_toggle_state(self, key: str, button: QPushButton | None, state: str, text: str, enabled: bool):
        button = button or self._get_button_widget(key)
        if not button:
            self._toggle_states[key] = state
            return
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

    def _disable_toggle_preserving_visual(self, key: str, button: QPushButton | None):
        current_state = self._toggle_states.get(key, 'red')
        current_text = button.text() if button else ''
        self._set_toggle_state(key, button, current_state, current_text, False)

    def set_toggle_visual(self, state: str, text: str, enabled: bool):
        if self._remote_master_enabled():
            self._set_toggle_state(
                'sim',
                self._get_button_widget('sim'),
                'grey',
                'Sim unavailable with remote ROS',
                False,
            )
            return
        self._set_toggle_state('sim', self._get_button_widget('sim'), state, text, enabled)

    def set_roscore_visual(self, state: str, text: str, enabled: bool):
        if self._remote_master_enabled():
            self._set_toggle_state(
                'roscore',
                self.roscore_button,
                'grey',
                'Using Remote Roscore',
                False,
            )
            return
        self._set_toggle_state('roscore', self.roscore_button, state, text, enabled)

    def set_tables_visual(self, state: str, text: str, enabled: bool):
        self._set_toggle_state('tables', self._get_button_widget('tables'), state, text, enabled)

    def set_rviz_visual(self, state: str, text: str, enabled: bool):
        self._set_toggle_state('rviz', self._get_button_widget('rviz'), state, text, enabled)

    def set_rqt_visual(self, state: str, text: str, enabled: bool):
        self._set_toggle_state('rqt', self._get_button_widget('rqt'), state, text, enabled)

    def set_script_visual(self, state: str, text: str, enabled: bool):
        self._set_toggle_state('script', self.run_script_button, state, text, enabled)

    def set_terminal_visual(self, state: str, text: str, enabled: bool):
        self._set_toggle_state('terminal', self.terminal_button, state, text, enabled)

    def set_auto_launch_visual(self, state: str, text: str, enabled: bool):
        self._set_toggle_state('auto_launch', self.auto_launch_button, state, text, enabled)

    def _guard_toggle_action(self, key: str, button: QPushButton | None = None) -> bool:
        if self._toggle_states.get(key) != 'yellow':
            return True
        text = button.text().strip().lower() if button else ''
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
        # Keep stdout/stderr moving in GUI tabs; color is also forced for
        # non-interactive tools, and PTY-sensitive tools get a wrapper above.
        color_env = (
            'export TERM="${TERM:-xterm-256color}" '
            'FORCE_COLOR=1 CLICOLOR_FORCE=1; '
        )
        return (
            'export PYTHONUNBUFFERED=1 PYTHONIOENCODING=UTF-8; '
            f'{color_env}'
            'if command -v stdbuf >/dev/null 2>&1; then '
            f'stdbuf -oL -eL {inner}; '
            'else '
            f'{inner}; '
            'fi'
        )

    def _start_program_with_pseudo_terminal(
        self,
        tab: ProcessTab,
        program: str,
        args: list[str],
    ) -> None:
        command = self._fmt_args([program] + args)
        wrapped = (
            'if command -v script >/dev/null 2>&1; then '
            f'script -qefc {self._sh_quote(command)} /dev/null; '
            'else '
            f'{command}; '
            'fi'
        )
        tab.start_shell(wrapped)

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

        if not self._confirm_workspace_mismatch_warning('Tables Demo'):
            return
        self.set_tables_visual('yellow', 'Starting Tables Demo...', False)

        def _start_tables():
            self._log_info('launching tables demo container')
            tab = self._ensure_tab('tables', 'Tables Demo', closable=False)
            exec_id = uuid.uuid4().hex
            tab.exec_id = exec_id
            tab.container_name = f'mpcmd-{exec_id[:10]}'
            self._claim_xhost(tab, 'tables', log_key=tab.key)
            inner = self._tables_demo_command()
            args = [
                'compose', 'run', '--rm', '--name', tab.container_name,
                '--label', f'mobipick.exec={exec_id}', '--label', f'mobipick.tab={tab.key}',
                *self._compose_env_args(container_name=tab.container_name),
                self._ros_tool_service(),
                'bash',
                '-lc',
                self._wrap_line_buffered(inner),
            ]
            tab.start_program('docker', args)
            self._schedule_host_to_container_copy(tab)
            self.set_tables_visual('green', 'Stop Tables Demo', True)
            self._focus_tab('tables')

        self._ensure_roscore_ready(_start_tables)

    def stop_tables_demo(self):
        tab = self.tasks['tables']
        if not tab.is_running():
            self.set_tables_visual('red', 'Run Tables Demo', True)
            return
        self.set_tables_visual('yellow', 'Stopping Tables Demo...', False)
        def _on_stopped():
            self._release_xhost(tab, log_key=tab.key)
            self.set_tables_visual('red', 'Run Tables Demo', True)

        self._stop_custom_tab(tab, on_stopped=_on_stopped)

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

        if not self._confirm_workspace_mismatch_warning('RViz'):
            return
        self.set_rviz_visual('yellow', 'Starting RViz...', False)

        def _start_rviz():
            self._log_info('starting RViz viewer')
            tab = self._ensure_tab('rviz', 'RViz', closable=False)
            exec_id = uuid.uuid4().hex
            tab.exec_id = exec_id
            tab.container_name = f'mpcmd-{exec_id[:10]}'
            self._claim_xhost(tab, 'rviz', log_key=tab.key)
            rviz_cmd = self._rviz_command()
            args = [
                'compose', 'run', '--rm', '--name', tab.container_name,
                '--label', f'mobipick.exec={exec_id}', '--label', f'mobipick.tab={tab.key}',
                *self._compose_env_args(container_name=tab.container_name),
                self._ros_tool_service(),
                'bash',
                '-lc',
                self._wrap_line_buffered(rviz_cmd),
            ]
            tab.start_program('docker', args)
            self._schedule_host_to_container_copy(tab)
            self.set_rviz_visual('green', 'Stop RViz', True)
            self._focus_tab('rviz')

        self._ensure_roscore_ready(_start_rviz)

    def stop_rviz(self):
        tab = self.tasks['rviz']
        if not tab.is_running():
            self.set_rviz_visual('red', 'Start RViz', True)
            return
        self.set_rviz_visual('yellow', 'Stopping RViz...', False)
        def _on_stopped():
            self._release_xhost(tab, log_key=tab.key)
            self.set_rviz_visual('red', 'Start RViz', True)

        self._stop_custom_tab(tab, on_stopped=_on_stopped)

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

        if not self._confirm_workspace_mismatch_warning('RQt Tables'):
            return
        self.set_rqt_visual('yellow', 'Starting RQt Tables...', False)

        def _start_rqt():
            world = self.world_combo.currentText().strip() or 'moelk_tables'
            self._log_info(f'starting rqt tables demo for {world}')
            tab = self._ensure_tab('rqt', 'RQt Tables', closable=False)
            exec_id = uuid.uuid4().hex
            tab.exec_id = exec_id
            tab.container_name = f'mpcmd-{exec_id[:10]}'
            self._claim_xhost(tab, 'rqt', log_key=tab.key)
            cmd = self._rqt_tables_command()
            args = [
                'compose', 'run', '--rm', '--name', tab.container_name,
                '--label', f'mobipick.exec={exec_id}', '--label', f'mobipick.tab={tab.key}',
                *self._compose_env_args(container_name=tab.container_name),
                self._ros_tool_service(),
                'bash',
                '-lc',
                self._wrap_line_buffered(cmd),
            ]
            tab.start_program('docker', args)
            self._schedule_host_to_container_copy(tab)
            self.set_rqt_visual('green', 'Stop RQt Tables', True)
            self._focus_tab('rqt')

        self._ensure_roscore_ready(_start_rqt)

    def stop_rqt_tables_demo(self):
        tab = self.tasks['rqt']
        if not tab.is_running():
            self.set_rqt_visual('red', 'Start RQt Tables', True)
            return
        self.set_rqt_visual('yellow', 'Stopping RQt Tables...', False)
        def _on_stopped():
            self._release_xhost(tab, log_key=tab.key)
            self.set_rqt_visual('red', 'Start RQt Tables', True)

        self._stop_custom_tab(tab, on_stopped=_on_stopped)

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
        if not self._confirm_workspace_mismatch_warning('Terminal'):
            return
        self.set_terminal_visual('yellow', 'Starting Terminal...', False)

        def _start_terminal():
            self._ensure_network(log_key='log')
            exec_id = uuid.uuid4().hex
            container_name = f"{self._terminal_container_prefix}-{exec_id[:10]}"

            self._grant_x('terminal', log_key='log')

            env_overrides = self._terminal_env_overrides()

            command_parts = [
                'docker', 'compose', 'run', '--rm', '--name', container_name,
                '--label', f'mobipick.exec={exec_id}',
                '--label', 'mobipick.role=terminal',
                '--label', 'mobipick.tab=terminal',
                '--user', 'root',
            ]
            env_overrides = dict(env_overrides)
            command_parts.extend(self._compose_env_args(env_overrides, container_name=container_name))
            command_parts.extend(
                [
                    self._ros_tool_service(),
                    'python3',
                    f'{CONTAINER_SCRIPTS_DIR}/enter_host_shell.py',
                    'bash',
                    '--rcfile',
                    f'{CONTAINER_SCRIPTS_DIR}/terminal.bashrc',
                    '-i',
                ]
            )
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
            proc.setProcessEnvironment(self._build_process_environment(env_overrides))
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

        self._ensure_roscore_ready(_start_terminal, allow_autostart=False)

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
        self._revoke_x('terminal', log_key='log')
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
        quoted = self._sh_quote(container_name)
        script = (
            f'until docker container inspect {quoted} >/dev/null 2>&1; do '
            'sleep 0.2; '
            'done; '
            f'docker logs -f --tail 0 {quoted}'
        )
        tab.start_program('bash', ['-lc', script])
        self._focus_tab(key)
        self._terminal_stream_tab_key = key

    def run_custom_command(self):
        text = self.command_input.text().strip()
        if not text:
            return
        if not self._confirm_workspace_mismatch_warning('Custom command'):
            return

        def _run_command():
            self._log_info(f'running custom command: {text}')

            key_target = self._select_custom_tab_key()

            tab = self.tasks[key_target]
            exec_id = uuid.uuid4().hex
            tab.exec_id = exec_id
            tab.container_name = f'mpcmd-{exec_id[:10]}'
            self._claim_xhost(tab, key_target, log_key=tab.key)
            wrapped = self._wrap_line_buffered(text)
            args = [
                'compose', 'run', '--rm', '--name', tab.container_name,
                '--label', f'mobipick.exec={exec_id}', '--label', f'mobipick.tab={key_target}',
                *self._compose_env_args(container_name=tab.container_name),
                self._ros_tool_service(), 'bash', '-lc', wrapped
            ]
            tab.start_program('docker', args)
            self._schedule_host_to_container_copy(tab)
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
        stop_command: str | None = None,
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
            self._release_xhost(tab, log_key=tab.key)
            if on_stopped:
                on_stopped()
            self._update_stop_custom_enabled()

        def _container_sigint_then_stop():
            if stop_command:
                try:
                    self._sp_run(['bash', '-lc', stop_command], log_key=tab.key, check=False)
                except Exception as exc:
                    self._append_gui_html(tab.key, f'<i>Failed to run stop command: {html.escape(str(exc))}</i>')

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

    def commit_current_tab(self):
        self._log_button_click(self.commit_current_tab_button, 'Commit Current Tab')

        key = self._current_tab_key()
        if not key:
            QMessageBox.information(self, 'Commit Current Tab', 'Select a tab before committing its container.')
            return

        tab = self.tasks.get(key)
        if not tab:
            QMessageBox.information(self, 'Commit Current Tab', 'The selected tab is unavailable.')
            return

        container_name = getattr(tab, 'container_name', None)
        if not container_name:
            QMessageBox.information(self, 'Commit Current Tab', 'The current tab is not associated with a running container.')
            return

        image_ref = (self._selected_image or '').strip()
        if not image_ref:
            QMessageBox.information(self, 'Commit Current Tab', 'Choose a target image from the image list before committing.')
            return

        container_ref = container_name
        try:
            inspect_cp = self._sp_run(
                ['docker', 'container', 'inspect', '--format', '{{.Id}}', container_name],
                log_key='log',
                log_stdout=False,
                text=True,
            )
        except Exception as exc:  # pragma: no cover - defensive logging
            self._console_log(1, f'Failed to inspect container {container_name}: {exc}')
        else:
            if inspect_cp.returncode == 0:
                container_id = (inspect_cp.stdout or '').strip()
                if container_id:
                    if container_id.startswith('sha256:'):
                        container_id = container_id.split(':', 1)[1]
                    container_ref = container_id[:64] or container_ref

        base_image, base_tag = self._split_image_ref(image_ref)
        if not base_image:
            QMessageBox.warning(
                self,
                'Commit Current Tab',
                'Unable to determine the repository for the selected image.',
            )
            return

        display_base_tag = base_tag or 'latest'
        timestamp = datetime.now().strftime('%d-%m-%Y-%H-%M-%S')
        timestamp_tag = f'{display_base_tag}--{timestamp}'

        dialog = QDialog(self)
        dialog.setWindowTitle('Commit Current Tab')

        layout = QVBoxLayout(dialog)
        intro = QLabel('Choose how you want to create the new image tag:')
        layout.addWidget(intro)

        overwrite_row = QHBoxLayout()
        overwrite_label = QLabel(f'{base_image}:{display_base_tag}')
        overwrite_row.addWidget(overwrite_label)
        overwrite_button = QPushButton('Overwrite existing tag')
        overwrite_row.addWidget(overwrite_button)
        layout.addLayout(overwrite_row)

        timestamp_row = QHBoxLayout()
        timestamp_label = QLabel(f'{base_image}:{timestamp_tag}')
        timestamp_row.addWidget(timestamp_label)
        timestamp_button = QPushButton('Create snapshot (timestamp)')
        timestamp_row.addWidget(timestamp_button)
        layout.addLayout(timestamp_row)

        custom_row = QHBoxLayout()
        custom_image_edit = QLineEdit(base_image)
        custom_tag_prefill = f'{display_base_tag}-' if display_base_tag else ''
        custom_tag_edit = QLineEdit(custom_tag_prefill)
        custom_row.addWidget(custom_image_edit)
        colon_label = QLabel(':')
        colon_label.setAlignment(Qt.AlignCenter)
        custom_row.addWidget(colon_label)
        custom_row.addWidget(custom_tag_edit)
        custom_button = QPushButton('Create snapshot (custom tag)')
        custom_row.addWidget(custom_button)
        layout.addLayout(custom_row)

        buttons_row = QHBoxLayout()
        buttons_row.addStretch(1)
        show_mounts_button = QPushButton('Show Mounts')
        buttons_row.addWidget(show_mounts_button)
        cancel_button = QPushButton('Cancel')
        buttons_row.addWidget(cancel_button)
        layout.addLayout(buttons_row)

        def _run_commit(target_repo: str, target_tag: str):
            repo = (target_repo or '').strip()
            tag = (target_tag or '').strip()
            if not repo:
                QMessageBox.warning(dialog, 'Commit Current Tab', 'Image name is required to commit the container.')
                return
            if not tag:
                QMessageBox.warning(dialog, 'Commit Current Tab', 'Tag name is required to commit the container.')
                return
            target_ref = f'{repo}:{tag}'
            commit_cp = self._sp_run(
                ['docker', 'commit', container_ref, target_ref],
                log_key=key,
                text=True,
            )

            if commit_cp.returncode == 0:
                message = f'Committed container {container_name} to image {target_ref}.'
                self._append_gui_html(key, html.escape(message))
                dialog.accept()
            else:
                stderr_text = self._decode_output(getattr(commit_cp, 'stderr', '')).strip()
                if stderr_text:
                    self._append_gui_html(key, html.escape(stderr_text))
                QMessageBox.warning(dialog, 'Commit Current Tab', 'Failed to commit the current tab container. Check the tab log for details.')

        def _show_mounts_dialog():
            mounts_dialog = QDialog(dialog)
            mounts_dialog.setWindowTitle('Container Mounts')
            mounts_layout = QVBoxLayout(mounts_dialog)
            mounts_text = QTextEdit(mounts_dialog)
            mounts_text.setReadOnly(True)

            mounts_output = 'No mount information available.'
            try:
                mounts_cp = self._sp_run(
                    [
                        'docker',
                        'container',
                        'inspect',
                        '--format',
                        '{{json .Mounts}}',
                        container_ref,
                    ],
                    log_stdout=False,
                    log_stderr=False,
                    text=True,
                )
            except Exception as exc:  # pragma: no cover - defensive logging
                mounts_output = f'Failed to inspect container mounts: {exc}'
            else:
                if mounts_cp.returncode == 0:
                    data = (mounts_cp.stdout or '').strip()
                    if data:
                        try:
                            mounts = json.loads(data)
                        except json.JSONDecodeError:
                            mounts_output = data
                        else:
                            if isinstance(mounts, list) and mounts:
                                lines = []
                                for mount in mounts:
                                    if isinstance(mount, dict):
                                        source = mount.get('Source', '')
                                        destination = mount.get('Destination', '')
                                        mode = mount.get('Mode', '')
                                        rw = mount.get('RW', '')
                                        details = [
                                            value
                                            for value in (
                                                f'Source: {source}' if source else '',
                                                f'Destination: {destination}' if destination else '',
                                                f'Mode: {mode}' if mode else '',
                                                f'Read/Write: {rw}' if rw != '' else '',
                                            )
                                            if value
                                        ]
                                        if details:
                                            lines.append('\n'.join(details))
                                mounts_output = '\n\n'.join(lines) or mounts_output
                            else:
                                mounts_output = 'No mount information available.'
                else:
                    stderr_text = self._decode_output(getattr(mounts_cp, 'stderr', '')).strip()
                    mounts_output = stderr_text or mounts_output

            mounts_text.setPlainText(mounts_output)
            mounts_layout.addWidget(mounts_text)

            ok_button = QPushButton('OK')
            ok_button.clicked.connect(mounts_dialog.accept)
            ok_row = QHBoxLayout()
            ok_row.addStretch(1)
            ok_row.addWidget(ok_button)
            mounts_layout.addLayout(ok_row)

            mounts_dialog.exec_()

        overwrite_button.clicked.connect(lambda: _run_commit(base_image, display_base_tag))
        timestamp_button.clicked.connect(lambda: _run_commit(base_image, timestamp_tag))
        custom_button.clicked.connect(lambda: _run_commit(custom_image_edit.text(), custom_tag_edit.text()))
        cancel_button.clicked.connect(dialog.reject)
        show_mounts_button.clicked.connect(_show_mounts_dialog)

        dialog.exec_()

    def manage_images(self):
        self._log_button_click(self.manage_images_button, 'Manage Images')

        dialog = QDialog(self)
        dialog.setWindowTitle('Manage Images')
        dialog.setModal(True)

        layout = QVBoxLayout(dialog)
        intro_label = QLabel(
            'Select the images you would like to remove and click Apply. '
            'Only images that match the configured discovery filters are shown.'
        )
        intro_label.setWordWrap(True)
        layout.addWidget(intro_label)

        status_label = QLabel()
        status_label.setWordWrap(True)
        status_label.setStyleSheet('color: #ff6b6b;')
        status_label.hide()
        layout.addWidget(status_label)

        scroll_area = QScrollArea(dialog)
        scroll_area.setWidgetResizable(True)
        layout.addWidget(scroll_area, 1)

        list_container = QWidget()
        scroll_area.setWidget(list_container)
        list_layout = QVBoxLayout(list_container)
        list_layout.setContentsMargins(0, 0, 0, 0)
        list_layout.setSpacing(6)

        force_checkbox = QCheckBox('Force removal (-f)')
        layout.addWidget(force_checkbox)

        button_row = QHBoxLayout()
        button_row.addStretch(1)
        apply_button = QPushButton('Apply')
        close_button = QPushButton('Close')
        button_row.addWidget(apply_button)
        button_row.addWidget(close_button)
        layout.addLayout(button_row)

        detail_level = self._manage_images_detail_level()
        entries: list[dict[str, object]] = []

        def _format_entry_text(record: dict[str, str]) -> str:
            repo = record.get('Repository', '') or ''
            tag = record.get('Tag', '') or ''
            if not repo and not tag:
                repo, tag = self._split_image_ref(record.get('ref', ''))
            repo_html = html.escape(repo) if repo else ''
            tag_html = html.escape(tag) if tag else ''
            if tag_html:
                tag_html = f'<span style="font-weight:600;">{tag_html}</span>'
            if repo_html and tag_html:
                ref_html = f'{repo_html}:{tag_html}'
            elif tag_html:
                ref_html = tag_html
            elif repo_html:
                ref_html = repo_html
            else:
                ref_html = html.escape(record.get('ref', '') or '')
            if not ref_html:
                ref_html = 'Unnamed image'

            if detail_level == 'simple':
                return ref_html

            parts: list[str] = [ref_html]
            created = record.get('CreatedSince', '') or record.get('CreatedAt', '')
            if created:
                parts.append(f'Created {html.escape(created)}')

            if detail_level == 'medium':
                return ' | '.join(parts)

            identifier = record.get('identifier', '')
            if identifier:
                short_id = identifier.split(':', 1)[-1]
                if short_id:
                    parts.append(f'ID {html.escape(short_id[:12])}')
            size = record.get('Size', '')
            if size:
                parts.append(f'Size {html.escape(size)}')

            return ' | '.join(parts)

        def _clear_list_layout():
            while list_layout.count():
                item = list_layout.takeAt(0)
                widget = item.widget()
                if widget is not None:
                    widget.deleteLater()

        def _populate_images():
            nonlocal entries
            entries = []
            _clear_list_layout()
            records, error_message = self._discover_filtered_image_records()
            if error_message:
                status_label.setText(error_message)
                status_label.show()
            else:
                status_label.hide()
                status_label.clear()
            if not records:
                placeholder = QLabel('No images match the configured filters.')
                placeholder.setWordWrap(True)
                list_layout.addWidget(placeholder)
                list_layout.addStretch(1)
                return
            for record in records:
                checkbox = QCheckBox()
                checkbox.setChecked(False)

                row_widget = QWidget()
                row_layout = QHBoxLayout(row_widget)
                row_layout.setContentsMargins(0, 0, 0, 0)
                row_layout.setSpacing(8)

                row_layout.addWidget(checkbox, 0, Qt.AlignTop)

                label = QLabel(_format_entry_text(record))
                label.setTextFormat(Qt.RichText)
                label.setWordWrap(True)
                label.setTextInteractionFlags(Qt.TextSelectableByMouse)

                original_mouse_release = label.mouseReleaseEvent

                def _label_mouse_release(event, *, cb=checkbox, original=original_mouse_release, lbl=label):
                    if event.button() == Qt.LeftButton:
                        cb.toggle()
                    if original is not None:
                        original(event)
                    else:
                        QLabel.mouseReleaseEvent(lbl, event)

                label.mouseReleaseEvent = _label_mouse_release  # type: ignore[method-assign]

                row_layout.addWidget(label, 1)
                list_layout.addWidget(row_widget)

                entries.append(
                    {
                        'checkbox': checkbox,
                        'target': record.get('identifier') or record.get('ref', ''),
                        'ref': record.get('ref', ''),
                    }
                )
            list_layout.addStretch(1)

        def _apply_removal():
            selected_targets: list[str] = []
            selected_refs: list[str] = []
            for entry in entries:
                checkbox = entry.get('checkbox')
                if not isinstance(checkbox, QCheckBox) or not checkbox.isChecked():
                    continue
                target = str(entry.get('target') or '').strip()
                if not target:
                    continue
                selected_targets.append(target)
                ref = str(entry.get('ref') or target)
                selected_refs.append(ref)
            if not selected_targets:
                QMessageBox.information(dialog, 'Manage Images', 'Select at least one image to remove.')
                return

            args = ['docker', 'rmi']
            if force_checkbox.isChecked():
                args.append('-f')
            args.extend(selected_targets)

            try:
                cp = self._sp_run(args, log_key='log', text=True)
            except Exception as exc:  # pragma: no cover - defensive
                QMessageBox.warning(dialog, 'Manage Images', f'Failed to remove images: {exc}')
                return

            if cp.returncode not in (0, None):
                QMessageBox.warning(
                    dialog,
                    'Manage Images',
                    'Failed to remove one or more images. Check the Log tab for details.',
                )
            else:
                removed = ', '.join(selected_refs)
                if removed:
                    self._log_info(f'Removed images: {removed}')
                QMessageBox.information(dialog, 'Manage Images', 'Selected images were removed.')

            self._load_available_images()
            _populate_images()

        _populate_images()

        apply_button.clicked.connect(_apply_removal)
        close_button.clicked.connect(dialog.reject)

        dialog.exec_()

    def execute_docker_cp_from_container(self):
        self._log_button_click(self.execute_docker_cp_button, 'Execute Docker cp')

        key = self._current_tab_key()
        if not key:
            QMessageBox.information(self, 'Execute Docker cp', 'Select a tab before copying from its container.')
            return

        tab = self.tasks.get(key)
        if not tab:
            QMessageBox.information(self, 'Execute Docker cp', 'The selected tab is unavailable.')
            return

        container_name = getattr(tab, 'container_name', None)
        if not container_name:
            QMessageBox.information(self, 'Execute Docker cp', 'The current tab is not associated with a running container.')
            return

        entries = self._docker_cp_entries('container_to_host')
        if not entries:
            QMessageBox.information(self, 'Execute Docker cp', 'No docker cp paths configured for the selected image.')
            return

        container_ref = self._container_reference_for_tab(tab)
        if not container_ref:
            QMessageBox.warning(self, 'Execute Docker cp', 'Unable to determine the running container for the current tab.')
            return

        commands = self._build_container_to_host_commands(container_ref, entries, key)
        if not commands:
            self._append_gui_html(key, '<i>No docker cp commands to execute for the current configuration.</i>')
            return

        self._append_gui_html(key, '<i>Copying configured container paths to the host...</i>')
        self._run_command_sequence(commands, log_key=key)

    def _open_docker_cp_config_dialog(self):
        save_path = self._workspace_docker_cp_config_path()
        dialog = DockerCpConfigDialog(
            self._docker_cp_config,
            load_docker_cp_user_config(save_path),
            self._workspace_registry.active,
            self._docker_cp_workspace_names(),
            save_path,
            self._docker_cp_setup_container_options,
            self._docker_cp_container_path_from_setup,
            self._docker_cp_host_start_path,
            self,
        )
        if dialog.exec_() != QDialog.Accepted:
            return
        try:
            saved_path = save_docker_cp_config(
                dialog.docker_cp_config(),
                save_path,
            )
        except Exception as exc:
            QMessageBox.warning(
                self,
                'Docker cp Paths',
                f'Failed to save docker cp paths:\n{exc}',
            )
            return
        self._docker_cp_config = load_docker_cp_config(save_path)
        self._synced_container_refs.clear()
        self._log_info(f'docker cp paths saved to {saved_path}')
        QMessageBox.information(
            self,
            'Docker cp Paths',
            f'Docker cp paths saved to:\n{saved_path}',
        )

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

    def _collect_log_entries(self) -> list[tuple[str, str]]:
        entries: list[tuple[str, str]] = []
        for i in range(self.tabs.count()):
            widget = self.tabs.widget(i)
            if not isinstance(widget, QTextEdit):
                continue
            html_content = self._extract_widget_html(widget)
            if html_content is None:
                continue
            label = self.tabs.tabText(i).strip() or f'tab{i + 1}'
            entries.append((label, html_content))
        return entries

    def _save_logs_to_directory(self, directory: Path, entries: list[tuple[str, str]] | None = None) -> int:
        entries = entries if entries is not None else self._collect_log_entries()
        if not entries:
            return 0
        try:
            directory.mkdir(parents=True, exist_ok=True)
        except Exception as exc:
            self._append_gui_html(
                'log',
                f'<i>Failed to prepare log directory {html.escape(str(directory))}: {html.escape(str(exc))}</i>',
            )
            return 0
        count = 0
        for label, html_content in entries:
            filename = self._suggest_log_filename(label)
            path = self._resolve_unique_path(str(directory), filename)
            if self._write_html_file(path, html_content):
                count += 1
        return count

    def save_all_logs(self):
        entries = self._collect_log_entries()
        if not entries:
            QMessageBox.information(self, 'Save Logs', 'There are no logs to save.')
            return
        directory = QFileDialog.getExistingDirectory(self, 'Select folder to save logs')
        if not directory:
            return
        saved = self._save_logs_to_directory(Path(directory), entries=entries)
        if saved:
            QMessageBox.information(self, 'Save Logs', f'Saved {saved} log file(s).')

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

    def _filter_terminal_escapes(self, data: str) -> str:
        if '\x1b' not in data:
            return data
        sanitized = OSC_SEQ_RE.sub('', data)
        sanitized = sanitized.replace('\x07', '')

        def _keep_sgr(match: Match[str]) -> str:
            seq = match.group(0)
            return seq if seq.endswith('m') else ''

        sanitized = CSI_SEQ_RE.sub(_keep_sgr, sanitized)
        sanitized = sanitized.replace('\x1b7', '').replace('\x1b8', '')
        if '\b' in sanitized:
            out_chars: list[str] = []
            for ch in sanitized:
                if ch == '\b':
                    if out_chars:
                        out_chars.pop()
                else:
                    out_chars.append(ch)
            sanitized = ''.join(out_chars)
        return sanitized

    def _collapse_carriage_returns(self, text: str) -> str:
        if '\r' not in text:
            return text
        text = text.replace('\r\n', '\n')
        if '\r' not in text:
            return text
        out_chars: list[str] = []
        for ch in text:
            if ch == '\r':
                while out_chars and out_chars[-1] != '\n':
                    out_chars.pop()
            else:
                out_chars.append(ch)
        return ''.join(out_chars)

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
        tab = self.tasks.get(key)
        if tab:
            self._release_xhost(tab, log_key=key)
            self._append_gui_html(key, f'<i>Process finished with code {exit_code} [{status_name}]</i>')
            tab.container_name = None
            tab.exec_id = None
        if key == 'roscore':
            self._roscore_running_cached = False
            if self._roscore_stopping:
                self._cancel_auto_launch_timers()
                self._cancel_recording_schedule()
                if self._recording_is_active():
                    self._stop_screen_recording(save_logs=True, reason='Roscore stopped; ending recording')
                self._finalize_auto_launch_stop()
                return
            self._roscore_last_start_ts = None
            self.set_roscore_visual('red', 'Start Roscore', enabled=True)
            self._sim_running_cached = False
            self.set_toggle_visual('red', 'Start Sim', enabled=True)
            self.set_tables_visual('red', 'Run Tables Demo', True)
            self.set_rviz_visual('red', 'Start RViz', True)
            self.set_rqt_visual('red', 'Start RQt Tables', True)
            self.stop_terminal()
            self._cancel_recording_schedule()
            if self._recording_is_active():
                self._stop_screen_recording(save_logs=True, reason='Roscore exited; stopping recording')
            self._script_active_tab_key = None
            self.set_script_visual('red', 'Run Script', bool(self._script_choices))
            self._update_stop_custom_enabled()
            return
        if key == 'sim':
            self._sim_running_cached = False
            if self._killing:
                return
            self.set_toggle_visual('red', 'Start Sim', enabled=True)
            return
        if key in self._config_buttons:
            cfg = self._config_buttons[key]
            label = self._config_label(cfg)
            self._set_config_visual(cfg, 'red', f'Start {label}', True)
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
        if self._window_layout_auto_apply and self._window_layout_manager:
            self._window_layout_manager.maybe_apply_saved_layout()

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
        self._save_window_state()
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
        self._cancel_auto_launch_timers()
        self._cancel_recording_schedule()
        self._stop_screen_recording(save_logs=False, reason='Exit requested; stopping recording')
        self._auto_launch_running = False
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
        self.set_auto_launch_visual('red', self._auto_launch_start_text(), True)
        for key in self._config_button_order:
            cfg = self._config_buttons.get(key, {})
            label = self._config_label(cfg)
            self._set_config_visual(cfg, 'red', f'Start {label}', True)
        self.stop_custom_button.setEnabled(False)
        self._update_stop_custom_enabled()

    # ---------- Close ----------

    def _restore_window_state(self, window_cfg: dict) -> None:
        geometry = window_cfg.get('geometry', [])
        if len(geometry) == 4:
            try:
                self.setGeometry(*[int(value) for value in geometry])
            except (TypeError, ValueError):
                pass
        if window_cfg.get('maximized'):
            self.setWindowState(self.windowState() | Qt.WindowMaximized)

    def _save_window_state(self) -> None:
        geometry = self.normalGeometry()
        if geometry.isNull():
            geometry = self.geometry()
        updates = {
            'window': {
                'geometry': [
                    geometry.x(),
                    geometry.y(),
                    geometry.width(),
                    geometry.height(),
                ],
                'maximized': self.isMaximized(),
            },
        }
        try:
            save_user_config_update(updates)
        except Exception as exc:
            self._console_log(1, f'Warning: failed to save window state: {exc}')

    def closeEvent(self, event):
        if self._exit_in_progress:
            event.ignore()
            return
        event.ignore()
        self._begin_exit_sequence()
