"""Profile-wide editor for launch argument selectors."""

from __future__ import annotations

import copy

from PyQt5.QtWidgets import (
    QAbstractItemView,
    QCheckBox,
    QComboBox,
    QDialog,
    QDialogButtonBox,
    QFormLayout,
    QHBoxLayout,
    QHeaderView,
    QLabel,
    QLineEdit,
    QMessageBox,
    QPlainTextEdit,
    QPushButton,
    QScrollArea,
    QSizePolicy,
    QTableWidget,
    QTableWidgetItem,
    QVBoxLayout,
    QWidget,
)

from .config import (
    DEFAULT_ADVANCED_ARG_NAMES,
    DEFAULT_ARG_SCOPE,
    GENERIC_BUTTON_ARG_SLOTS,
    normalize_arg_scope,
    profile_arg_scope,
)
from . import popups


ARG_FIELDS = (
    'name', 'options', 'applies', 'advanced', 'scope', 'description',
    'option_descriptions',
)
# scope value -> label of the editor's Scope combo and the Location column
SCOPE_LABELS = {
    'shared': 'Shared',
    'real': 'Real robot only',
    'sim': 'Sim only',
}


def profile_arguments(entries: list[dict]) -> list[dict]:
    """Collect each globally shared argument slot from all profile buttons."""
    arguments = []
    for slot in GENERIC_BUTTON_ARG_SLOTS:
        defining = next(
            (entry for entry in entries if entry.get(f'arg_{slot}_name')),
            None,
        )
        if defining is None:
            continue
        name = str(defining[f'arg_{slot}_name']).strip()
        options = next((
            list(entry.get(f'arg_{slot}_options') or [])
            for entry in entries if entry.get(f'arg_{slot}_options')
        ), [])
        description = next((
            str(entry.get(f'arg_{slot}_description') or '')
            for entry in entries if entry.get(f'arg_{slot}_description')
        ), '')
        option_descriptions = next((
            dict(entry.get(f'arg_{slot}_option_descriptions') or {})
            for entry in entries
            if entry.get(f'arg_{slot}_option_descriptions')
        ), {})
        placement = next(
            (entry[f'arg_{slot}_advanced'] for entry in entries
             if entry.get(f'arg_{slot}_name')
             and entry.get(f'arg_{slot}_advanced') is not None),
            None,
        )
        arguments.append({
            'slot': slot,
            'name': name,
            'options': options,
            'description': description,
            'option_descriptions': option_descriptions,
            'advanced': (
                bool(placement) if placement is not None
                else name in DEFAULT_ADVANCED_ARG_NAMES
            ),
            # None = unset: counts as shared and is not written to the profile
            'scope': profile_arg_scope(entries, slot),
            'buttons': {
                str(entry.get('key') or '') for entry in entries
                if entry.get(f'arg_{slot}_applies')
            },
        })
    return arguments


class LaunchArgumentDetailsDialog(QDialog):
    """Edit one argument and its per-button applicability."""

    def __init__(self, argument: dict, buttons: list[tuple[str, str]],
                 parent: QWidget | None = None):
        super().__init__(parent)
        self.setWindowTitle('Launch Argument')
        self.resize(720, 620)
        self.setMinimumWidth(620)
        self._argument = copy.deepcopy(argument)
        root = QVBoxLayout(self)
        form = QFormLayout()
        self.name_input = QLineEdit(argument.get('name', ''))
        self.name_input.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Preferred)
        form.addRow('ROS argument name:', self.name_input)
        self.options_input = QLineEdit(', '.join(argument.get('options', [])))
        self.options_input.setPlaceholderText('true, false')
        self.options_input.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Preferred)
        form.addRow('Selectable values:', self.options_input)
        self.description_input = QPlainTextEdit(argument.get('description', ''))
        self.description_input.setPlaceholderText(
            'Explain what changing this launch argument does.'
        )
        self.description_input.setMaximumHeight(105)
        form.addRow('Description:', self.description_input)
        value_lines = [
            f'{value} | {description}'
            for value, description in (
                argument.get('option_descriptions') or {}
            ).items()
        ]
        self.value_descriptions_input = QPlainTextEdit('\n'.join(value_lines))
        self.value_descriptions_input.setPlaceholderText(
            'One per line: true | Enable this feature'
        )
        self.value_descriptions_input.setMaximumHeight(120)
        form.addRow('Value descriptions:', self.value_descriptions_input)
        self.advanced_check = QCheckBox('Show in Advanced Launch Options')
        self.advanced_check.setChecked(bool(argument.get('advanced')))
        form.addRow('Placement:', self.advanced_check)
        self.scope_combo = QComboBox()
        for scope, label in SCOPE_LABELS.items():
            self.scope_combo.addItem(label, scope)
        self.scope_combo.setCurrentIndex(self.scope_combo.findData(
            normalize_arg_scope(argument.get('scope'))
        ))
        self.scope_combo.setToolTip(
            'Sim only: never passed to a launch on the real robot (remote '
            'ROS master), greyed out there.\nReal robot only: never passed '
            'to a launch in the simulation, greyed out there.\nShared: passed '
            'everywhere. The scope also picks the tab in Advanced Launch '
            'Options.'
        )
        form.addRow('Scope:', self.scope_combo)
        root.addLayout(form)

        root.addWidget(QLabel('Apply this argument to:'))
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        button_widget = QWidget()
        button_layout = QVBoxLayout(button_widget)
        self.button_checks: dict[str, QCheckBox] = {}
        for key, label in buttons:
            check = QCheckBox(f'{label} ({key})')
            check.setChecked(key in argument.get('buttons', set()))
            button_layout.addWidget(check)
            self.button_checks[key] = check
        button_layout.addStretch()
        scroll.setWidget(button_widget)
        root.addWidget(scroll, 1)
        buttons_box = QDialogButtonBox(
            QDialogButtonBox.Save | QDialogButtonBox.Cancel
        )
        buttons_box.accepted.connect(self.accept)
        buttons_box.rejected.connect(self.reject)
        root.addWidget(buttons_box)

    def argument(self) -> dict:
        """Return validated input for the selected argument slot."""
        return copy.deepcopy(self._argument)

    def accept(self) -> None:
        name = self.name_input.text().strip()
        options = list(dict.fromkeys(
            value.strip() for value in self.options_input.text().split(',')
            if value.strip()
        ))
        if not name or not options:
            popups.warning(
                self, 'Launch Argument',
                'Enter an argument name and at least one selectable value.',
            )
            return
        descriptions = {}
        for line in self.value_descriptions_input.toPlainText().splitlines():
            if not line.strip():
                continue
            value, separator, description = line.partition('|')
            value = value.strip()
            description = description.strip()
            if not separator or value not in options or not description:
                popups.warning(
                    self, 'Launch Argument',
                    f'Invalid value description: {line}\n'
                    'Use "value | description" with a listed value.',
                )
                return
            descriptions[value] = description
        self._argument.update({
            'name': name,
            'options': options,
            'description': self.description_input.toPlainText().strip(),
            'option_descriptions': descriptions,
            'advanced': self.advanced_check.isChecked(),
            'scope': self._chosen_scope(),
            'buttons': {
                key for key, check in self.button_checks.items()
                if check.isChecked()
            },
        })
        super().accept()

    def _chosen_scope(self) -> str | None:
        """The chosen scope; None keeps an unset scope unset when Shared stays chosen."""
        scope = normalize_arg_scope(self.scope_combo.currentData())
        if scope == DEFAULT_ARG_SCOPE and self._argument.get('scope') is None:
            return None
        return scope


def _location_text(argument: dict) -> str:
    """Placement and, when not shared, scope of an argument for the table."""
    location = 'Advanced' if argument['advanced'] else 'Main window'
    scope = normalize_arg_scope(argument.get('scope'))
    if scope != DEFAULT_ARG_SCOPE:
        location += f', {SCOPE_LABELS[scope]}'
    return location


class LaunchArgumentsDialog(QDialog):
    """Show and edit all launch arguments across the loaded button profile."""

    def __init__(self, entries: list[dict], parent: QWidget | None = None):
        super().__init__(parent)
        self.setWindowTitle('Edit Launch Options')
        self.resize(900, 560)
        self.setMinimumWidth(700)
        self._entries = copy.deepcopy(entries)
        self._arguments = profile_arguments(self._entries)
        self._buttons = [
            (str(entry['key']), str(entry.get('label') or entry['key']))
            for entry in self._entries if entry.get('key')
        ]
        root = QVBoxLayout(self)
        note = QLabel(
            'Edit every profile argument here. The main window can show at '
            'most six argument selectors alongside world selection.'
        )
        note.setWordWrap(True)
        root.addWidget(note)
        self.table = QTableWidget(0, 4)
        self.table.setHorizontalHeaderLabels(
            ['Slot', 'Argument', 'Location', 'Applies to']
        )
        self.table.setSelectionBehavior(QAbstractItemView.SelectRows)
        self.table.setSelectionMode(QAbstractItemView.SingleSelection)
        self.table.setEditTriggers(QAbstractItemView.NoEditTriggers)
        header = self.table.horizontalHeader()
        header.setSectionResizeMode(0, QHeaderView.ResizeToContents)
        header.setSectionResizeMode(1, QHeaderView.Stretch)
        header.setSectionResizeMode(2, QHeaderView.ResizeToContents)
        header.setSectionResizeMode(3, QHeaderView.Stretch)
        self.table.cellDoubleClicked.connect(
            lambda _row, _column: self._edit_selected()
        )
        root.addWidget(self.table, 1)
        actions = QHBoxLayout()
        add_button = QPushButton('Add Argument')
        add_button.clicked.connect(self._add_argument)
        actions.addWidget(add_button)
        edit_button = QPushButton('Edit Selected...')
        edit_button.clicked.connect(self._edit_selected)
        actions.addWidget(edit_button)
        remove_button = QPushButton('Remove Selected')
        remove_button.clicked.connect(self._remove_selected)
        actions.addWidget(remove_button)
        actions.addStretch()
        root.addLayout(actions)
        buttons_box = QDialogButtonBox(
            QDialogButtonBox.Save | QDialogButtonBox.Cancel
        )
        buttons_box.accepted.connect(self.accept)
        buttons_box.rejected.connect(self.reject)
        root.addWidget(buttons_box)
        self._refresh_table()

    def _refresh_table(self, selected_slot: int | None = None) -> None:
        self._arguments.sort(key=lambda item: item['slot'])
        self.table.setRowCount(len(self._arguments))
        for row, argument in enumerate(self._arguments):
            values = (
                str(argument['slot']), argument['name'],
                _location_text(argument),
                ', '.join(sorted(argument['buttons'])) or 'None',
            )
            for column, value in enumerate(values):
                self.table.setItem(row, column, QTableWidgetItem(value))
            if argument['slot'] == selected_slot:
                self.table.selectRow(row)
        if selected_slot is None and self._arguments:
            self.table.selectRow(0)

    def _selected_argument(self) -> dict | None:
        row = self.table.currentRow()
        return self._arguments[row] if 0 <= row < len(self._arguments) else None

    def _edit_selected(self) -> None:
        argument = self._selected_argument()
        if argument is None:
            return
        dialog = LaunchArgumentDetailsDialog(argument, self._buttons, None)   # top-level: moves independently
        if dialog.exec_() != QDialog.Accepted:
            return
        updated = dialog.argument()
        argument.update(updated)
        self._refresh_table(argument['slot'])

    def _add_argument(self) -> None:
        used = {argument['slot'] for argument in self._arguments}
        slot = next((s for s in GENERIC_BUTTON_ARG_SLOTS if s not in used), None)
        if slot is None:
            popups.warning(self, 'Launch Options', 'All argument slots are used.')
            return
        argument = {
            'slot': slot, 'name': '', 'options': [], 'description': '',
            'option_descriptions': {}, 'advanced': True, 'scope': None,
            'buttons': set(),
        }
        dialog = LaunchArgumentDetailsDialog(argument, self._buttons, None)   # top-level: moves independently
        if dialog.exec_() == QDialog.Accepted:
            self._arguments.append(dialog.argument())
            self._refresh_table(slot)

    def _remove_selected(self) -> None:
        argument = self._selected_argument()
        if argument is None:
            return
        self._arguments.remove(argument)
        self._refresh_table()

    def button_layout(self) -> list[dict]:
        """Return updated profile entries after a successful save."""
        return copy.deepcopy(self._entries)

    def accept(self) -> None:
        names = [argument['name'] for argument in self._arguments]
        if len(names) != len(set(names)):
            popups.warning(
                self, 'Launch Options', 'Argument names must be unique.'
            )
            return
        if sum(not arg['advanced'] for arg in self._arguments) > 6:
            popups.warning(
                self, 'Launch Options',
                'The main window can hold at most six arguments plus world. '
                'Move another argument to Advanced Launch Options.',
            )
            return
        for entry in self._entries:
            for slot in GENERIC_BUTTON_ARG_SLOTS:
                for field in ARG_FIELDS:
                    entry.pop(f'arg_{slot}_{field}', None)
        for argument in self._arguments:
            slot = argument['slot']
            applying = argument['buttons']
            targets = [
                entry for entry in self._entries
                if entry.get('key') in applying
            ] or self._entries[:1]
            for entry in targets:
                entry.update({
                    f'arg_{slot}_name': argument['name'],
                    f'arg_{slot}_options': list(argument['options']),
                    f'arg_{slot}_description': argument['description'],
                    f'arg_{slot}_option_descriptions': dict(
                        argument['option_descriptions']
                    ),
                    f'arg_{slot}_advanced': argument['advanced'],
                    f'arg_{slot}_applies': entry.get('key') in applying,
                })
                if argument.get('scope') is not None:
                    entry[f'arg_{slot}_scope'] = normalize_arg_scope(
                        argument['scope']
                    )
        super().accept()
