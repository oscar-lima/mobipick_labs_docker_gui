"""Features dialog: switch whole features on or off and load or save presets.

The dialog shows the features file of the button profile (see ``features.py``): a radio choice per mode group, a
tick box per feature with its class (safe or careful), its description and the options it sets. Ticking changes
nothing until Apply, which sets the launch arguments through the main window (``apply_feature_selection``), so
the option rules, the saved selections and the remote API see the same values as a change in Advanced Launch
Options. On a remote ROS master (the real robot) careful features stay off unless ticked there after a
confirmation.
"""

from __future__ import annotations

import html
from pathlib import Path

import yaml
from PyQt5.QtCore import Qt
from PyQt5.QtWidgets import (
    QButtonGroup,
    QCheckBox,
    QComboBox,
    QDialog,
    QFileDialog,
    QFrame,
    QHBoxLayout,
    QInputDialog,
    QLabel,
    QLineEdit,
    QMessageBox,
    QPushButton,
    QRadioButton,
    QScrollArea,
    QVBoxLayout,
    QWidget,
)

from . import features as feature_sets

NO_PRESET = '(no preset)'
DESCRIPTION_CHARS = 180
CAREFUL_ON_REAL = 'careful: tick to test it on the real robot, e-stop in hand'


def _badge(cls: str) -> QLabel:
    careful = cls == feature_sets.CAREFUL
    badge = QLabel(cls)
    badge.setStyleSheet(
        'border-radius: 7px; padding: 0px 6px; font-size: 11px; '
        + ('color: #8a5300; background: #ffe7b8; border: 1px solid #d9a441;' if careful
           else 'color: #1d6b2f; background: #dcf5e1; border: 1px solid #7cc58e;')
    )
    badge.setToolTip('careful: it can change arm or base motion; off on the real robot until a supervised test'
                     if careful else 'safe: it cannot damage the robot')
    return badge


def _plain(text: str) -> str:
    return text.replace('`', '')


def _short(text: str) -> str:
    text = _plain(text)
    return text if len(text) <= DESCRIPTION_CHARS else text[: DESCRIPTION_CHARS - 1].rstrip() + '…'


def _sets(values: dict[str, str]) -> str:
    return '  '.join(f'{name}:={value}' for name, value in values.items())


class _Row(QWidget):
    """One mode or feature: its button, class badge, description, the options it sets and a note."""

    def __init__(self, button, cls: str, description: str, values: dict[str, str], indent: int = 0,
                 pending: str = ''):
        super().__init__()
        self.button = button
        self.search_text = ' '.join([button.text(), description, ' '.join(values)]).lower()
        layout = QVBoxLayout(self)
        layout.setContentsMargins(8 + 22 * indent, 4, 8, 4)
        layout.setSpacing(1)
        head = QHBoxLayout()
        font = button.font()
        font.setBold(True)
        button.setFont(font)
        head.addWidget(button)
        head.addWidget(_badge(cls))
        head.addStretch(1)
        layout.addLayout(head)
        self.note = QLabel('')
        self.note.setWordWrap(True)
        self.note.setStyleSheet('color: #c0392b;')
        self.note.setContentsMargins(24, 0, 0, 0)
        self.note.setVisible(False)
        layout.addWidget(self.note)
        if description:
            text = QLabel(_short(description))
            text.setWordWrap(True)
            text.setToolTip(_plain(description))
            text.setStyleSheet('color: palette(mid);')
            text.setContentsMargins(24, 0, 0, 0)
            layout.addWidget(text)
        option_line = QLabel(_sets(values) + (f'\n{pending}' if pending else ''))
        option_line.setWordWrap(True)
        option_line.setStyleSheet('font-family: monospace; font-size: 11px;')
        option_line.setTextInteractionFlags(Qt.TextSelectableByMouse)
        option_line.setContentsMargins(24, 0, 0, 0)
        layout.addWidget(option_line)

    def set_note(self, text: str) -> None:
        self.note.setText(text)
        self.note.setVisible(bool(text))


class FeaturesDialog(QDialog):
    """Non-modal top-level window for ``window`` (the main window): no parent, so it moves independently of it."""

    def __init__(self, window):
        super().__init__(None)
        self.window = window
        self.setWindowTitle('Features')
        self.setMinimumWidth(640)
        self._allowed_careful: set[str] = set()
        self._feature_boxes: dict[str, QCheckBox] = {}
        self._usable: dict[str, bool] = {}   # feature key -> its options exist and the rules allow it
        self._mode_buttons: dict[str, dict[str, QRadioButton]] = {}
        self._rows: list[_Row] = []
        self._headers: list[tuple[QLabel, list[_Row]]] = []
        self._building = False

        root = QVBoxLayout(self)
        top = QHBoxLayout()
        top.addWidget(QLabel('Preset:'))
        self.preset_combo = QComboBox()
        self.preset_combo.setMinimumContentsLength(18)
        self.preset_combo.activated.connect(self._preset_chosen)
        top.addWidget(self.preset_combo, 1)
        load_button = QPushButton('Load file...')
        load_button.setToolTip('Import a preset file into your presets and show it')
        load_button.clicked.connect(self._load_file)
        top.addWidget(load_button)
        save_button = QPushButton('Save as...')
        save_button.setToolTip('Save the ticks below as one of your presets')
        save_button.clicked.connect(self._save_as)
        top.addWidget(save_button)
        self.target_label = QLabel('')
        self.target_label.setStyleSheet('font-weight: bold; padding: 2px 8px; border-radius: 8px;')
        top.addWidget(self.target_label)
        root.addLayout(top)

        self.warning_label = QLabel('')
        self.warning_label.setWordWrap(True)
        self.warning_label.setStyleSheet('color: #c0392b; font-weight: bold;')
        self.warning_label.setVisible(False)
        root.addWidget(self.warning_label)

        self.search = QLineEdit()
        self.search.setPlaceholderText('Search features by name, description or option')
        self.search.setClearButtonEnabled(True)
        self.search.textChanged.connect(self._filter)
        root.addWidget(self.search)

        self._content = QWidget()
        self._content_layout = QVBoxLayout(self._content)
        self._content_layout.setContentsMargins(0, 0, 0, 0)
        self._content_layout.setSpacing(0)
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setFrameShape(QFrame.StyledPanel)
        scroll.setWidget(self._content)
        root.addWidget(scroll, 1)

        self.summary_label = QLabel('')
        self.summary_label.setWordWrap(True)
        root.addWidget(self.summary_label)
        self.result_label = QLabel('')
        self.result_label.setWordWrap(True)
        self.result_label.setVisible(False)
        root.addWidget(self.result_label)

        buttons = QHBoxLayout()
        buttons.addStretch(1)
        self.apply_button = QPushButton('Apply')
        self.apply_button.clicked.connect(lambda: self.apply())
        buttons.addWidget(self.apply_button)
        self.launch_button = QPushButton('Apply and Auto Launch')
        self.launch_button.clicked.connect(self._apply_and_launch)
        buttons.addWidget(self.launch_button)
        close_button = QPushButton('Close')
        close_button.clicked.connect(self.close)
        buttons.addWidget(close_button)
        root.addLayout(buttons)

    # -- building --------------------------------------------------------

    def refresh(self) -> None:
        """Rebuild from the main window's features and show its current values."""
        self._building = True
        try:
            self._rebuild()
            status = self.window.feature_status()
            self._show_status(status)
            self._fill_presets(status)
        finally:
            self._building = False
        self._update_summary()
        screen = self.screen().availableGeometry() if self.screen() else None
        if screen is not None and not self.isVisible():
            self.resize(min(820, int(screen.width() * 0.9)), int(screen.height() * 0.85))

    def _rebuild(self) -> None:
        while self._content_layout.count():
            item = self._content_layout.takeAt(0)
            widget = item.widget()
            if widget is not None:   # hidden and detached now, or it stays painted until Qt deletes it
                widget.hide()
                widget.setParent(None)
                widget.deleteLater()
        self._feature_boxes.clear()
        self._mode_buttons.clear()
        self._rows.clear()
        self._headers.clear()
        features = self.window._features
        for group in features.groups:
            header = QLabel(group.name + (' (choose one)' if group.kind == 'modes' else ''))
            header.setStyleSheet('font-weight: bold; padding: 6px 8px; background: palette(alternate-base);')
            if group.description:
                header.setToolTip(_plain(group.description))
            self._content_layout.addWidget(header)
            rows: list[_Row] = []
            if group.kind == 'modes':
                buttons = QButtonGroup(self)
                self._mode_buttons[group.key] = {}
                sim_default = any(mode.sim_default for mode in features.modes[group.key].values())
                for mode in features.modes[group.key].values():
                    mark = (' (sim default)' if mode.sim_default else
                            (' (real robot default)' if sim_default else ' (default)') if mode.default else '')
                    radio = QRadioButton(mode.label + mark)
                    radio.toggled.connect(lambda on, g=group.key, m=mode.key: on and self._mode_toggled(g, m))
                    buttons.addButton(radio)
                    self._mode_buttons[group.key][mode.key] = radio
                    pending = (f'pending {mode.pending_on}, not set yet: {_sets(mode.pending)}'
                               if mode.pending else '')
                    row = _Row(radio, mode.cls, mode.description, mode.sets, pending=pending)
                    rows.append(row)
            else:
                for key in group.members:
                    feature = features.features[key]
                    box = QCheckBox(feature.label)
                    box.toggled.connect(lambda on, k=key: self._feature_toggled(k, on))
                    self._feature_boxes[key] = box
                    row = _Row(box, feature.cls, feature.description, feature.on, indent=1 if feature.parent else 0)
                    rows.append(row)
            for row in rows:
                self._content_layout.addWidget(row)
            self._rows += rows
            self._headers.append((header, rows))
        self._content_layout.addStretch(1)
        self._filter(self.search.text())

    def _show_status(self, status: dict) -> None:
        real = status['target'] == 'real'
        self.target_label.setText('Real robot' if real else 'Simulation')
        self.target_label.setToolTip('The GUI uses a remote ROS master: careful features stay off until ticked here'
                                     if real else 'The GUI runs the simulation (no remote ROS master)')
        self.target_label.setStyleSheet(
            'font-weight: bold; padding: 2px 8px; border-radius: 8px; color: white; background: '
            + ('#c0392b;' if real else '#2d6cdf;'))
        self.warning_label.setText('Real robot: careful choices start off here' if real else '')
        self.warning_label.setVisible(real)
        rows = iter(self._rows)
        for group in status['groups']:
            for member in group['members']:
                row = next(rows)
                note = ''
                if member['missing']:
                    note = 'not in this profile: ' + ', '.join(member['missing'])
                elif member.get('scope_off'):
                    note = member['scope_off']
                elif member['refused']:
                    note = '; '.join(sorted(set(member['refused'].values())))
                elif real and member['class'] == feature_sets.CAREFUL:
                    note = CAREFUL_ON_REAL
                elif member.get('mixed'):
                    note = 'mixed: ' + member['mixed']
                enabled = not member['missing'] and not member['refused']
                # all its options are left off on this target: greyed out also when on (Apply cannot change them)
                scope_off = bool(member.get('scope_off'))
                if group['kind'] == 'features':
                    self._usable[member['key']] = (enabled or member['on']) and not scope_off
                row.button.setEnabled((enabled or member['on']) and not scope_off)
                row.button.setToolTip(member.get('scope_off') or '')
                row.set_note(note)
                row.button.setChecked(bool(member['on']))
        self._apply_parents()

    def _fill_presets(self, status: dict) -> None:
        self.preset_combo.clear()
        self.preset_combo.addItem(NO_PRESET)
        for preset in status['presets']:
            label = preset['name'] + (' (yours)' if preset['user'] else '')
            self.preset_combo.addItem(label, preset['name'])
            index = self.preset_combo.count() - 1
            self.preset_combo.setItemData(index, f'{preset["description"]} (for the {preset["target"]})',
                                          Qt.ToolTipRole)
        current = status.get('preset')
        index = self.preset_combo.findData(current) if current else 0
        self.preset_combo.setCurrentIndex(max(index, 0))

    # -- ticks -----------------------------------------------------------

    def selection(self) -> feature_sets.Selection:
        sim = not self.window._remote_master_enabled()
        modes = {group: next((key for key, radio in radios.items() if radio.isChecked()),
                             self.window._features.default_mode(group, sim))
                 for group, radios in self._mode_buttons.items()}
        return feature_sets.Selection(modes, {key for key, box in self._feature_boxes.items() if box.isChecked()})

    def _confirm_careful(self, key: str, label: str, tick) -> bool:
        """True when ``key`` may be on now; else False and a non-modal top-level question (no parent, it moves
        independently of both windows) whose Yes allows the key and calls ``tick`` to tick it again."""
        if self._building or not self.window._remote_master_enabled() or key in self._allowed_careful:
            return True
        box = QMessageBox(
            QMessageBox.Question, 'Careful feature on the real robot',
            f'{label} is careful: it can change arm or base motion. Switch it on for the real robot? '
            'Keep the e-stop in hand.',
            QMessageBox.Yes | QMessageBox.No, None,
        )
        box.setWindowModality(Qt.NonModal)
        box.setAttribute(Qt.WA_DeleteOnClose)

        def answered(button) -> None:
            if box.standardButton(button) == QMessageBox.Yes:
                self._allowed_careful.add(key)
                tick()

        box.buttonClicked.connect(answered)
        self._careful_question = box
        box.show()
        return False

    def _feature_toggled(self, key: str, on: bool) -> None:
        feature = self.window._features.features[key]
        box = self._feature_boxes[key]
        if on and feature.careful and not self._confirm_careful(key, feature.label, lambda: box.setChecked(True)):
            box.setChecked(False)
            return
        self._apply_parents()
        self._update_summary()

    def _mode_toggled(self, group: str, mode_key: str) -> None:
        mode = self.window._features.modes[group][mode_key]
        radio = self._mode_buttons[group][mode_key]
        if mode.careful and not self._confirm_careful(mode_key, mode.label, lambda: radio.setChecked(True)):
            default = self.window._features.default_mode(group)
            self._mode_buttons[group][default].setChecked(True)
            return
        self._update_summary()

    def _apply_parents(self) -> None:
        for key, box in self._feature_boxes.items():
            parent = self.window._features.features[key].parent
            usable = self._usable.get(key, True)
            box.setEnabled(usable and (not parent or self._feature_boxes[parent].isChecked()))

    def _filter(self, text: str) -> None:
        needle = text.strip().lower()
        for header, rows in self._headers:
            shown = 0
            for row in rows:
                visible = not needle or needle in row.search_text
                row.setVisible(visible)
                shown += visible
            header.setVisible(shown > 0)

    def _update_summary(self) -> None:
        if self._building:
            return
        features = self.window._features
        selection = self.selection()
        on = [features.features[key] for key in selection.features_on]
        careful = sum(feature.careful for feature in on)
        modes = ', '.join(features.modes[group][mode].label for group, mode in selection.modes.items())
        text = (f'{modes} + {len(features.features)} features, {len(on)} on: '
                f'{len(on) - careful} safe, {careful} careful')
        name = self.preset_combo.currentData()
        preset = self.window._feature_presets.get(name) if name else None
        if preset is not None:
            wanted = feature_sets.preset_values(features, preset)
            mine = feature_sets.selection_values(features, selection)
            present = self.window._feature_option_values()   # options the profile lacks cannot differ
            differs = sorted(option for option, value in mine.items()
                             if option in present and wanted.get(option, value) != value)
            text += (f' · {len(differs)} option(s) differ from {name}: {", ".join(differs)}' if differs
                     else f' · as preset {name}')
        self.summary_label.setText(text)

    # -- presets ---------------------------------------------------------

    def _preset_chosen(self, index: int) -> None:
        name = self.preset_combo.itemData(index)
        preset = self.window._feature_presets.get(name) if name else None
        if preset is None:
            self._update_summary()
            return
        selection = feature_sets.selection_of(self.window._features, preset)
        self._building = True
        try:
            for group, radios in self._mode_buttons.items():
                radios[selection.modes[group]].setChecked(True)
            for key, box in self._feature_boxes.items():
                box.setChecked(key in selection.features_on)
        finally:
            self._building = False
        target = 'real' if self.window._remote_master_enabled() else 'sim'
        self._show_result(
            f'Preset {name} shown; Apply sets it.'
            + (f' It is meant for the {preset.target}, the GUI is on the {target}.' if preset.target != target else '')
        )
        self._apply_parents()
        self._update_summary()

    def _save_as(self) -> None:
        name, ok = QInputDialog.getText(None, 'Save preset', 'Preset name:')   # top-level: moves independently
        if not ok or not name.strip():
            return
        description, ok = QInputDialog.getText(None, 'Save preset', 'What it is (one line):')
        if not ok:
            return
        try:
            saved = self.window.save_feature_preset(name, description, self.selection())
        except (OSError, ValueError) as exc:
            self._show_result(f'Save preset failed: {exc}')
            return
        self._fill_presets(self.window.feature_status())
        self.preset_combo.setCurrentIndex(max(self.preset_combo.findData(saved['name']), 0))
        self._show_result(f'Saved preset {saved["name"]} to {saved["path"]}.')

    def _load_file(self) -> None:
        path, _filter = QFileDialog.getOpenFileName(None, 'Load preset', str(Path.home()), 'Preset (*.yaml *.yml)')
        if not path:
            return
        try:
            preset = feature_sets.parse_preset(yaml.safe_load(Path(path).read_text(encoding='utf-8')), Path(path))
            problems = feature_sets.preset_problems(self.window._features, preset)
            if problems:
                raise ValueError('; '.join(problems))
            saved = self.window.save_feature_preset(
                preset.name, preset.description, feature_sets.selection_of(self.window._features, preset),
                preset.options)
        except (OSError, ValueError, yaml.YAMLError) as exc:
            self._show_result(f'Load preset failed: {path}: {exc}')
            return
        self._fill_presets(self.window.feature_status())
        index = max(self.preset_combo.findData(saved['name']), 0)
        self.preset_combo.setCurrentIndex(index)
        self._preset_chosen(index)

    # -- apply -------------------------------------------------------------

    def apply(self) -> dict:
        name = self.preset_combo.currentData()
        preset = self.window._feature_presets.get(name) if name else None
        selection = self.selection()
        same = preset is not None and (
            feature_sets.selection_values(self.window._features, selection)
            == feature_sets.selection_values(self.window._features,
                                             feature_sets.selection_of(self.window._features, preset)))
        report = self.window.apply_feature_selection(
            selection,
            options=preset.options if same else None,
            preset=name if same else None,
            ticked_careful=self._allowed_careful,
        )
        self.refresh()
        lines = [f'Applied: {len(report["applied"])} option(s) changed.']
        if report['refused']:
            lines.append('Refused: ' + '; '.join(f'{n}={r["value"]} ({r["reason"]})'
                                                 for n, r in report['refused'].items()))
        if report['careful_kept_off']:
            lines.append('Careful, kept off on the real robot: ' + ', '.join(report['careful_kept_off']))
        if report['missing']:
            lines.append('Not in this profile: ' + ', '.join(report['missing']))
        self._show_result(' '.join(lines))
        return report

    def _apply_and_launch(self) -> None:
        self.apply()
        self.window._apply_features_and_auto_launch()

    def _show_result(self, text: str) -> None:
        self.result_label.setText(html.escape(text))
        self.result_label.setVisible(bool(text))
