"""Launch argument scopes: sim-only arguments never reach the real robot, real-robot-only ones never the simulation,
and the Advanced Launch Options dialog shows them in the tabs Real robot, Sim only and Shared."""

import os

import pytest
import yaml

os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')

from PyQt5.QtWidgets import QApplication, QLabel, QTabWidget

from mobipick_gui import popups
from mobipick_gui.config import load_button_layout, normalize_arg_scope, save_button_layout
from mobipick_gui.features_dialog import FeaturesDialog
from mobipick_gui.launch_arguments_dialog import (
    LaunchArgumentDetailsDialog,
    LaunchArgumentsDialog,
    profile_arguments,
)
from mobipick_gui.main_window import (
    REAL_ONLY_OFF_REASON,
    SIM_ONLY_OFF_REASON,
    MainWindow,
)
from mobipick_gui.option_rules import parse_option_rules
from test_features import FEATURES
from test_remote_ros_master import _create_window

ENTRY = {
    'key': 'demo', 'label': 'Demo', 'kind': 'command', 'command': 'roslaunch demo demo.launch',
    'arg_1_name': 'gazebo_gui', 'arg_1_options': ['false', 'true'], 'arg_1_applies': True,
    'arg_1_advanced': True, 'arg_1_scope': 'sim', 'arg_1_description': 'Show the Gazebo window',
    'arg_2_name': 'sensor_pc', 'arg_2_options': ['os-sensor', 'nuc'], 'arg_2_applies': True,
    'arg_2_advanced': True, 'arg_2_scope': 'real',
    'arg_3_name': 'use_mtc', 'arg_3_options': ['true', 'false'], 'arg_3_applies': True,
    'arg_3_advanced': True,
    'arg_4_name': 'num_robots', 'arg_4_options': ['1', '2'], 'arg_4_applies': True,
    'arg_4_advanced': False, 'arg_4_scope': 'sim',   # a sim-only selector in the toolbar
}


def _profile():
    return [
        {'key': 'sim', 'label': 'Sim', 'kind': 'builtin', 'action': 'sim', 'command': 'roslaunch demo_sim.launch',
         'arg_1_name': 'gui', 'arg_1_options': ['true', 'false'], 'arg_1_applies': True, 'arg_1_scope': 'sim',
         'arg_2_name': 'robot_ip', 'arg_2_options': ['a', 'b'], 'arg_2_applies': True},
        {'key': 'bringup', 'label': 'Bringup', 'kind': 'command', 'command': 'roslaunch bringup.launch'},
    ]


# -- profile ---------------------------------------------------------------

def test_scope_defaults_to_shared_and_unknown_values_count_as_shared():
    assert normalize_arg_scope(None) == 'shared'
    assert normalize_arg_scope('') == 'shared'
    assert normalize_arg_scope('SIM') == 'sim'
    assert normalize_arg_scope('real') == 'real'
    assert normalize_arg_scope('robot') == 'shared'


def test_scope_round_trips_and_is_not_written_when_absent(tmp_path):
    target = tmp_path / 'profile.yaml'
    save_button_layout(target, _profile())
    saved = yaml.safe_load(target.read_text())['buttons']
    sim = next(entry for entry in saved if entry['key'] == 'sim')
    assert sim['arg_1_scope'] == 'sim'
    assert not any('arg_2_scope' in entry for entry in saved)   # absent stays absent: minimal YAML diffs

    loaded = {entry['key']: entry for entry in load_button_layout(target)}
    assert loaded['sim']['arg_1_scope'] == 'sim'
    assert loaded['sim']['arg_2_scope'] is None
    scopes = {argument['name']: argument['scope'] for argument in profile_arguments(list(loaded.values()))}
    assert scopes == {'gui': 'sim', 'robot_ip': None}


def test_scope_survives_when_the_strip_drops_the_copy_that_carries_it(tmp_path):
    target = tmp_path / 'profile.yaml'
    entries = _profile()
    entries[1].update({'arg_1_name': 'gui', 'arg_1_options': ['true', 'false'], 'arg_1_applies': True})
    entries[0].update({'arg_1_applies': False})   # the carrier does not apply: its copy is dropped
    save_button_layout(target, entries)
    loaded = {entry['key']: entry for entry in load_button_layout(target)}
    assert loaded['bringup']['arg_1_scope'] == 'sim'


def test_editor_scope_combo_writes_the_scope_back(tmp_path):
    app = QApplication.instance() or QApplication([])
    entries = load_button_layout_from(tmp_path, _profile())
    argument = next(arg for arg in profile_arguments(entries) if arg['name'] == 'robot_ip')
    details = LaunchArgumentDetailsDialog(argument, [('sim', 'Sim'), ('bringup', 'Bringup')], None)
    try:
        assert details.parent() is None
        assert details.scope_combo.currentData() == 'shared'
        details.accept()   # Shared kept on an unset scope stays unset
        assert details.argument()['scope'] is None
        details.scope_combo.setCurrentIndex(details.scope_combo.findData('real'))
        details.accept()
        assert details.argument()['scope'] == 'real'
    finally:
        details.deleteLater()

    editor = LaunchArgumentsDialog(entries, None)
    try:
        robot_ip = next(arg for arg in editor._arguments if arg['name'] == 'robot_ip')
        robot_ip['scope'] = 'real'
        editor._refresh_table()
        locations = [editor.table.item(row, 2).text() for row in range(editor.table.rowCount())]
        assert 'Advanced, Sim only' in locations and 'Main window, Real robot only' in locations
        editor.accept()
        layout = editor.button_layout()
    finally:
        editor.deleteLater()
        app.processEvents()
    target = tmp_path / 'edited.yaml'
    save_button_layout(target, layout)
    sim = next(entry for entry in yaml.safe_load(target.read_text())['buttons'] if entry['key'] == 'sim')
    assert sim['arg_1_scope'] == 'sim' and sim['arg_2_scope'] == 'real'


def load_button_layout_from(tmp_path, entries):
    path = tmp_path / 'source.yaml'
    save_button_layout(path, entries)
    return load_button_layout(path)


# -- main window -----------------------------------------------------------

def _window(monkeypatch, tmp_path, entry=None, rules=None):
    app, window = _create_window(monkeypatch, tmp_path)
    entry = dict(entry or ENTRY)
    window._button_layout = [entry]
    window._config_buttons = {**window._config_buttons, 'demo': entry}
    window._refresh_generic_arg_controls()
    if rules is not None:
        window._option_rules = parse_option_rules(rules)
        window._apply_option_rules()
    return app, window, entry


def _tab_texts(window):
    tabs = window._advanced_tabs
    return [tabs.tabText(index) for index in range(tabs.count())]


def _tab_names(window, scope):
    layout = window._advanced_arg_layouts[scope]
    return [layout.itemAt(i).widget().text() for i in range(layout.count())
            if isinstance(layout.itemAt(i).widget(), QLabel)]


def _page_enabled(window, scope):
    return window._advanced_scrolls[scope].widget().isEnabled()


def _command(window, entry):
    return MainWindow._command_with_generic_args(window, entry['command'], entry)


def _close(app, window):
    for box in popups.open_boxes():   # the shared rules' notes, e.g. the world switched for the real robot
        box.close()
    window.advanced_launch_dialog.close()
    window.close()
    app.processEvents()


def test_rows_land_in_the_tab_of_their_scope(monkeypatch, tmp_path):
    app, window, _entry = _window(monkeypatch, tmp_path)
    try:
        assert isinstance(window._advanced_tabs, QTabWidget)
        assert window.advanced_launch_dialog.parent() is None   # top-level: moves independently
        assert _tab_texts(window) == ['Real robot (1, off in the simulation)', 'Sim only (1)', 'Shared (1)']
        assert _tab_names(window, 'real') == ['sensor_pc:']
        assert _tab_names(window, 'sim') == ['gazebo_gui:']
        assert _tab_names(window, 'shared') == ['use_mtc:']
        assert window._advanced_arg_layout is window._advanced_arg_layouts['shared']

        window._advanced_filter.setText('gazebo')   # one search box filters every tab
        assert _tab_texts(window)[1:] == ['Sim only (1)', 'Shared (0)']
        empty = window._advanced_tab_empty_labels['shared']
        assert not empty.isHidden() and 'matches the search' in empty.text()
        window._advanced_filter.setText('nothing such')
        assert not window._advanced_no_match_label.isHidden()
        window._advanced_filter.setText('')
        assert window._advanced_no_match_label.isHidden()
    finally:
        _close(app, window)


def test_empty_tab_shows_a_grey_note(monkeypatch, tmp_path):
    entry = {key: value for key, value in ENTRY.items() if not key.startswith('arg_2_')}
    app, window, _entry = _window(monkeypatch, tmp_path, entry)
    try:
        empty = window._advanced_tab_empty_labels['real']
        assert not empty.isHidden()
        assert empty.text() == 'No real-robot-only option in this profile.'
        assert window._advanced_tab_empty_labels['shared'].isHidden()
    finally:
        _close(app, window)


def test_sim_mode_greys_and_omits_real_robot_only_arguments(monkeypatch, tmp_path):
    app, window, entry = _window(monkeypatch, tmp_path)
    try:
        inputs = window._generic_arg_inputs
        assert inputs[1].isEnabled() and inputs[3].isEnabled() and inputs[4].isEnabled()
        assert not inputs[2].isEnabled()
        assert inputs[2].rule_note.text() == REAL_ONLY_OFF_REASON
        assert not _page_enabled(window, 'real') and _page_enabled(window, 'sim')
        assert window._advanced_tabs.isTabEnabled(0)   # the tab still opens, to look at it

        command = _command(window, entry)
        assert "gazebo_gui:='false'" in command and "num_robots:='1'" in command and 'sensor_pc' not in command
        assert window.button_args('demo') == {'gazebo_gui': 'false', 'use_mtc': 'true', 'num_robots': '1'}
        args = {arg['name']: arg for arg in window.generic_args()}
        assert args['sensor_pc']['scope'] == 'real' and args['sensor_pc']['disabled'] == REAL_ONLY_OFF_REASON
        assert args['gazebo_gui']['scope'] == 'sim' and args['gazebo_gui']['disabled'] == ''
        assert args['use_mtc']['scope'] == 'shared'
        with pytest.raises(ValueError, match='Real robot only'):
            window.set_generic_args({'sensor_pc': 'nuc'})
        window.set_generic_args({'gazebo_gui': 'true'})
        assert inputs[1].currentText() == 'true'
    finally:
        _close(app, window)


def test_real_robot_mode_greys_and_omits_sim_only_arguments_and_toggling_back_restores(monkeypatch, tmp_path):
    app, window, entry = _window(monkeypatch, tmp_path)
    try:
        inputs = window._generic_arg_inputs
        window.set_generic_args({'gazebo_gui': 'true', 'num_robots': '2'})
        window.remote_master_checkbox.setChecked(True)
        assert window._remote_master_enabled()

        assert not inputs[1].isEnabled() and not inputs[4].isEnabled()   # the toolbar one too
        assert inputs[1].rule_note.text() == SIM_ONLY_OFF_REASON and not inputs[1].rule_note.isHidden()
        assert SIM_ONLY_OFF_REASON in inputs[4].toolTip()
        assert inputs[2].isEnabled() and inputs[3].isEnabled()
        assert inputs[1].currentText() == 'true' and inputs[4].currentText() == '2'   # values kept
        assert _tab_texts(window) == ['Real robot (1)', 'Sim only (1, off on the real robot)', 'Shared (1)']
        assert not _page_enabled(window, 'sim') and _page_enabled(window, 'real')
        assert window._advanced_tabs.isTabEnabled(1)

        command = _command(window, entry)
        assert 'gazebo_gui' not in command and 'num_robots' not in command
        assert "sensor_pc:='os-sensor'" in command and "use_mtc:='true'" in command
        assert window.button_args('demo') == {'sensor_pc': 'os-sensor', 'use_mtc': 'true'}
        args = {arg['name']: arg for arg in window.generic_args()}
        assert args['gazebo_gui']['disabled'] == SIM_ONLY_OFF_REASON
        assert args['sensor_pc']['disabled'] == ''
        with pytest.raises(ValueError, match='Sim only'):
            window.set_generic_args({'gazebo_gui': 'false'})
        assert inputs[1].currentText() == 'true'
        window.set_generic_args({'sensor_pc': 'nuc'})

        window.remote_master_checkbox.setChecked(False)
        assert inputs[1].isEnabled() and inputs[4].isEnabled() and inputs[1].rule_note.isHidden()
        assert not inputs[2].isEnabled()
        assert _page_enabled(window, 'sim') and not _page_enabled(window, 'real')
        command = _command(window, entry)
        assert "gazebo_gui:='true'" in command and "num_robots:='2'" in command and 'sensor_pc' not in command
    finally:
        _close(app, window)


def test_headless_and_rule_start_args_do_not_bring_a_sim_only_argument_back(monkeypatch, tmp_path):
    entry = dict(ENTRY, headless_args={'gazebo_gui': 'false', 'rviz': 'false'})
    app, window, entry = _window(monkeypatch, tmp_path, entry)
    try:
        window._headless_mode = True
        window._remote_master_enabled_value = True
        window._apply_option_rules()
        command = _command(window, entry)
        assert 'gazebo_gui' not in command and "rviz:='false'" in command
    finally:
        window._headless_mode = False
        _close(app, window)


def test_a_rule_reason_is_kept_after_the_scope_reason(monkeypatch, tmp_path):
    rules = {'rules': [
        {'when': {'remote_master': True}, 'disable': ['gazebo_gui'], 'reason': 'no Gazebo on the robot'},
        {'when': {'remote_master': True}, 'disable': ['use_mtc'], 'reason': 'MTC is fixed on the robot'},
    ]}
    app, window, _entry = _window(monkeypatch, tmp_path, rules=rules)
    try:
        window.remote_master_checkbox.setChecked(True)
        inputs = window._generic_arg_inputs
        assert inputs[1].rule_note.text() == f'{SIM_ONLY_OFF_REASON}\nno Gazebo on the robot'
        assert inputs[3].rule_note.text() == 'MTC is fixed on the robot'
        args = {arg['name']: arg for arg in window.generic_args()}
        assert args['gazebo_gui']['disabled'] == f'{SIM_ONLY_OFF_REASON}; no Gazebo on the robot'
    finally:
        _close(app, window)


def test_dialog_width_follows_the_widest_tab(monkeypatch, tmp_path):
    entry = dict(ENTRY)
    for slot in range(5, 10):
        entry.update({f'arg_{slot}_name': f'sim_option_{slot}', f'arg_{slot}_options': ['a', 'b'],
                      f'arg_{slot}_advanced': True, f'arg_{slot}_scope': 'sim'})
    app, window, _entry = _window(monkeypatch, tmp_path, entry)
    try:
        assert window._advanced_columns == 2   # 6 sim-only options take two columns, the others one
        MainWindow._show_advanced_launch_dialog(window)
        assert window.advanced_launch_dialog.isVisible()
        assert not window.advanced_launch_dialog.isModal()
    finally:
        _close(app, window)


# -- Features window -------------------------------------------------------

FEATURE_LAYOUT = {
    'key': 'sim',
    'arg_1_name': 'skip_untangle', 'arg_1_options': ['false', 'true'], 'arg_1_advanced': True,
    'arg_2_name': 'align_jaw', 'arg_2_options': ['false', 'true'], 'arg_2_advanced': True, 'arg_2_scope': 'sim',
    'arg_3_name': 'narrow_side', 'arg_3_options': ['true', 'false'], 'arg_3_advanced': True,
    'arg_4_name': 'box_padding', 'arg_4_options': ['0.01', '0.005'], 'arg_4_advanced': True, 'arg_4_scope': 'real',
}


def _features_window(monkeypatch, tmp_path):
    app, window = _create_window(monkeypatch, tmp_path)
    profile = tmp_path / 'demo.yaml'
    profile.write_text('buttons: []\n')
    (tmp_path / 'demo_features.yaml').write_text(yaml.safe_dump(FEATURES))
    (tmp_path / 'demo_presets').mkdir()
    preset = {'name': 'all-on', 'description': 'all on', 'target': 'real',
              'features_on': ['jaw', 'narrow', 'padding']}
    (tmp_path / 'demo_presets' / 'all-on.yaml').write_text(yaml.safe_dump(preset))
    monkeypatch.setattr(MainWindow, '_resolved_button_config_path', lambda self: profile)
    monkeypatch.setattr(MainWindow, '_user_presets_dir', lambda self: tmp_path / 'user_presets')
    window._button_layout = [FEATURE_LAYOUT]
    window._refresh_generic_arg_controls()
    MainWindow._load_features(window)
    window._option_rules = parse_option_rules({'rules': []})
    return app, window


def _members(window):
    return {member['key']: member for group in window.feature_status()['groups'] for member in group['members']}


def test_features_whose_options_are_all_off_by_scope_are_greyed_out(monkeypatch, tmp_path):
    app, window = _features_window(monkeypatch, tmp_path)
    dialog = None
    try:
        members = _members(window)   # the sim: the real-robot-only padding is off
        assert members['padding']['scope_off'] == REAL_ONLY_OFF_REASON
        assert members['jaw']['scope_off'] == '' and members['narrow']['scope_off'] == ''

        window.remote_master_checkbox.setChecked(True)
        members = _members(window)
        assert members['jaw']['scope_off'] == SIM_ONLY_OFF_REASON
        assert members['padding']['scope_off'] == '' and members['narrow']['scope_off'] == ''
        dialog = FeaturesDialog(window)
        dialog.refresh()
        assert dialog.parent() is None
        assert not dialog._feature_boxes['jaw'].isEnabled()
        assert dialog._feature_boxes['jaw'].toolTip() == SIM_ONLY_OFF_REASON

        report = window.load_feature_preset('all-on')   # a preset does not change a scope-greyed option
        assert report['refused']['align_jaw']['reason'] == f'disabled: {SIM_ONLY_OFF_REASON}'
        assert {arg['name']: arg['value'] for arg in window.generic_args()}['align_jaw'] == 'false'
    finally:
        if dialog is not None:
            dialog.close()
        _close(app, window)
