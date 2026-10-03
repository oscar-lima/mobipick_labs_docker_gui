"""Regression tests for the profile-wide launch argument editor."""

import os

os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')

from PyQt5.QtWidgets import QApplication, QDialog, QMessageBox

from mobipick_gui.config import load_button_layout, save_button_layout
from mobipick_gui.launch_arguments_dialog import (
    LaunchArgumentDetailsDialog,
    LaunchArgumentsDialog,
    profile_arguments,
)
from mobipick_gui.main_window import ButtonArgumentsDialog


def _profile():
    return [
        {
            'key': 'sim', 'label': 'Sim', 'kind': 'builtin', 'action': 'sim',
            'command': 'roslaunch demo_sim.launch',
            'arg_1_name': 'gui', 'arg_1_options': ['true', 'false'],
            'arg_1_applies': True, 'arg_1_advanced': True,
            'arg_1_description': 'Show the Gazebo client window.',
            'arg_1_option_descriptions': {
                'true': 'Open the window', 'false': 'Stay headless',
            },
        },
        {
            'key': 'bringup', 'label': 'Bringup', 'kind': 'command',
            'command': 'roslaunch bringup.launch',
        },
    ]


def test_profile_round_trip_preserves_argument_and_value_descriptions(tmp_path):
    target = tmp_path / 'profile.yaml'
    entries = _profile()
    entries[0].update({
        'arg_24_name': 'mtc_cable_constraints',
        'arg_24_options': ['true', 'false'],
        'arg_24_applies': True,
        'arg_24_advanced': True,
        'arg_24_description': 'Check arm cable stretch.',
    })
    save_button_layout(target, entries)

    loaded = {entry['key']: entry for entry in load_button_layout(target)}

    assert loaded['sim']['arg_1_description'] == (
        'Show the Gazebo client window.'
    )
    assert loaded['sim']['arg_1_option_descriptions'] == {
        'true': 'Open the window', 'false': 'Stay headless',
    }
    assert loaded['sim']['arg_24_description'] == 'Check arm cable stretch.'


def test_slots_above_24_are_kept_by_the_profile_round_trip(tmp_path):
    # the tables demo profile has 26 named slots (#236): arg_25 and arg_26 were silently dropped at 24 slots
    target = tmp_path / 'profile.yaml'
    entries = _profile()
    entries[0].update({
        'arg_26_name': 'reset_cache_mrad',
        'arg_26_options': ['5', '0'],
        'arg_26_applies': True,
        'arg_26_advanced': True,
        'arg_26_description': 'Start-pose cache.',
    })
    save_button_layout(target, entries)

    loaded = {entry['key']: entry for entry in load_button_layout(target)}

    assert loaded['sim']['arg_26_name'] == 'reset_cache_mrad'
    assert loaded['sim']['arg_26_options'] == ['5', '0']


def test_slots_33_to_64_are_kept_listed_and_reach_the_command_line(tmp_path):
    # the tables demo profile grew past 32 slots (arg_33..45): the GUI read slots 1..32 only (#328)
    from types import SimpleNamespace
    from mobipick_gui.main_window import MainWindow

    class _Combo:
        def __init__(self, value):
            self._value = value

        def currentText(self):
            return self._value

    target = tmp_path / 'profile.yaml'
    entries = _profile()
    for slot, name in ((33, 'drop_floating_boxes_at_target'), (45, 'use_mesh_object_heights'), (64, 'last_slot')):
        entries[0].update({
            f'arg_{slot}_name': name,
            f'arg_{slot}_options': ['true', 'false'],
            f'arg_{slot}_applies': True,
            f'arg_{slot}_advanced': True,
            f'arg_{slot}_description': f'Slot {slot}.',
        })
    entries[0].update({'arg_65_name': 'beyond', 'arg_65_options': ['a'], 'arg_65_applies': True})
    save_button_layout(target, entries)

    loaded = load_button_layout(target)
    sim = {entry['key']: entry for entry in loaded}['sim']
    for slot, name in ((33, 'drop_floating_boxes_at_target'), (45, 'use_mesh_object_heights'), (64, 'last_slot')):
        assert sim[f'arg_{slot}_name'] == name
        assert sim[f'arg_{slot}_options'] == ['true', 'false']
        assert sim[f'arg_{slot}_description'] == f'Slot {slot}.'
    assert 'arg_65_name' not in sim
    listed = {argument['slot']: argument['name'] for argument in profile_arguments(loaded)}
    assert listed[33] == 'drop_floating_boxes_at_target'
    assert listed[45] == 'use_mesh_object_heights'
    assert 65 not in listed

    window = SimpleNamespace(
        _generic_arg_inputs={33: _Combo('true'), 45: _Combo('false')},
        _headless_mode=False,
        _option_rules=None,
        _sh_quote=MainWindow._sh_quote,
    )
    command = MainWindow._command_with_generic_args(window, 'roslaunch demo_sim.launch', sim)
    assert command.endswith("drop_floating_boxes_at_target:='true' use_mesh_object_heights:='false'")


def test_explicit_main_placement_overrides_advanced_name_default(tmp_path):
    entries = _profile()
    entries[0]['arg_1_advanced'] = False
    target = tmp_path / 'profile.yaml'
    save_button_layout(target, entries)

    loaded = load_button_layout(target)

    assert loaded[0]['arg_1_advanced'] is False
    assert profile_arguments(loaded)[0]['advanced'] is False


def test_yaml_boolean_value_description_keys_match_combo_values(tmp_path):
    target = tmp_path / 'profile.yaml'
    target.write_text(
        'buttons:\n'
        '  - key: sim\n'
        '    command: roslaunch demo_sim.launch\n'
        '    arg_1_name: gui\n'
        '    arg_1_options: ["true", "false"]\n'
        '    arg_1_option_descriptions:\n'
        '      true: Open Gazebo\n'
    )

    loaded = load_button_layout(target)

    assert loaded[0]['arg_1_option_descriptions'] == {
        'true': 'Open Gazebo'
    }


def test_per_button_arguments_dialog_edits_descriptions():
    app = QApplication.instance() or QApplication([])
    dialog = ButtonArgumentsDialog(_profile()[0])
    dialog._description_inputs[1].setPlainText('Show the simulation window.')
    dialog._option_description_inputs[1].setPlainText(
        'true | Open Gazebo\nfalse | Run headless'
    )
    dialog.accept()

    assert dialog.result() == QDialog.Accepted
    values = dialog.arguments()
    assert values['arg_1_description'] == 'Show the simulation window.'
    assert values['arg_1_option_descriptions'] == {
        'true': 'Open Gazebo', 'false': 'Run headless',
    }
    dialog.deleteLater()
    app.processEvents()


def test_argument_details_values_field_expands_with_dialog():
    app = QApplication.instance() or QApplication([])
    editor = LaunchArgumentsDialog(_profile())
    details = LaunchArgumentDetailsDialog(
        editor._arguments[0], editor._buttons, editor
    )
    details.show()
    app.processEvents()
    narrow = details.options_input.width()
    details.resize(details.width() + 240, details.height())
    app.processEvents()

    assert details.options_input.width() > narrow
    details.close()
    editor.deleteLater()
    app.processEvents()


def test_advanced_editor_updates_all_applicable_buttons_and_reloads(tmp_path):
    app = QApplication.instance() or QApplication([])
    editor = LaunchArgumentsDialog(_profile())
    argument = editor._arguments[0]
    details = LaunchArgumentDetailsDialog(argument, editor._buttons, editor)
    details.name_input.setText('use_mtc')
    details.options_input.setText('true, false')
    details.description_input.setPlainText('Use MoveIt Task Constructor.')
    details.value_descriptions_input.setPlainText(
        'true | Use MTC\nfalse | Use the original planner'
    )
    details.button_checks['bringup'].setChecked(True)
    details.accept()
    assert details.result() == QDialog.Accepted
    argument.update(details.argument())
    editor.accept()
    assert editor.result() == QDialog.Accepted

    target = tmp_path / 'profile.yaml'
    save_button_layout(target, editor.button_layout())
    reloaded = {entry['key']: entry for entry in load_button_layout(target)}

    for key in ('sim', 'bringup'):
        assert reloaded[key]['arg_1_name'] == 'use_mtc'
        assert reloaded[key]['arg_1_applies'] is True
        assert reloaded[key]['arg_1_description'] == (
            'Use MoveIt Task Constructor.'
        )
        assert reloaded[key]['arg_1_option_descriptions']['false'] == (
            'Use the original planner'
        )
    editor.deleteLater()
    app.processEvents()


def test_advanced_editor_can_add_and_remove_arguments(monkeypatch):
    app = QApplication.instance() or QApplication([])
    editor = LaunchArgumentsDialog(_profile())

    def accept_new(dialog):
        dialog._argument.update({
            'name': 'use_mtc', 'options': ['true', 'false'],
            'buttons': {'sim', 'bringup'},
        })
        return QDialog.Accepted

    monkeypatch.setattr(LaunchArgumentDetailsDialog, 'exec_', accept_new)
    editor._add_argument()
    assert len(editor._arguments) == 2
    assert editor._arguments[1]['name'] == 'use_mtc'
    editor._remove_selected()
    assert len(editor._arguments) == 1
    editor.deleteLater()
    app.processEvents()


def test_advanced_editor_rejects_eighth_main_launch_option(monkeypatch):
    app = QApplication.instance() or QApplication([])
    editor = LaunchArgumentsDialog(_profile())
    editor._arguments = [
        {
            'slot': slot, 'name': f'option_{slot}', 'options': ['on'],
            'description': '', 'option_descriptions': {},
            'advanced': False, 'buttons': {'sim'},
        }
        for slot in range(1, 8)
    ]
    warnings = []
    monkeypatch.setattr(
        QMessageBox, 'warning',
        lambda *args: warnings.append(args[2]),
    )

    editor.accept()

    assert editor.result() != QDialog.Accepted
    assert 'at most six arguments plus world' in warnings[0]
    editor.deleteLater()
    app.processEvents()
