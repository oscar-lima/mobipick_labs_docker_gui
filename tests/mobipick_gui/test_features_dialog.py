import os

import yaml

os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')

from mobipick_gui.features import Selection
from mobipick_gui.main_window import MainWindow
from mobipick_gui.option_rules import parse_option_rules
from mobipick_gui.remote_adapter import MainWindowRemoteAdapter
from test_features import FEATURES
from test_remote_ros_master import _create_window

LAYOUT = [{
    'key': 'sim',
    'arg_1_name': 'skip_untangle', 'arg_1_options': ['false', 'true'], 'arg_1_advanced': True,
    'arg_2_name': 'align_jaw', 'arg_2_options': ['true', 'false'], 'arg_2_advanced': True,
    'arg_3_name': 'narrow_side', 'arg_3_options': ['true', 'false'], 'arg_3_advanced': True,
    'arg_4_name': 'box_padding', 'arg_4_options': ['0.01', '0.005'], 'arg_4_advanced': True,
    'arg_5_name': 'cable_monitor', 'arg_5_options': ['false', 'true'], 'arg_5_advanced': True,
}]
REAL_PADDING = {'rules': [{'when': {'remote_master': True}, 'only': {'box_padding': ['0.01']},
                           'reason': 'the real robot keeps 1 cm until the supervised test'}]}
PRESET = {'name': 'sim-full', 'description': 'all on', 'target': 'sim',
          'modes': {'cable_handling': 'cable_aware'}, 'features_on': ['jaw', 'narrow', 'padding'],
          'options': {'cable_monitor': 'true'}}


def _window(monkeypatch, tmp_path, rules=None):
    app, window = _create_window(monkeypatch, tmp_path)
    profile = tmp_path / 'demo.yaml'
    profile.write_text('buttons: []\n')
    (tmp_path / 'demo_features.yaml').write_text(yaml.safe_dump(FEATURES))
    (tmp_path / 'demo_presets').mkdir()
    (tmp_path / 'demo_presets' / 'sim-full.yaml').write_text(yaml.safe_dump(PRESET))
    monkeypatch.setattr(MainWindow, '_resolved_button_config_path', lambda self: profile)
    monkeypatch.setattr(MainWindow, '_user_presets_dir', lambda self: tmp_path / 'user_presets')
    window._button_layout = LAYOUT
    window._refresh_generic_arg_controls()
    MainWindow._load_features(window)
    window._option_rules = parse_option_rules(rules or {'rules': []})
    return app, window


def _values(window):
    return {entry['name']: entry['value'] for entry in window.generic_args() if entry['name'] != 'world'}


def test_no_features_button_without_a_features_file(monkeypatch, tmp_path):
    app, window = _create_window(monkeypatch, tmp_path)
    try:
        assert not window.features_button.isVisibleTo(window)
        assert window.feature_status()['available'] is False
    finally:
        window.close()
        app.processEvents()


def test_features_button_and_status_with_a_features_file(monkeypatch, tmp_path):
    app, window = _window(monkeypatch, tmp_path)
    try:
        assert window.features_button.isVisibleTo(window)
        status = window.feature_status()
        assert status['available'] and status['target'] == 'sim'
        assert [p['name'] for p in status['presets']] == ['sim-full']
        modes = status['groups'][0]['members']
        assert [m['key'] for m in modes if m['on']] == ['hack']
        assert modes[1]['missing'] == ['pruning']   # the profile lacks it: reported, left alone
    finally:
        window.close()
        app.processEvents()


def test_preset_sets_every_option_and_reports_what_the_profile_lacks(monkeypatch, tmp_path):
    app, window = _window(monkeypatch, tmp_path)
    try:
        report = window.load_feature_preset('sim-full')
        assert _values(window) == {'skip_untangle': 'true', 'align_jaw': 'true', 'narrow_side': 'true',
                                   'box_padding': '0.005', 'cable_monitor': 'true'}
        assert report['missing'] == ['pruning'] and report['refused'] == {}
        assert window._preset_differences() == {}
        assert window._ui_selection_state()['feature_preset'] == 'sim-full'

        window.set_generic_args({'align_jaw': 'false'})
        assert window._preset_differences() == {'align_jaw': {'preset': 'true', 'current': 'false'}}
        assert 'align_jaw=false' in window._advanced_preset_label.text()
    finally:
        window.close()
        app.processEvents()


def test_real_robot_keeps_careful_features_off_unless_allowed(monkeypatch, tmp_path):
    app, window = _window(monkeypatch, tmp_path, rules=REAL_PADDING)
    try:
        window.remote_master_checkbox.setChecked(True)
        report = window.load_feature_preset('sim-full')
        assert sorted(report['careful_kept_off']) == ['cable_aware', 'padding']
        assert report['target_mismatch'] == 'preset sim-full is for the sim, the GUI is on the real'
        assert _values(window)['skip_untangle'] == 'false'   # back to the default mode
        assert _values(window)['box_padding'] == '0.01'

        report = window.apply_feature_selection(Selection({'cable_handling': 'hack'}, {'padding'}),
                                                ticked_careful={'padding'})
        assert report['refused'] == {'box_padding': {
            'value': '0.005', 'reason': 'the real robot keeps 1 cm until the supervised test'}}
    finally:
        window.close()
        app.processEvents()


def test_save_preset_and_load_it_back(monkeypatch, tmp_path):
    app, window = _window(monkeypatch, tmp_path)
    try:
        window.apply_feature_selection(Selection({'cable_handling': 'hack'}, {'jaw'}))
        saved = window.save_feature_preset('mine', 'only the jaw')
        data = yaml.safe_load((tmp_path / 'user_presets' / 'mine.yaml').read_text())
        assert data['features_on'] == ['jaw'] and data['modes'] == {'cable_handling': 'hack'}
        assert saved['name'] == 'mine' and window.feature_status()['preset'] == 'mine'
        window.load_feature_preset('sim-full')
        window.load_feature_preset('mine')
        assert _values(window)['narrow_side'] == 'false' and _values(window)['align_jaw'] == 'true'
    finally:
        window.close()
        app.processEvents()


def test_dialog_ticks_apply_through_the_main_window(monkeypatch, tmp_path):
    app, window = _window(monkeypatch, tmp_path)
    try:
        window._show_features_dialog()
        dialog = window._features_dialog
        assert dialog.target_label.text() == 'Simulation'
        assert dialog._feature_boxes['jaw'].isChecked()          # align_jaw starts at its first option, true
        dialog._feature_boxes['jaw'].setChecked(False)
        assert not dialog._feature_boxes['narrow'].isEnabled()    # needs its parent
        dialog._feature_boxes['jaw'].setChecked(True)
        assert dialog._feature_boxes['narrow'].isEnabled()
        dialog._feature_boxes['narrow'].setChecked(True)
        dialog._mode_buttons['cable_handling']['cable_aware'].setChecked(True)
        assert '2 on: 2 safe, 0 careful' in dialog.summary_label.text()
        report = dialog.apply()
        assert report['applied']['skip_untangle'] == 'true' and report['refused'] == {}
        assert _values(window)['narrow_side'] == 'true' and _values(window)['box_padding'] == '0.01'
        dialog.search.setText('padding')
        assert dialog._feature_boxes['padding'].parentWidget().isVisibleTo(dialog)
        assert not dialog._feature_boxes['jaw'].parentWidget().isVisibleTo(dialog)

        dialog.preset_combo.setCurrentIndex(dialog.preset_combo.findData('sim-full'))
        dialog._preset_chosen(dialog.preset_combo.currentIndex())
        assert dialog._feature_boxes['padding'].isChecked()
        dialog.apply()
        assert window.feature_status()['preset'] == 'sim-full'
        assert _values(window)['cable_monitor'] == 'true'
        dialog.close()
    finally:
        window.close()
        app.processEvents()


def test_remote_adapter_reads_features_and_loads_presets(monkeypatch, tmp_path):
    app, window = _window(monkeypatch, tmp_path)
    try:
        adapter = MainWindowRemoteAdapter(window)
        assert adapter.features()['available']
        report = adapter.set_preset({'name': 'sim-full'})
        assert report['preset'] == 'sim-full' and _values(window)['box_padding'] == '0.005'
        report = adapter.set_features({'off': ['padding'], 'on': [], 'modes': {'cable_handling': 'hack'}})
        assert _values(window)['box_padding'] == '0.01' and _values(window)['skip_untangle'] == 'false'
        assert _values(window)['align_jaw'] == 'true'   # untouched features keep their state
        assert report['preset'] is None
        saved = adapter.set_preset({'save': 'agent-run', 'description': 'saved by a test'})
        assert saved['name'] == 'agent-run'
    finally:
        window.close()
        app.processEvents()


def test_remote_control_and_preset_files_cannot_switch_careful_features_on_on_the_real_robot(monkeypatch, tmp_path):
    app, window = _window(monkeypatch, tmp_path)
    try:
        (tmp_path / 'demo_presets' / 'old.yaml').write_text(yaml.safe_dump(
            dict(PRESET, name='old', target='real', careful_on_real=['padding', 'cable_aware'])))
        logged = []
        monkeypatch.setattr(window, '_append_log_html', logged.append)
        MainWindow._load_features(window)
        assert any('old.yaml: careful_on_real is ignored' in line for line in logged)
        window.remote_master_checkbox.setChecked(True)
        adapter = MainWindowRemoteAdapter(window)

        report = adapter.set_preset({'name': 'old', 'allow_careful': ['padding']})
        assert sorted(report['careful_kept_off']) == ['cable_aware', 'padding']
        assert 'allow_careful is ignored' in report['ignored']
        assert _values(window)['box_padding'] == '0.01' and _values(window)['skip_untangle'] == 'false'

        report = adapter.set_features({'on': ['padding'], 'allow_careful': ['padding']})
        assert report['careful_kept_off'] == ['padding'] and 'ignored' in report
        assert _values(window)['box_padding'] == '0.01'

        # plain options are gated too, on both routes: the API body and a preset file's options
        report = adapter.set_features({'options': {'box_padding': '0.005', 'cable_monitor': 'true'}})
        assert report['refused']['box_padding']['value'] == '0.005'
        assert 'careful (padding)' in report['refused']['box_padding']['reason']
        assert _values(window)['box_padding'] == '0.01' and _values(window)['cable_monitor'] == 'true'
        (tmp_path / 'demo_presets' / 'sneaky.yaml').write_text(yaml.safe_dump(dict(
            PRESET, name='sneaky', target='real', modes={'cable_handling': 'hack'}, features_on=['jaw'],
            options={'box_padding': '0.005', 'skip_untangle': 'true'})))
        MainWindow._load_features(window)
        report = adapter.set_preset({'name': 'sneaky'})
        assert set(report['refused']) == {'box_padding', 'skip_untangle'}
        assert _values(window)['box_padding'] == '0.01' and _values(window)['skip_untangle'] == 'false'
    finally:
        window.close()
        app.processEvents()


def test_careful_tick_on_the_real_robot_asks_in_a_non_modal_top_level_window(monkeypatch, tmp_path):
    from PyQt5.QtCore import Qt
    from PyQt5.QtWidgets import QMessageBox
    app, window = _window(monkeypatch, tmp_path)
    try:
        window.remote_master_checkbox.setChecked(True)
        window._show_features_dialog()
        dialog = window._features_dialog
        assert dialog.parent() is None   # moves independently of the main window
        dialog._feature_boxes['padding'].setChecked(True)
        question = dialog._careful_question
        assert not dialog._feature_boxes['padding'].isChecked()   # off until the answer
        assert question.parent() is None and question.windowModality() == Qt.NonModal and question.isVisible()
        question.button(QMessageBox.No).click()
        assert not dialog._feature_boxes['padding'].isChecked()
        dialog._feature_boxes['padding'].setChecked(True)
        dialog._careful_question.button(QMessageBox.Yes).click()
        assert dialog._feature_boxes['padding'].isChecked() and 'padding' in dialog._allowed_careful
        dialog._mode_buttons['cable_handling']['cable_aware'].setChecked(True)
        assert dialog._mode_buttons['cable_handling']['hack'].isChecked()
        dialog._careful_question.button(QMessageBox.Yes).click()
        assert dialog._mode_buttons['cable_handling']['cable_aware'].isChecked()
        report = dialog.apply()
        assert report['careful_kept_off'] == [] and _values(window)['skip_untangle'] == 'true'
        dialog.close()
    finally:
        window.close()
        app.processEvents()
