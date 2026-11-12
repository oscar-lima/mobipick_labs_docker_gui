import os

os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')

from PyQt5.QtWidgets import QGroupBox

from mobipick_gui.main_window import MainWindow
from mobipick_gui.option_groups import OTHER_TITLE, load_option_groups, parse_option_groups
from test_remote_ros_master import _create_window

ENTRY = {
    'key': 'demo',
    'arg_1_name': 'voice_agent', 'arg_1_options': ['off', 'cedar'], 'arg_1_advanced': True,
    'arg_2_name': 'voice_ack', 'arg_2_options': ['off', 'always'], 'arg_2_advanced': True,
    'arg_3_name': 'cable_monitor', 'arg_3_options': ['true', 'false'], 'arg_3_advanced': True,
    'arg_4_name': 'world_name', 'arg_4_options': ['a', 'b'], 'arg_4_advanced': True,
}
GROUPS = {'groups': [{'title': 'Voice agent', 'options': ['voice_agent', 'voice_ack']},
                     {'title': 'Cable', 'options': ['cable_monitor']},
                     {'title': 'Empty', 'options': ['nothing_such']}]}


def test_parse_keeps_order_reports_problems_and_names_other_last():
    groups = parse_option_groups({'groups': [{'title': 'A', 'options': ['x', 'y']}, {'title': 'B', 'options': ['y', 'z']},
                                             {'title': OTHER_TITLE, 'options': ['q']}, {'title': 'C'}]})
    assert groups.groups == [('A', ('x', 'y')), ('B', ('z',))]
    assert groups.title_of('z') == 'B' and groups.title_of('unknown') == OTHER_TITLE
    assert groups.order() == ['A', 'B', OTHER_TITLE]
    assert len(groups.errors) == 3
    assert not parse_option_groups(None) and not parse_option_groups({'x': 1}).groups


def test_load_reads_the_file_beside_the_profile(tmp_path):
    (tmp_path / 'p.yaml').write_text('[]')
    assert not load_option_groups(tmp_path / 'p.yaml') and not load_option_groups(None)
    (tmp_path / 'p_option_groups.yaml').write_text('groups:\n- {title: V, options: [a]}\n')
    assert load_option_groups(tmp_path / 'p.yaml').groups == [('V', ('a',))]
    (tmp_path / 'p_option_groups.yaml').write_text('groups: [')
    assert load_option_groups(tmp_path / 'p.yaml').errors


def _no_row_is_a_window(window):
    return not [w for label, field, *_ in window._advanced_all_rows for w in (label, field) if w.isWindow()]


def _boxes(window):
    return {box.objectName(): box for box in window.advanced_launch_dialog.findChildren(QGroupBox)
            if box.objectName().startswith('advanced_group_')}


def _titles(window):
    return sorted(name.removeprefix('advanced_group_') for name in _boxes(window))


def test_options_sit_in_group_boxes_search_hides_empty_boxes_and_values_stay(monkeypatch, tmp_path):
    app, window = _create_window(monkeypatch, tmp_path)
    try:
        window._option_groups = parse_option_groups(GROUPS)
        window._button_layout = [ENTRY]
        window._refresh_generic_arg_controls()
        window._generic_arg_inputs[1].setCurrentIndex(1)   # cedar
        MainWindow._show_advanced_launch_dialog(window)
        assert _titles(window) == ['Cable', OTHER_TITLE, 'Voice agent']   # the empty group has no box
        voice = _boxes(window)['advanced_group_Voice agent']
        assert voice.title() == 'Voice agent (2)'
        assert {w.text() for w in voice.findChildren(type(window._advanced_all_rows[0][0])) if w.text().endswith(':')} \
            >= {'voice_agent:', 'voice_ack:'}
        assert _no_row_is_a_window(window)
        window._advanced_filter.setText('cable')
        assert _titles(window) == ['Cable']
        assert _no_row_is_a_window(window)
        window._advanced_filter.setText('')
        assert _titles(window) == ['Cable', OTHER_TITLE, 'Voice agent']
        assert _no_row_is_a_window(window)
        assert window._generic_arg_inputs[1].currentText() == 'cedar'   # layout only: values untouched
        window._refresh_generic_arg_controls()   # a rebuild (profile reload) builds the boxes again
        assert _titles(window) == ['Cable', OTHER_TITLE, 'Voice agent']
    finally:
        window.advanced_launch_dialog.close()
        window.close()
        app.processEvents()


def test_without_a_groups_file_the_dialog_keeps_the_plain_list(monkeypatch, tmp_path):
    app, window = _create_window(monkeypatch, tmp_path)
    try:
        window._option_groups = parse_option_groups(None)
        window._button_layout = [ENTRY]
        window._refresh_generic_arg_controls()
        MainWindow._show_advanced_launch_dialog(window)
        assert _boxes(window) == {}
        window._advanced_filter.setText('cable')
        window._advanced_filter.setText('')
        assert _no_row_is_a_window(window)   # the plain list reparents nothing
        assert [label.text() for label, *_ in window._advanced_all_rows if not label.isHidden()] == [
            'voice_agent:', 'voice_ack:', 'cable_monitor:', 'world_name:']
    finally:
        window.advanced_launch_dialog.close()
        window.close()
        app.processEvents()


def test_the_groups_file_problems_are_logged_once(monkeypatch, tmp_path):
    app, window = _create_window(monkeypatch, tmp_path)
    try:
        logged = []
        monkeypatch.setattr(window, '_append_log_html', logged.append)
        window._option_groups = parse_option_groups({'groups': [{'title': 'A', 'options': ['x']},
                                                                 {'title': 'B', 'options': ['x']}]})
        window._option_groups_reported = False
        window._button_layout = [ENTRY]
        window._refresh_generic_arg_controls()
        window._advanced_filter.setText('cable')
        window._advanced_filter.setText('')
        assert len([line for line in logged if 'option groups' in line]) == 1
    finally:
        window.advanced_launch_dialog.close()
        window.close()
        app.processEvents()
