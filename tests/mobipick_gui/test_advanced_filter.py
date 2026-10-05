import os

os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')

from PyQt5.QtCore import Qt
from PyQt5.QtTest import QTest

from mobipick_gui.main_window import MainWindow
from test_remote_ros_master import _create_window


def _shown_names(window):
    return [label.text() for label, *_rest in window._advanced_all_rows if not label.isHidden()]


def test_search_filters_advanced_options_and_keeps_hidden_values(monkeypatch, tmp_path):
    app, window = _create_window(monkeypatch, tmp_path)
    entry = {'key': 'demo', 'arg_1_name': 'use_anygrasp', 'arg_1_options': ['false', 'true'],
             'arg_1_advanced': True, 'arg_1_description': 'Grasp open-set objects',
             'arg_2_name': 'cable_monitor', 'arg_2_options': ['true', 'false'], 'arg_2_advanced': True,
             'arg_3_name': 'world_name', 'arg_3_options': ['moelk_tables', 'cic_tables'], 'arg_3_advanced': True}
    window._button_layout = [entry]
    window._refresh_generic_arg_controls()
    window._generic_arg_inputs[1].setCurrentIndex(1)
    MainWindow._show_advanced_launch_dialog(window)
    search = window._advanced_filter
    try:
        assert not search.isHidden()
        assert _shown_names(window) == ['use_anygrasp:', 'cable_monitor:', 'world_name:']
        search.setText('CABLE')   # name, case-insensitive
        assert _shown_names(window) == ['cable_monitor:']
        search.setText('open-set')   # description
        assert _shown_names(window) == ['use_anygrasp:']
        search.setText('moelk')   # current value
        assert _shown_names(window) == ['world_name:']
        search.setText('nothing such')
        assert _shown_names(window) == []
        assert not window._advanced_no_match_label.isHidden()
        assert window._generic_arg_inputs[1].currentText() == 'true'   # hidden rows keep their values
        QTest.keyClick(search, Qt.Key_Escape)
        assert search.text() == ''
        assert len(_shown_names(window)) == 3
        assert window.advanced_launch_dialog.isVisible()   # the first Esc only clears
    finally:
        window.advanced_launch_dialog.close()
        window.close()
        app.processEvents()
