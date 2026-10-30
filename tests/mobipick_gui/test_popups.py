import os
from types import SimpleNamespace

import pytest

os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')

from PyQt5.QtCore import Qt
from PyQt5.QtWidgets import QApplication, QDialog, QMessageBox, QWidget

from mobipick_gui import popups
from mobipick_gui.remote_adapter import MainWindowRemoteAdapter


_app = None


def app():
    global _app
    _app = QApplication.instance() or QApplication([])
    return _app


def close_open_boxes():
    for box in popups.open_boxes():
        box.close()
    app().processEvents()


@pytest.fixture(autouse=True)
def no_window_left_open_by_earlier_tests():
    # an earlier test may leave a modal window (the exit notice) or a box open; either changes what opens where
    close_open_boxes()
    for widget in app().topLevelWidgets():
        if widget.isVisible() and widget.isModal():
            widget.hide()
    yield
    close_open_boxes()


def test_a_message_box_has_no_parent_and_does_not_block():
    app()
    main = QWidget()
    box = popups.warning(main, 'Toolbar Buttons', 'All argument slots are used.')

    # top-level and non-modal: GNOME does not attach it to the main window, and the call returned at once
    assert box.parent() is None
    assert not box.isModal()
    assert box.isVisible()
    assert box.icon() == QMessageBox.Warning
    assert box.text() == 'All argument slots are used.'
    assert popups.open_boxes() == [box]

    box.close()
    app().processEvents()
    assert popups.open_boxes() == []


def test_a_message_box_over_a_modal_dialog_opens_on_that_dialog(monkeypatch):
    # a box without a parent would be blocked by the modal dialog and could not be clicked, so it opens on the dialog
    # and waits for its answer, as before
    app()
    dialog = QDialog()
    dialog.setWindowModality(Qt.ApplicationModal)
    dialog.show()
    assert app().activeModalWidget() is dialog
    waited = []
    monkeypatch.setattr(popups.QMessageBox, 'exec_', lambda box: waited.append((box.parent(), box.text())) or 0)

    popups.information(None, 'Images', 'Image removed.')

    assert waited == [(dialog, 'Image removed.')]
    assert popups.open_boxes() == []
    dialog.hide()


def test_the_remote_control_lists_and_dismisses_an_open_box():
    app()
    adapter = MainWindowRemoteAdapter(SimpleNamespace())
    assert adapter.active_dialog() is None
    popups.information(None, 'Save Logs', 'Saved 3 log file(s).')

    info = adapter.active_dialog()
    assert info['title'] == 'Save Logs' and info['text'] == 'Saved 3 log file(s).'
    assert info['buttons'] == ['OK'] and info['modal'] is False

    result = adapter.dismiss_dialog('OK')
    app().processEvents()
    assert result['dismissed'] is True
    assert popups.open_boxes() == []
    assert adapter.dismiss_dialog('OK') == {'dismissed': False, 'reason': 'no open dialog'}
    close_open_boxes()


def test_the_remote_control_answers_the_newest_box_first():
    app()
    adapter = MainWindowRemoteAdapter(SimpleNamespace())
    popups.warning(None, 'First', 'one')
    popups.critical(None, 'Second', 'two')

    assert adapter.active_dialog()['title'] == 'Second'
    adapter.dismiss_dialog('OK')
    app().processEvents()
    assert adapter.active_dialog()['title'] == 'First'
    close_open_boxes()


def test_a_question_waits_for_its_answer_in_a_box_without_a_parent(monkeypatch):
    # the caller needs the answer, so the box stays modal, but as a window of its own
    app()
    asked = []

    def answer(box):
        asked.append((box.parent(), box.icon(), box.text()))
        return QMessageBox.Yes

    monkeypatch.setattr(popups.QMessageBox, 'exec_', answer)
    assert popups.question(QWidget(), 'Remove Workspace', 'Remove gpt_ws?') == QMessageBox.Yes
    assert popups.ask(QMessageBox.Warning, 'Clean stale ROS nodes', 'Continue?') == QMessageBox.Yes
    assert asked == [
        (None, QMessageBox.Question, 'Remove gpt_ws?'),
        (None, QMessageBox.Warning, 'Continue?'),
    ]


def test_a_launch_option_rule_notice_opens_in_its_own_window():
    from mobipick_gui.main_window import MainWindow

    app()
    MainWindow._show_rule_popup(SimpleNamespace(), 'Launch options', 'gzclient needs the sim.')

    box, = popups.open_boxes()
    assert box.parent() is None and not box.isModal() and box.isVisible()
    assert box.text() == 'gzclient needs the sim.'
