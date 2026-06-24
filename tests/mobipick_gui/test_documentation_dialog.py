import os

os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')

from PyQt5.QtWidgets import QApplication

from mobipick_gui.config import CONFIG
from mobipick_gui.documentation_dialog import DocumentationDialog
from mobipick_gui.main_window import MainWindow, _about_details_html


def _find_menu_action(menu, text):
    for action in menu.actions():
        if action.text() == text:
            return action
        submenu = action.menu()
        if submenu is not None:
            found = _find_menu_action(submenu, text)
            if found is not None:
                return found
    return None


def test_documentation_dialog_renders_markdown_and_highlights_search(tmp_path):
    markdown_path = tmp_path / 'gui_user_documentation.md'
    markdown_path.write_text(
        '# User Guide\n\nStart RViz from the toolbar.\nStop RViz when done.\n',
        encoding='utf-8',
    )

    app = QApplication.instance() or QApplication([])
    dialog = DocumentationDialog(markdown_path)

    assert 'Start RViz from the toolbar.' in dialog.viewer.toPlainText()

    dialog.search_input.setText('RViz')
    dialog.find_next()

    assert dialog.match_label.text() == '2 matches'
    assert dialog.viewer.textCursor().selectedText() == 'RViz'

    dialog.deleteLater()
    app.processEvents()


def test_about_dialog_details_show_maintainer_contact():
    details = _about_details_html()

    assert 'Maintainer:<br>Oscar Lima<br>' in details
    assert 'href="mailto:oscar.lima@dfki.de"' in details
    assert '>oscar.lima@dfki.de</a>' in details
    assert 'GUI source code:<br>' in details
    assert (
        'href="https://github.com/oscar-lima/mobipick_labs_docker_gui"'
        in details
    )


def test_help_menu_documentation_opens_dialog(tmp_path, monkeypatch):
    monkeypatch.setenv(
        'MOBIPICK_WORKSPACE_CONFIG',
        str(tmp_path / 'workspaces.yaml'),
    )
    monkeypatch.setattr(
        MainWindow,
        '_discover_filtered_image_records',
        lambda self: ([{'ref': CONFIG['images']['default']}], None),
    )
    monkeypatch.setattr(
        MainWindow,
        'update_sim_status_from_poll',
        lambda self, force=False: None,
    )

    app = QApplication.instance() or QApplication([])
    window = MainWindow(verbosity=1)
    window.poll_timer.stop()
    window._sigint_timer.stop()

    help_menu = next(
        action.menu()
        for action in window.menuBar().actions()
        if action.text() == 'Help'
    )
    action = _find_menu_action(help_menu, 'Documentation')
    assert action is not None

    action.trigger()
    app.processEvents()

    assert window._documentation_dialog is not None
    assert 'Mobipick Labs Docker GUI User Documentation' in (
        window._documentation_dialog.viewer.toPlainText()
    )

    window._documentation_dialog.close()
    window.deleteLater()
    app.processEvents()
