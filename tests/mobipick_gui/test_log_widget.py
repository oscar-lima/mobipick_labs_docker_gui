import os

os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')

from PyQt5.QtWidgets import QApplication

from mobipick_gui.log_widget import LogTextEdit


_app = None


def make_widget():
    global _app
    _app = QApplication.instance() or QApplication([])
    return LogTextEdit()


def test_flush_tick_renders_a_bounded_number_of_entries():
    widget = make_widget()
    widget._max_entries_per_flush = 10
    for i in range(25):
        widget.enqueue(False, f'line {i}\n')

    widget._flush_tick()
    assert widget.pending_count() == 15
    assert widget._flush_timer.isActive()

    widget._flush()
    assert widget.pending_count() == 0
    assert widget.toPlainText().count('line ') == 25


def test_pending_buffer_drops_oldest_entries_with_a_notice():
    widget = make_widget()
    widget._max_pending_entries = 5
    for i in range(12):
        widget.enqueue(False, f'line {i}\n')

    assert widget.pending_count() == 5
    widget._flush()
    text = widget.toPlainText()
    assert 'dropped 7 buffered lines' in text
    assert 'line 0' not in text
    assert 'line 11' in text


def test_document_is_trimmed_to_max_characters_also_for_html_lines():
    widget = make_widget()
    widget._max_characters = 2000
    for i in range(500):
        widget.enqueue(True, f'<span style="color:#f1fa8c">html line {i:04d}</span><br>')
    widget._flush()

    assert widget.document().characterCount() <= 2000
    text = widget.toPlainText()
    assert 'html line 0499' in text
    assert 'html line 0000' not in text
