import os
import time

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


def make_shown_widget():
    # Qt lays out only a shown text edit; the cost under test is that layout
    widget = make_widget()
    widget.resize(1000, 600)
    widget.show()
    _app.processEvents()
    return widget


def _flush_seconds(widget):
    started = time.perf_counter()
    widget._flush_tick()
    return time.perf_counter() - started


def test_each_html_line_is_a_block_of_its_own():
    widget = make_widget()
    for i in range(50):
        widget.enqueue(True, f'<span style="color:#f1fa8c">html line {i:02d}</span><br>')
    widget._flush()

    assert widget.document().blockCount() >= 50
    lines = widget.toPlainText().splitlines()
    assert lines[:2] == ['html line 00', 'html line 01']
    assert lines[49] == 'html line 49'


def test_a_coloured_line_does_not_colour_the_next_plain_line():
    widget = make_widget()
    widget.enqueue(True, '<span style="color:#ff0000">red</span><br>')
    widget.enqueue(False, 'plain\n')
    widget._flush()

    block = widget.document().findBlock(widget.toPlainText().index('plain'))
    fragment = block.begin().fragment()
    assert fragment.text() == 'plain'
    assert fragment.charFormat().foreground().color().name() != '#ff0000'


def test_a_flush_tick_of_coloured_lines_stays_short():
    # a GUI thread busy for over a second fails every remote API call
    widget = make_shown_widget()
    widget._max_entries_per_flush = 600
    for i in range(600):
        widget.enqueue(True, f'<span style="color:#f1fa8c">warning {i:04d}: goal tolerance not met</span><br>')

    assert _flush_seconds(widget) < 1.0


def test_a_flush_tick_into_a_full_tab_stays_short():
    widget = make_shown_widget()
    widget.document().setMaximumBlockCount(2000)
    widget._max_entries_per_flush = 2000
    for i in range(2000):
        widget.enqueue(False, f'line {i:05d}: move_group plan request received\n')
    widget._flush_tick()
    for i in range(2000, 4000):
        widget.enqueue(False, f'line {i:05d}: move_group plan request received\n')

    assert _flush_seconds(widget) < 1.0
    text = widget.toPlainText()
    assert 'line 03999' in text
    assert 'line 01000' not in text
