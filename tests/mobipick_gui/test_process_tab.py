import codecs
import sys

from PyQt5.QtCore import QProcess, QProcessEnvironment
from PyQt5.QtWidgets import QApplication, QMainWindow

from mobipick_gui.process_tab import ProcessTab, ROS_WARNING_COLOR


class FakeOutput:
    def __init__(self):
        self.entries = []

    def enqueue(self, is_html, text):
        self.entries.append((is_html, text))


class FakeParent:
    @staticmethod
    def _filter_terminal_escapes(data):
        return data

    @staticmethod
    def _collapse_carriage_returns(data):
        return data

    @staticmethod
    def _prepare_tab_for_origin(*_args):
        return None


def make_process_tab():
    tab = ProcessTab.__new__(ProcessTab)
    tab.key = 'sim'
    tab.parent = FakeParent()
    tab.output = FakeOutput()
    tab.notify_parent_finished = True

    tab._output_decoder = codecs.getincrementaldecoder('utf-8')(
        errors='replace'
    )
    tab._output_pending = ''
    return tab


def test_warning_split_across_process_reads_is_always_yellow():
    tab = make_process_tab()

    tab._append_raw(b'[WA')
    assert tab.output.entries == []

    tab._append_raw(b'RN] [14:26:56] [/pose_selector]: Clearing scene\n')

    assert len(tab.output.entries) == 1
    is_html, rendered = tab.output.entries[0]
    assert is_html is True
    assert f'color:{ROS_WARNING_COLOR}' in rendered
    assert '[/pose_selector]: Clearing scene' in rendered


def test_ansi_warning_split_across_process_reads_keeps_yellow_color():
    tab = make_process_tab()

    tab._append_raw(b'\x1b[3')
    tab._append_raw(b'3m[WARN] warning from ROS\x1b[0m\n')

    assert len(tab.output.entries) == 1
    is_html, rendered = tab.output.entries[0]
    assert is_html is True
    assert 'color:#f1fa8c' in rendered
    assert '[WARN] warning from ROS' in rendered


def test_incomplete_final_line_is_flushed_when_process_finishes():
    tab = make_process_tab()
    tab._append_raw(b'[WARN] final warning')

    tab._flush_output_pending(final=True)

    assert len(tab.output.entries) == 1
    assert f'color:{ROS_WARNING_COLOR}' in tab.output.entries[0][1]


def test_stop_for_shutdown_reaps_process_and_disables_callbacks():
    app = QApplication.instance() or QApplication([])
    parent = QMainWindow()
    parent._build_process_environment = (
        lambda _env: QProcessEnvironment.systemEnvironment()
    )
    parent._log_cmd = lambda _command: None
    parent._command_log_color = '#4da3ff'
    output = FakeOutput()
    tab = ProcessTab(
        'shutdown-test',
        'Shutdown test',
        parent,
        False,
        output=output,
        notify_parent_finished=False,
    )
    tab.start_program(sys.executable, ['-c', 'import time; time.sleep(10)'])
    assert tab.proc.waitForStarted(1000)

    assert tab.stop_for_shutdown()
    assert tab.proc.state() == QProcess.NotRunning

    app.processEvents()
    parent.deleteLater()
