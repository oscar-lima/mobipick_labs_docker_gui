import codecs
import sys
import time

from PyQt5.QtCore import QProcess, QProcessEnvironment
from PyQt5.QtWidgets import QApplication, QMainWindow

from mobipick_gui.log_files import LogFileManager
from mobipick_gui.output_flood import OutputFloodGuard
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


def make_process_tab(*, max_lines_per_second=1000, clock=None):
    tab = ProcessTab.__new__(ProcessTab)
    tab.key = 'sim'
    tab.parent = FakeParent()
    tab.output = FakeOutput()
    tab.notify_parent_finished = True
    tab._shutting_down = False
    tab._disk_log = None
    tab._guard_timer = None
    guard_kwargs = {'max_lines_per_second': max_lines_per_second}
    if clock is not None:
        guard_kwargs['clock'] = clock
    tab._flood_guard = OutputFloodGuard(**guard_kwargs)

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
    deadline = time.monotonic() + 2
    while tab.proc.state() != QProcess.NotRunning and time.monotonic() < deadline:
        app.processEvents()
        time.sleep(0.01)
    assert tab.proc.state() == QProcess.NotRunning

    app.processEvents()
    parent.deleteLater()


def test_repeated_lines_below_the_rate_are_shown_as_they_came():
    tab = make_process_tab()

    tab._append_raw(b'[INFO] Added plan for pipeline pick. Queue is now of size 1\n' * 3)
    tab._flush_output_pending(final=True)

    assert tab.output.entries == [
        (False, '[INFO] Added plan for pipeline pick. Queue is now of size 1\n')
    ] * 3


def test_a_flood_of_one_repeated_line_is_rate_limited_like_any_other():
    tab = make_process_tab(max_lines_per_second=100)

    tab._append_raw(b'still waiting for /query (49 s left)\n' * 5000)
    tab._flush_output_pending(final=True)

    shown = [e for e in tab.output.entries if e == (False, 'still waiting for /query (49 s left)\n')]
    assert len(shown) == 100
    is_html, notice = tab.output.entries[-1]
    assert is_html is True
    assert 'dropped 4900 lines' in notice


def test_one_megabyte_of_repeated_lines_is_processed_quickly_and_bounded():
    tab = make_process_tab()
    line = b'still waiting for /pick_pose_selector_node/pose_selector_class_query (49 s left)\n'
    payload = line * (1_000_000 // len(line) + 1)
    assert len(payload) >= 1_000_000

    started = time.perf_counter()
    for start in range(0, len(payload), 65536):
        tab._append_raw(payload[start:start + 65536])
    tab._flush_output_pending(final=True)
    elapsed = time.perf_counter() - started

    assert elapsed < 1.0, f'processing 1 MB took {elapsed:.2f} s'
    assert len(tab.output.entries) <= 1001   # one second of lines plus the drop notice


def test_distinct_lines_beyond_the_rate_are_dropped_with_a_notice():
    now = [0.0]
    tab = make_process_tab(max_lines_per_second=100, clock=lambda: now[0])

    tab._append_raw(''.join(f'line {i}\n' for i in range(50_000)).encode())
    assert len(tab.output.entries) == 100

    now[0] += 1.5
    tab._append_raw(b'after the flood\n')

    notices = [text for is_html, text in tab.output.entries if 'dropped' in text]
    assert len(notices) == 1
    assert 'dropped 49900 lines in the last second' in notices[0]
    assert 'rate limited' in notices[0]
    assert tab.output.entries[-1] == (False, 'after the flood\n')


def test_process_output_and_notices_are_written_to_the_tab_log_file(tmp_path):
    tab = make_process_tab()
    manager = LogFileManager(tmp_path / 'logs', session='20260101-120000')
    tab.parent._log_files = manager
    tab._open_disk_log(new_run=True)

    tab._append_raw(b'\x1b[32mgreen line\x1b[0m\n')
    tab._append_raw(b'[WARN] careful\n' * 3)
    tab.append_line_html('<i>&gt; roslaunch demo.launch</i>')
    tab._flush_output_pending(final=True)
    manager.close_all()

    files = sorted((tmp_path / 'logs' / '20260101-120000').glob('sim-*.log'))
    assert len(files) == 1
    assert files[0].read_text(encoding='utf-8') == (
        'green line\n'
        '[WARN] careful\n'
        '[WARN] careful\n'
        '[WARN] careful\n'
        '> roslaunch demo.launch\n'
    )


def _pump_until(app, predicate, timeout_s):
    deadline = time.monotonic() + timeout_s
    longest_tick = 0.0
    while not predicate() and time.monotonic() < deadline:
        tick_started = time.perf_counter()
        app.processEvents()
        longest_tick = max(longest_tick, time.perf_counter() - tick_started)
        time.sleep(0.005)
    return longest_tick


def test_real_process_flood_keeps_the_event_loop_responsive_and_widget_bounded():
    app = QApplication.instance() or QApplication([])
    parent = QMainWindow()
    parent._build_process_environment = (
        lambda _env: QProcessEnvironment.systemEnvironment()
    )
    parent._log_cmd = lambda _command: None
    parent._command_log_color = '#4da3ff'
    parent._filter_terminal_escapes = lambda data: data
    parent._collapse_carriage_returns = lambda data: data
    tab = ProcessTab(
        'flood-test',
        'Flood test',
        parent,
        False,
        notify_parent_finished=False,
    )
    script = (
        'import sys\n'
        'w = sys.stdout.write\n'
        'for _ in range(200000):\n'
        "    w('still waiting for /pose_selector_class_query (49 s left)\\n')\n"
        'for i in range(200000):\n'
        "    w(f'distinct line {i}\\n')\n"
        "w('done\\n')\n"
    )
    tab.start_program(sys.executable, ['-u', '-c', script])
    assert tab.proc.waitForStarted(2000)

    longest_tick = _pump_until(
        app,
        lambda: tab.proc.state() == QProcess.NotRunning
        and tab.output.pending_count() == 0,
        timeout_s=30,
    )
    assert tab.proc.state() == QProcess.NotRunning
    assert longest_tick < 0.5, f'event loop blocked for {longest_tick:.2f} s'

    text = tab.output.toPlainText()
    lines = text.splitlines()
    assert 1 <= lines.count('still waiting for /pose_selector_class_query (49 s left)') <= 1000 * 30
    assert 'rate limited' in text
    assert len(lines) < 1000 * 30 + 100
    parent.deleteLater()
    app.processEvents()
