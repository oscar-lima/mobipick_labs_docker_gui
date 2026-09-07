import codecs

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
