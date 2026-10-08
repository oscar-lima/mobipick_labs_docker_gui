from mobipick_gui.output_flood import LINE, NOTICE, OutputFloodGuard


class FakeClock:
    def __init__(self):
        self.now = 0.0

    def __call__(self):
        return self.now


def make_guard(max_lines_per_second=1000):
    clock = FakeClock()
    return OutputFloodGuard(max_lines_per_second=max_lines_per_second, clock=clock), clock


def test_distinct_lines_pass_through_unchanged():
    guard, _ = make_guard()
    assert guard.feed_many(['a\n', 'b\n']) == [(LINE, 'a\n'), (LINE, 'b\n')]


def test_identical_lines_collapse_and_report_once_on_change():
    guard, _ = make_guard()
    events = guard.feed_many(['same\n'] * 4 + ['other\n'])
    assert events == [
        (LINE, 'same\n'),
        (NOTICE, '... (previous line repeated 3 times)'),
        (LINE, 'other\n'),
    ]
    assert guard.total_collapsed == 3


def test_interim_repeat_notice_at_most_once_per_second():
    guard, clock = make_guard()
    assert guard.feed_many(['x\n'] * 10) == [(LINE, 'x\n')]
    clock.now = 1.0
    events = guard.feed_many(['x\n'] * 5)
    assert events == [(NOTICE, '... (previous line repeated 10 times so far)')]
    clock.now = 1.5
    assert guard.feed_many(['x\n'] * 5) == []
    assert guard.has_pending
    assert guard.flush(final=True) == [
        (NOTICE, '... (previous line repeated 19 times)')
    ]
    assert not guard.has_pending


def test_flush_without_final_reports_only_after_a_second():
    guard, clock = make_guard()
    guard.feed_many(['x\n', 'x\n'])
    assert guard.flush() == []
    clock.now = 1.0
    assert guard.flush() == [(NOTICE, '... (previous line repeated 1 times so far)')]
    assert guard.flush() == []


def test_rate_limit_drops_lines_and_reports_the_count_once_per_second():
    guard, clock = make_guard(max_lines_per_second=3)
    events = guard.feed_many([f'{i}\n' for i in range(10)])
    assert events == [(LINE, '0\n'), (LINE, '1\n'), (LINE, '2\n')]
    assert guard.total_dropped == 7
    assert guard.has_pending

    clock.now = 1.0
    events = guard.feed('next\n')
    assert events[0][0] == NOTICE
    assert 'dropped 7 lines in the last second' in events[0][1]
    assert 'rate limited (3 lines/s shown)' in events[0][1]
    assert events[1] == (LINE, 'next\n')


def test_repeats_do_not_count_against_the_rate():
    guard, _ = make_guard(max_lines_per_second=2)
    events = guard.feed_many(['a\n'] * 1000 + ['b\n'])
    assert [e for e in events if e[0] == LINE] == [(LINE, 'a\n'), (LINE, 'b\n')]
    assert guard.total_dropped == 0


def test_reset_forgets_the_last_line():
    guard, _ = make_guard()
    guard.feed('a\n')
    guard.reset()
    assert guard.feed('a\n') == [(LINE, 'a\n')]
