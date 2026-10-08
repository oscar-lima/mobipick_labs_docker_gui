from mobipick_gui.output_flood import LINE, NOTICE, OutputFloodGuard


class FakeClock:
    def __init__(self):
        self.now = 0.0

    def __call__(self):
        return self.now


def make_guard(max_lines_per_second=1000, min_repeats=3, interval=10.0):
    clock = FakeClock()
    guard = OutputFloodGuard(
        max_lines_per_second=max_lines_per_second, min_repeats=min_repeats,
        repeat_notice_interval_s=interval, clock=clock,
    )
    return guard, clock


def test_distinct_lines_pass_through_unchanged():
    guard, _ = make_guard()
    assert guard.feed_many(['a\n', 'b\n']) == [(LINE, 'a\n'), (LINE, 'b\n')]


def test_a_few_repeats_are_shown_and_the_rest_collapses_with_a_notice_on_change():
    guard, _ = make_guard()
    events = guard.feed_many(['same\n'] * 6 + ['other\n'])
    assert events == [
        (LINE, 'same\n'),
        (LINE, 'same\n'),
        (LINE, 'same\n'),
        (NOTICE, '... (previous line repeated 3 more times)'),
        (LINE, 'other\n'),
    ]
    assert guard.total_collapsed == 3


def test_up_to_min_repeats_copies_pass_through_without_any_notice():
    guard, _ = make_guard()
    events = guard.feed_many(['twice\n'] * 2 + ['thrice\n'] * 3 + ['end\n'])
    assert events == [(LINE, 'twice\n')] * 2 + [(LINE, 'thrice\n')] * 3 + [(LINE, 'end\n')]
    assert guard.flush(final=True) == []
    assert guard.total_collapsed == 0


def test_interim_repeat_notice_at_most_once_per_interval():
    guard, clock = make_guard()
    assert guard.feed_many(['x\n'] * 10) == [(LINE, 'x\n')] * 3
    clock.now = 10.0
    events = guard.feed_many(['x\n'] * 5)
    assert events == [(NOTICE, '... (previous line repeated 8 more times so far)')]
    clock.now = 15.0
    assert guard.feed_many(['x\n'] * 5) == []
    assert guard.has_pending
    assert guard.flush(final=True) == [
        (NOTICE, '... (previous line repeated 17 more times)')
    ]
    assert not guard.has_pending


def test_flush_without_final_reports_only_after_the_interval():
    guard, clock = make_guard()
    guard.feed_many(['x\n'] * 4)
    assert guard.flush() == []
    clock.now = 10.0
    assert guard.flush() == [(NOTICE, '... (previous line repeated 1 more times so far)')]
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


def test_hidden_repeats_do_not_count_against_the_rate():
    guard, _ = make_guard(max_lines_per_second=4)
    events = guard.feed_many(['a\n'] * 1000 + ['b\n'])
    assert [e for e in events if e[0] == LINE] == [(LINE, 'a\n')] * 3 + [(LINE, 'b\n')]
    assert guard.total_dropped == 0


def test_reset_forgets_the_last_line():
    guard, _ = make_guard()
    guard.feed('a\n')
    guard.reset()
    assert guard.feed('a\n') == [(LINE, 'a\n')]
