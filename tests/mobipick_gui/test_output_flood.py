from mobipick_gui.output_flood import LINE, NOTICE, OutputFloodGuard


class FakeClock:
    def __init__(self):
        self.now = 0.0

    def __call__(self):
        return self.now


def make_guard(max_lines_per_second=1000):
    clock = FakeClock()
    return OutputFloodGuard(max_lines_per_second=max_lines_per_second, clock=clock), clock


def test_lines_pass_through_unchanged():
    guard, _ = make_guard()
    assert guard.feed_many(['a\n', 'b\n']) == [(LINE, 'a\n'), (LINE, 'b\n')]


def test_identical_lines_are_shown_as_they_came():
    # ROS nodes print the same text for separate events (one per planning pipeline, per
    # costmap layer, per grasp thread); hiding or counting them would hamper debugging
    guard, _ = make_guard()
    events = guard.feed_many(['same\n'] * 4 + ['other\n'])
    assert events == [(LINE, 'same\n')] * 4 + [(LINE, 'other\n')]
    assert guard.flush(final=True) == []
    assert not guard.has_pending


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


def test_flush_without_final_reports_the_drop_only_after_a_second():
    guard, clock = make_guard(max_lines_per_second=1)
    guard.feed_many(['a\n', 'b\n'])
    assert guard.flush() == []
    clock.now = 1.0
    events = guard.flush()
    assert len(events) == 1 and 'dropped 1 lines' in events[0][1]
    assert guard.flush() == []
    assert not guard.has_pending


def test_final_flush_reports_a_pending_drop_at_once():
    guard, _ = make_guard(max_lines_per_second=1)
    guard.feed_many(['a\n', 'b\n', 'c\n'])
    events = guard.flush(final=True)
    assert len(events) == 1 and 'dropped 2 lines' in events[0][1]


def test_reset_forgets_the_window():
    guard, _ = make_guard(max_lines_per_second=1)
    guard.feed_many(['a\n', 'b\n'])
    guard.reset()
    assert guard.feed('c\n') == [(LINE, 'c\n')]
    assert not guard.has_pending
