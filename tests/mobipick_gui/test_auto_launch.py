from types import MethodType, SimpleNamespace

from mobipick_gui.main_window import MainWindow


def _auto_launch_harness():
    events = []
    harness = SimpleNamespace(
        _auto_launch_running=True,
        _auto_launch_stopping=False,
        _auto_launch_active_keys=['roscore'],
        _roscore_stopping=False,
        _cancel_auto_launch_timers=lambda: events.append('cancel_timers'),
        _cancel_recording_schedule=lambda: events.append('cancel_recording'),
        _auto_launch_shutdown_order=lambda: ['roscore'],
        _recording_is_active=lambda: False,
        _stop_screen_recording=lambda **_kwargs: None,
        _auto_launch_start_text=lambda: 'Auto Launch',
        _flush_ui_events=lambda: events.append('flush'),
        set_auto_launch_visual=lambda state, text, enabled: events.append(
            ('visual', state, text, enabled)
        ),
    )
    harness._stop_auto_launch_stack = MethodType(
        MainWindow._stop_auto_launch_stack,
        harness,
    )
    harness._finalize_auto_launch_stop = MethodType(
        MainWindow._finalize_auto_launch_stop,
        harness,
    )
    return harness, events


def test_auto_launch_stop_ignores_recursive_roscore_stop():
    harness, events = _auto_launch_harness()

    def _trigger(key, *, target_running):
        events.append(('trigger', key, target_running))
        harness._stop_auto_launch_stack()

    harness._trigger_auto_launch_step = _trigger

    harness._stop_auto_launch_stack()

    assert events.count('cancel_timers') == 1
    assert events.count(('trigger', 'roscore', False)) == 1
    assert ('visual', 'yellow', 'Stopping Auto Launch...', False) in events
    assert events[-1] == ('visual', 'red', 'Auto Launch', True)
    assert harness._auto_launch_stopping is False


def test_auto_launch_stop_waits_for_roscore_shutdown_to_finalize():
    harness, events = _auto_launch_harness()

    def _trigger(_key, *, target_running):
        events.append(('trigger', _key, target_running))
        harness._roscore_stopping = True

    harness._trigger_auto_launch_step = _trigger

    harness._stop_auto_launch_stack()

    assert events[-1] == ('trigger', 'roscore', False)
    assert harness._auto_launch_stopping is True
    assert ('visual', 'red', 'Auto Launch', True) not in events

    harness._finalize_auto_launch_stop()

    assert events[-1] == ('visual', 'red', 'Auto Launch', True)
    assert harness._auto_launch_stopping is False
    assert harness._auto_launch_active_keys == []
