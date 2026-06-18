from types import MethodType, SimpleNamespace

from PyQt5.QtCore import QProcess

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


def test_record_auto_launch_uncheck_stops_active_recording():
    events = []
    harness = SimpleNamespace(
        _recording_remember_output_dir=True,
        _cancel_recording_schedule=lambda: events.append('cancel_recording'),
        _recording_is_active=lambda: True,
        _stop_screen_recording=lambda **kwargs: events.append(
            ('stop_recording', kwargs)
        ),
        _log_info=lambda message: events.append(('log', message)),
    )
    harness._on_record_checkbox_toggled = MethodType(
        MainWindow._on_record_checkbox_toggled,
        harness,
    )

    harness._on_record_checkbox_toggled(False)

    assert events == [
        'cancel_recording',
        (
            'stop_recording',
            {
                'save_logs': True,
                'reason': 'Record Auto Launch unchecked; stopping recording',
            },
        ),
    ]


def test_recording_stop_requests_ffmpeg_clean_quit():
    events = []

    class FakeProcess:
        def state(self):
            return QProcess.Running

        def write(self, data):
            events.append(('write', data))

        def closeWriteChannel(self):
            events.append('close_stdin')

    session = {}
    harness = SimpleNamespace(
        _recording_start_timer=None,
        _recording_session=session,
        _recording_proc=FakeProcess(),
        _cancel_recording_schedule=lambda: events.append('cancel_recording'),
        _log_info=lambda message: events.append(('log', message)),
        _request_ffmpeg_stop=MethodType(
            MainWindow._request_ffmpeg_stop,
            SimpleNamespace(
                _log_info=lambda message: events.append(('log', message)),
                _append_gui_html=lambda key, text: events.append((key, text)),
            ),
        ),
    )
    harness._stop_screen_recording = MethodType(
        MainWindow._stop_screen_recording,
        harness,
    )
    harness._terminate_recording_if_running = lambda _proc: None

    harness._stop_screen_recording(save_logs=True, reason='stop test')

    assert session['save_logs'] is True
    assert session['stop_requested'] is True
    assert ('write', b'q\n') in events
    assert 'close_stdin' in events


def test_recording_display_prefers_config_then_environment(monkeypatch):
    harness = SimpleNamespace(_recording_cfg={})
    harness._recording_display = MethodType(
        MainWindow._recording_display,
        harness,
    )

    monkeypatch.setenv('DISPLAY', ':42')
    assert harness._recording_display() == ':42'

    harness._recording_cfg = {'display': ':7'}
    assert harness._recording_display() == ':7'

    monkeypatch.delenv('DISPLAY')
    harness._recording_cfg = {}
    assert harness._recording_display() == ':1'
