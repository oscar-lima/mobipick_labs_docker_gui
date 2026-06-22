from types import MethodType, SimpleNamespace

from PyQt5.QtCore import QProcess

import mobipick_gui.main_window as main_window_module
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


def test_recording_start_delay_includes_launch_layout_and_extra_delay():
    harness = SimpleNamespace(
        _launch_plan={
            'timeline': [
                {'button': 'roscore', 'at_seconds': 1.0},
                {'button': 'sim', 'at_seconds': 4.5},
            ],
            'recording_start_delay_seconds': 2.0,
        },
        _window_layout_delay_ms=6000,
    )
    harness._recording_start_delay_ms = MethodType(
        MainWindow._recording_start_delay_ms,
        harness,
    )

    assert harness._recording_start_delay_ms() == 8000


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


def test_builtin_sim_and_rviz_commands_can_be_overridden():
    harness = SimpleNamespace(
        _config_buttons={
            'sim': {'command': 'roslaunch custom sim.launch'},
            'tables': {'command': 'rosrun custom tables.py'},
            'rviz': {'command': 'rviz -d /tmp/custom.rviz'},
            'rqt': {'command': 'roslaunch custom rqt.launch'},
        },
        _workspace_registry=SimpleNamespace(active_workspace=lambda: None),
        _current_world=lambda: 'demo_world',
        _sh_quote=lambda value: value,
    )
    harness._workspace_sim_command = MethodType(
        MainWindow._workspace_sim_command,
        harness,
    )
    harness._profile_button_command = MethodType(
        MainWindow._profile_button_command,
        harness,
    )
    harness._rviz_command = MethodType(MainWindow._rviz_command, harness)
    harness._tables_demo_command = MethodType(
        MainWindow._tables_demo_command,
        harness,
    )
    harness._rqt_tables_command = MethodType(
        MainWindow._rqt_tables_command,
        harness,
    )

    assert harness._workspace_sim_command() == 'roslaunch custom sim.launch'
    assert harness._tables_demo_command() == 'rosrun custom tables.py'
    assert harness._rviz_command() == 'rviz -d /tmp/custom.rviz'
    assert harness._rqt_tables_command() == 'roslaunch custom rqt.launch'


def test_reset_config_button_visuals_restores_idle_labels():
    events = []
    harness = SimpleNamespace(
        _config_button_order=['demo', 'tools'],
        _config_buttons={
            'demo': {'key': 'demo', 'label': 'Demo'},
            'tools': {'key': 'tools'},
        },
        _set_config_visual=lambda cfg, state, text, enabled: events.append(
            (cfg.get('key'), state, text, enabled)
        ),
    )
    harness._config_label = MethodType(MainWindow._config_label, harness)
    harness._reset_config_button_visuals = MethodType(
        MainWindow._reset_config_button_visuals,
        harness,
    )

    harness._reset_config_button_visuals()

    assert events == [
        ('demo', 'red', 'Start Demo', True),
        ('tools', 'red', 'Start tools', True),
    ]


def test_roscore_shutdown_finalizer_resets_config_buttons(monkeypatch):
    events = []

    def _record_visual(name):
        def _record(state, text, enabled):
            events.append((name, state, text, enabled))

        return _record

    class FakeTab:
        key = 'roscore'
        container_name = 'mobipick-roscore'
        exec_id = 'exec-id'

        def is_running(self):
            return False

        def pid(self):
            return None

    roscore_tab = FakeTab()
    harness = SimpleNamespace(
        _roscore_stopping=False,
        _roscore_container_name='mobipick-roscore',
        _roscore_running_cached=True,
        _roscore_last_start_ts=1.0,
        _sim_running_cached=False,
        _killing=False,
        _script_active_tab_key=None,
        _script_choices=['script.py'],
        _terminal_stopping=False,
        _terminal_running_cached=False,
        _terminal_stream_tab_key=None,
        run_script_button='script-button',
        terminal_button='terminal-button',
        _cleanup_done=False,
        _config_button_order=['demo'],
        _config_buttons={'demo': {'key': 'demo', 'label': 'Demo'}},
        _timers_cfg={'custom_tab_sigint_delay_ms': 0},
        tasks={
            'roscore': roscore_tab,
            'sim': SimpleNamespace(is_running=lambda: False),
            'tables': SimpleNamespace(is_running=lambda: False),
            'rviz': SimpleNamespace(is_running=lambda: False),
            'rqt': SimpleNamespace(is_running=lambda: False),
        },
        _stop_auto_launch_stack=lambda: events.append('stop_auto_launch'),
        _log_info=lambda message: events.append(('log', message)),
        set_roscore_visual=_record_visual('roscore'),
        _disable_toggle_preserving_visual=lambda key, button: events.append(
            ('disable', key, button)
        ),
        _get_button_widget=lambda key: f'{key}-button',
        _terminal_is_active=lambda: False,
        stop_terminal=lambda: events.append('stop_terminal'),
        _ensure_tab=lambda key, label, closable=False: roscore_tab,
        _append_gui_html=lambda *args: events.append(('html', *args)),
        _docker_stop_if_exists=lambda *args, **kwargs: [],
        _stop_all_related=lambda *args, **kwargs: [],
        _cleanup_script_available=lambda: False,
        _run_command_sequence=lambda commands, on_finished, log_key: on_finished(),
        _revoke_x=lambda: events.append('revoke_x'),
        set_toggle_visual=_record_visual('sim'),
        set_tables_visual=_record_visual('tables'),
        set_rviz_visual=_record_visual('rviz'),
        set_rqt_visual=_record_visual('rqt'),
        _reset_config_button_visuals=lambda: events.append('reset_config_buttons'),
        set_script_visual=_record_visual('script'),
        set_terminal_visual=_record_visual('terminal'),
        _update_stop_custom_enabled=lambda: events.append('update_stop_custom'),
        _finalize_auto_launch_stop=lambda: events.append('finalize_auto_launch'),
    )
    harness.shutdown_roscore = MethodType(MainWindow.shutdown_roscore, harness)
    monkeypatch.setattr(
        main_window_module.QTimer,
        'singleShot',
        lambda _delay, callback: callback(),
    )

    harness.shutdown_roscore()

    assert 'reset_config_buttons' in events
    assert events.index('reset_config_buttons') > events.index(
        ('rqt', 'red', 'Start RQt Tables', True)
    )
    assert harness._roscore_stopping is False
