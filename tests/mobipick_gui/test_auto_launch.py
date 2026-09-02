from pathlib import Path
from types import MethodType, SimpleNamespace

from PyQt5.QtCore import QProcess
from PyQt5.QtWidgets import QApplication, QHeaderView, QWidget

import mobipick_gui.main_window as main_window_module
from mobipick_gui.main_window import (
    AutoLaunchProgressWindow,
    AutoLaunchWizard,
    MainWindow,
    dependency_launch_schedule,
)


def test_auto_launch_progress_counts_down_and_hides_after_ready(monkeypatch):
    monkeypatch.delenv('ROBOT_RACE', raising=False)
    app = QApplication.instance() or QApplication([])
    now = {'ns': 0}
    progress = AutoLaunchProgressWindow()
    progress._clock = lambda: now['ns']

    progress.start_countdown(5.7)
    assert progress.isVisible()
    assert progress.robot_animation is None
    assert progress.progress_bar.isVisible()
    assert progress.status_label.text() == 'Demo ready in 5.7 s'

    now['ns'] = 2_810_000_000
    progress._update_progress()
    assert progress.status_label.text() == 'Demo ready in 2.9 s'
    assert 490 <= progress.progress_bar.value() <= 494

    now['ns'] = 5_700_000_000
    progress._update_progress()
    assert progress.progress_bar.value() == 1000
    assert progress.status_label.text() == 'Demo ready'
    assert progress._hide_timer.isActive()
    assert progress._hide_timer.interval() == 1000
    progress.dismiss()
    app.processEvents()


def test_auto_launch_progress_shows_each_process_timeline(monkeypatch):
    monkeypatch.delenv('ROBOT_RACE', raising=False)
    app = QApplication.instance() or QApplication([])
    now = {'ns': 0}
    progress = AutoLaunchProgressWindow()
    progress._clock = lambda: now['ns']

    progress.start_countdown(
        10,
        [
            {
                'label': 'Roscore',
                'launch_at': 0,
                'ready_at': 4,
            },
            {
                'label': 'Simulation',
                'launch_at': 4,
                'ready_at': 10,
            },
            {
                'label': 'LiteLLM',
                'already_running': True,
            },
        ],
    )

    assert progress.processes_widget.isVisible()
    assert progress._update_timer.interval() == 25
    assert len(progress._process_rows) == 3
    assert all(row['animation'] is None for row in progress._process_rows)
    assert all(row['bar'].isVisible() for row in progress._process_rows)
    assert len(
        {row['name'].width() for row in progress._process_rows}
    ) == 1
    assert progress._process_rows[0]['bar'].format() == 'Ready in 4.0 s'
    assert progress._process_rows[1]['bar'].format() == 'Launches in 4.0 s'
    assert progress._process_rows[2]['bar'].format() == 'Already running'

    now['ns'] = 5_000_000_000
    progress._update_progress()
    assert progress._process_rows[0]['bar'].format() == 'Ready'
    assert progress._process_rows[1]['bar'].format() == 'Ready in 5.0 s'
    assert 165 <= progress._process_rows[1]['bar'].value() <= 167
    progress.dismiss()
    app.processEvents()


def test_auto_launch_progress_synchronizes_native_robot_race(monkeypatch):
    monkeypatch.setenv('ROBOT_RACE', 'true')
    app = QApplication.instance() or QApplication([])
    now = {'ns': 0}
    progress = AutoLaunchProgressWindow()
    progress._clock = lambda: now['ns']

    assert progress.robot_animation is not None
    assert progress.robot_animation.is_available
    assert progress.robot_animation.movie.scaledSize().isEmpty()
    assert progress.robot_animation.size() == (
        progress.robot_animation.movie.frameRect().size()
    )
    progress.start_countdown(
        4,
        [{'label': 'Simulation', 'launch_at': 0, 'ready_at': 4}],
    )
    row = progress._process_rows[0]
    assert not progress.progress_bar.isVisible()
    assert not row['bar'].isVisible()
    assert row['animation'] is not None
    assert row['animation'].movie.scaledSize().isEmpty()
    assert progress.width() >= (
        row['name'].width() + row['animation'].width()
    )
    assert progress.robot_animation.movie.currentFrameNumber() == 0

    now['ns'] = 2_000_000_000
    progress._update_progress()
    middle_frame = progress.robot_animation.movie.currentFrameNumber()
    assert 0 < middle_frame < progress.robot_animation.movie.frameCount() - 1
    assert row['animation'].movie.currentFrameNumber() == middle_frame

    now['ns'] = 4_000_000_000
    progress._update_progress()
    assert (
        progress.robot_animation.movie.currentFrameNumber()
        == progress.robot_animation.movie.frameCount() - 1
    )
    progress.dismiss()
    app.processEvents()


def test_auto_launch_progress_includes_window_layout_milestone():
    app = QApplication.instance() or QApplication([])
    now = {'ns': 0}
    progress = AutoLaunchProgressWindow()
    progress._clock = lambda: now['ns']

    progress.start_countdown(
        6,
        [
            {
                'label': 'Arrange windows',
                'launch_at': 6,
                'ready_at': 6,
                'pending_text': 'Runs in',
                'complete_text': 'Rearranged',
            }
        ],
    )

    bar = progress._process_rows[0]['bar']
    assert bar.format() == 'Runs in 6.0 s'
    now['ns'] = 6_000_000_000
    progress._update_progress()
    assert bar.format() == 'Rearranged'
    progress.dismiss()
    app.processEvents()


def test_auto_window_layout_runs_one_second_after_processes_are_ready():
    advanced = SimpleNamespace(
        _launch_plan={
            'mode': 'advanced',
            'processes': [
                {'button': 'roscore', 'duration_seconds': 4.5},
                {
                    'button': 'sim',
                    'duration_seconds': 10,
                    'depends_on': 'roscore',
                },
            ],
        }
    )
    advanced._window_layout_delay_from_timeline = MethodType(
        MainWindow._window_layout_delay_from_timeline,
        advanced,
    )
    legacy = SimpleNamespace(
        _launch_plan={
            'timeline': [
                {'button': 'roscore', 'at_seconds': 0},
                {'button': 'sim', 'at_seconds': 4.5},
            ]
        }
    )
    legacy._window_layout_delay_from_timeline = MethodType(
        MainWindow._window_layout_delay_from_timeline,
        legacy,
    )

    assert advanced._window_layout_delay_from_timeline() == 15_500
    assert legacy._window_layout_delay_from_timeline() == 5_500


def test_auto_launch_adds_saved_window_layout_to_progress():
    manager = SimpleNamespace(has_saved_layout=lambda: True)
    harness = SimpleNamespace(
        _window_layout_auto_apply=True,
        _window_layout_manager=manager,
        _window_layout_delay_ms=5500,
    )
    harness._add_window_layout_progress = MethodType(
        MainWindow._add_window_layout_progress,
        harness,
    )
    processes = []

    total = harness._add_window_layout_progress(processes, 4.5)

    assert total == 5.5
    assert processes == [
        {
            'label': 'Arrange windows',
            'launch_at': 5.5,
            'ready_at': 5.5,
            'pending_text': 'Runs in',
            'complete_text': 'Rearranged',
        }
    ]


def test_auto_launch_progress_centers_on_parent():
    app = QApplication.instance() or QApplication([])
    parent = QWidget()
    parent.setGeometry(100, 120, 900, 600)
    parent.show()
    app.processEvents()
    progress = AutoLaunchProgressWindow(parent)

    progress.start_countdown(10)
    app.processEvents()

    offset = progress.frameGeometry().center() - parent.frameGeometry().center()
    assert abs(offset.x()) <= 2
    assert abs(offset.y()) <= 2
    progress.dismiss()
    parent.close()
    app.processEvents()


def test_auto_launch_wizard_opens_advanced_profile():
    app = QApplication.instance() or QApplication([])
    launched = []
    dialog = AutoLaunchWizard(
        [('roscore', 'Roscore'), ('sim', 'Simulation')],
        [],
        Path('/tmp/auto_launch_test.yaml'),
        processes=[
            {
                'button': 'sim',
                'duration_seconds': 12,
                'depends_on': 'roscore',
                'dependency_type': 'soft',
                'ready_percentage': 30,
            }
        ],
        mode='advanced',
        measurement_launcher=lambda key: launched.append(key) or True,
    )

    assert dialog.mode() == 'advanced'
    assert dialog.processes() == [
        {
            'button': 'sim',
            'duration_seconds': 12.0,
            'depends_on': 'roscore',
            'dependency_type': 'soft',
            'ready_percentage': 30.0,
        }
    ]
    assert (
        dialog._advanced_table.horizontalHeader().sectionResizeMode(1)
        == QHeaderView.Stretch
    )
    clock_values = iter([0, 5_650_000_000])
    dialog._measurement_clock = lambda: next(clock_values)
    sim_row = dialog._advanced_rows[1]
    sim_row['measure'].click()
    sim_row['ready'].click()
    assert launched == ['sim']
    assert sim_row['duration'].value() == 5.7
    dialog.close()
    app.processEvents()


def test_dependency_schedule_supports_hard_and_soft_dependencies():
    schedule = dependency_launch_schedule(
        [
            {'button': 'roscore', 'duration_seconds': 10},
            {
                'button': 'sim',
                'duration_seconds': 20,
                'depends_on': 'roscore',
                'dependency_type': 'hard',
            },
            {
                'button': 'rviz',
                'duration_seconds': 4,
                'depends_on': 'sim',
                'dependency_type': 'soft',
                'ready_percentage': 30,
            },
        ]
    )

    assert schedule['roscore'] == (0.0, 10.0, False)
    assert schedule['sim'] == (10.0, 30.0, False)
    assert schedule['rviz'] == (16.0, 20.0, False)


def test_dependency_schedule_does_not_wait_for_running_process():
    schedule = dependency_launch_schedule(
        [
            {'button': 'roscore', 'duration_seconds': 10},
            {
                'button': 'sim',
                'duration_seconds': 20,
                'depends_on': 'roscore',
                'dependency_type': 'hard',
            },
        ],
        {'roscore'},
    )

    assert schedule['roscore'] == (0.0, 0.0, True)
    assert schedule['sim'] == (0.0, 20.0, False)


def test_dependency_schedule_rejects_cycles():
    processes = [
        {'button': 'sim', 'depends_on': 'rviz'},
        {'button': 'rviz', 'depends_on': 'sim'},
    ]

    try:
        dependency_launch_schedule(processes)
    except ValueError as exc:
        assert 'cycle' in str(exc)
    else:
        raise AssertionError('cycle was accepted')


def test_advanced_shutdown_does_not_stop_preexisting_processes():
    harness = SimpleNamespace(
        _launch_plan={
            'mode': 'advanced',
            'processes': [{'button': 'roscore'}],
            'shutdown_order': ['roscore'],
            'shutdown_skip': [],
        },
        _auto_launch_active_keys=[],
    )
    harness._auto_launch_shutdown_order = MethodType(
        MainWindow._auto_launch_shutdown_order,
        harness,
    )

    assert harness._auto_launch_shutdown_order() == []


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


def test_roscore_auto_launch_stop_preserves_host_commands():
    harness, events = _auto_launch_harness()
    harness._auto_launch_active_keys = ['litellm', 'sim', 'roscore']
    harness._auto_launch_shutdown_order = lambda: [
        'litellm',
        'sim',
        'roscore',
    ]
    harness._config_buttons = {
        'litellm': {'kind': 'command', 'host': True},
        'sim': {'kind': 'builtin', 'host': False},
    }
    harness._config_runs_on_host = MainWindow._config_runs_on_host
    harness._trigger_auto_launch_step = lambda key, *, target_running: (
        events.append(('trigger', key, target_running))
    )

    harness._stop_auto_launch_stack(preserve_host_commands=True)

    assert ('trigger', 'litellm', False) not in events
    assert ('trigger', 'sim', False) in events
    assert ('trigger', 'roscore', False) in events


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


def test_reset_config_button_visuals_preserves_running_host_command():
    events = []
    harness = SimpleNamespace(
        _config_button_order=['litellm', 'demo'],
        _config_buttons={
            'litellm': {
                'key': 'litellm',
                'label': 'LiteLLM',
                'kind': 'command',
                'host': True,
            },
            'demo': {'key': 'demo', 'label': 'Demo'},
        },
        tasks={
            'litellm': SimpleNamespace(is_running=lambda: True),
            'demo': SimpleNamespace(is_running=lambda: False),
        },
        _set_config_visual=lambda cfg, state, text, enabled: events.append(
            (cfg.get('key'), state, text, enabled)
        ),
    )
    harness._config_label = MethodType(MainWindow._config_label, harness)
    harness._config_runs_on_host = MainWindow._config_runs_on_host
    harness._reset_config_button_visuals = MethodType(
        MainWindow._reset_config_button_visuals,
        harness,
    )

    harness._reset_config_button_visuals(
        preserve_running_host_commands=True,
    )

    assert events == [
        ('litellm', 'green', 'Stop LiteLLM', True),
        ('demo', 'red', 'Start Demo', True),
    ]


def test_host_config_command_does_not_start_roscore():
    events = []

    class FakeTab:
        container_name = 'old-container'
        exec_id = 'old-exec'

        def is_running(self):
            return False

        def start_program(self, program, args):
            events.append(('start_program', program, args))

    harness = SimpleNamespace(
        _get_button_widget=lambda key: f'{key}-button',
        _guard_toggle_action=lambda key, button: True,
        _ensure_tab=lambda key, label, closable=False: FakeTab(),
        _current_master_uri=lambda: '',
        _log_info=lambda message: events.append(('log', message)),
        _neutralize_compose_ignore=MainWindow._neutralize_compose_ignore,
        _config_runs_on_host=MainWindow._config_runs_on_host,
        _focus_tab=lambda key: events.append(('focus', key)),
        _update_stop_custom_enabled=lambda: events.append('update_stop_custom'),
        _set_config_visual=lambda cfg, state, text, enabled: events.append(
            ('visual', state, text, enabled)
        ),
        _ensure_roscore_ready=lambda callback: events.append(
            'ensure_roscore_ready'
        ),
    )
    harness._config_label = MethodType(MainWindow._config_label, harness)
    harness._prepared_config_stop_command = MethodType(
        MainWindow._prepared_config_stop_command,
        harness,
    )
    harness._run_config_command = MethodType(
        MainWindow._run_config_command,
        harness,
    )

    harness._run_config_command(
        {
            'key': 'litellm',
            'label': 'LiteLLM',
            'kind': 'command',
            'command': 'docker run --rm litellm',
            'host': True,
            'requires_roscore': True,
        }
    )

    assert 'ensure_roscore_ready' not in events
    assert any(
        event[:2] == ('start_program', 'bash')
        for event in events
        if isinstance(event, tuple)
    )


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
        _config_runs_on_host=MainWindow._config_runs_on_host,
        _timers_cfg={'custom_tab_sigint_delay_ms': 0},
        tasks={
            'roscore': roscore_tab,
            'sim': SimpleNamespace(is_running=lambda: False),
            'tables': SimpleNamespace(is_running=lambda: False),
            'rviz': SimpleNamespace(is_running=lambda: False),
            'rqt': SimpleNamespace(is_running=lambda: False),
        },
        _stop_auto_launch_stack=lambda **kwargs: events.append(
            ('stop_auto_launch', kwargs)
        ),
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
        _reset_config_button_visuals=lambda **kwargs: events.append(
            ('reset_config_buttons', kwargs)
        ),
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

    reset_event = (
        'reset_config_buttons',
        {'preserve_running_host_commands': True},
    )
    assert reset_event in events
    assert events.index(reset_event) > events.index(
        ('rqt', 'red', 'Start RQt Tables', True)
    )
    assert harness._roscore_stopping is False
