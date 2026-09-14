"""Container tool windows must carry an identity the host desktop can match."""
import os
from types import MethodType, SimpleNamespace

os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')

import pytest

import mobipick_gui.main_window as main_window_module
from mobipick_gui.desktop_launcher import RQT_DESKTOP_ID
from mobipick_gui.display_runtime import DisplayRuntime
from mobipick_gui.main_window import MainWindow


def _display_runtime(backend: str) -> DisplayRuntime:
    environment = {'QT_QPA_PLATFORM': 'xcb' if backend == 'x11' else 'wayland'}
    return DisplayRuntime(
        backend=backend,
        environment=environment,
        mounts=(),
        x11_available=backend == 'x11',
        xauthority_mounted=False,
        warnings=(),
    )


def _env_harness(monkeypatch, backend: str) -> SimpleNamespace:
    monkeypatch.setattr(
        main_window_module,
        'graphics_device_group_environment',
        lambda: {},
    )
    harness = SimpleNamespace(
        _display_warnings_logged=set(),
        _display_runtime=lambda: _display_runtime(backend),
        _log_info=lambda _message: None,
        _workspace_runtime_env=lambda: {},
        _image_runtime_env=lambda _env: {},
        _selected_image='',
        _current_world=lambda: '',
        _current_master_uri=lambda: '',
    )
    harness._compose_env_args = MethodType(
        MainWindow._compose_env_args,
        harness,
    )
    return harness


@pytest.mark.parametrize('backend', ['x11', 'wayland'])
def test_compose_env_names_desktop_entry_only_when_requested(
    monkeypatch,
    backend,
):
    harness = _env_harness(monkeypatch, backend)

    plain = harness._compose_env_args()
    named = harness._compose_env_args(desktop_entry=RQT_DESKTOP_ID)

    assert not any(arg.startswith('RESOURCE_NAME=') for arg in plain)
    assert 'RESOURCE_NAME=mobipick-rqt' in named
    # The desktop identity never changes the display transport selection.
    assert [arg for arg in named if arg.startswith('QT_QPA_PLATFORM=')] == [
        arg for arg in plain if arg.startswith('QT_QPA_PLATFORM=')
    ]


@pytest.mark.parametrize('backend', ['x11', 'wayland'])
def test_compose_env_blocks_host_desktop_notifications(
    monkeypatch,
    backend,
):
    harness = _env_harness(monkeypatch, backend)

    env_args = harness._compose_env_args(
        {'DBUS_SESSION_BUS_ADDRESS': 'unix:path=/run/user/1000/bus'}
    )

    assert 'DBUS_SESSION_BUS_ADDRESS=unix:path=/run/mobipick-no-session-bus' in (
        env_args
    )
    assert 'DBUS_SESSION_BUS_ADDRESS=unix:path=/run/user/1000/bus' not in (
        env_args
    )


def _launch_harness(events: list) -> SimpleNamespace:
    class FakeTab:
        key = 'rqt'
        container_name = None
        exec_id = None

        def is_running(self):
            return False

        def start_program(self, program, args):
            events.append(('start_program', program, args))

    tab = FakeTab()
    harness = SimpleNamespace(
        tasks={'rviz': tab, 'rqt': tab},
        world_combo=SimpleNamespace(currentText=lambda: 'moelk_tables'),
        _ensure_tab=lambda key, label, closable=False: tab,
        _confirm_workspace_mismatch_warning=lambda _label: True,
        _log_info=lambda message: events.append(('log', message)),
        _claim_xhost=lambda *_args, **_kwargs: None,
        _rqt_tables_command=lambda: 'roslaunch rqt_tables_demo demo.launch',
        _rviz_command=lambda: 'rosrun rviz rviz',
        _compose_env_args=lambda **kwargs: events.append(
            ('compose_env', kwargs)
        ) or ['--env', 'DISPLAY=:0'],
        _ros_tool_service=lambda: 'mobipick_cmd',
        _wrap_line_buffered=lambda command: command,
        _schedule_host_to_container_copy=lambda _tab: None,
        _focus_tab=lambda key: events.append(('focus', key)),
        _ensure_roscore_ready=lambda callback: callback(),
        set_rqt_visual=lambda *args: events.append(('rqt_visual', *args)),
        set_rviz_visual=lambda *args: events.append(('rviz_visual', *args)),
    )
    for name in ('open_rqt_tables_demo', 'open_rviz'):
        setattr(harness, name, MethodType(getattr(MainWindow, name), harness))
    return harness


def test_rqt_launch_carries_its_desktop_entry_identity():
    events = []
    harness = _launch_harness(events)

    harness.open_rqt_tables_demo()

    compose_calls = [event[1] for event in events if event[0] == 'compose_env']
    assert compose_calls == [
        {'container_name': harness.tasks['rqt'].container_name,
         'desktop_entry': RQT_DESKTOP_ID}
    ]
    assert any(event[0] == 'start_program' for event in events)


def test_rviz_launch_keeps_its_native_window_class():
    events = []
    harness = _launch_harness(events)

    harness.open_rviz()

    compose_calls = [event[1] for event in events if event[0] == 'compose_env']
    assert len(compose_calls) == 1
    assert compose_calls[0]['ogre_glx'] is True
    assert compose_calls[0].get('desktop_entry') is None


def _config_command_harness(events: list) -> SimpleNamespace:
    class FakeTab:
        key = 'custom'
        container_name = None
        exec_id = None

        def is_running(self):
            return False

        def start_program(self, program, args):
            events.append(('start_program', program, args))

    harness = SimpleNamespace(
        _get_button_widget=lambda key: f'{key}-button',
        _guard_toggle_action=lambda key, button: True,
        _ensure_tab=lambda key, label, closable=False: FakeTab(),
        _confirm_workspace_mismatch_warning=lambda _label: True,
        _current_master_uri=lambda: '',
        _current_world=lambda: '',
        _host_ros_environment=lambda: {},
        _log_info=lambda message: events.append(('log', message)),
        _sh_quote=MainWindow._sh_quote,
        _neutralize_compose_ignore=MainWindow._neutralize_compose_ignore,
        _config_runs_on_host=MainWindow._config_runs_on_host,
        _claim_xhost=lambda *_args, **_kwargs: None,
        _configured_command_service=lambda _config: 'mobipick_cmd',
        _wrap_line_buffered=lambda command: command,
        _compose_env_args=lambda **kwargs: events.append(
            ('compose_env', kwargs)
        ) or [],
        _schedule_host_to_container_copy=lambda _tab: None,
        _focus_tab=lambda key: None,
        _update_stop_custom_enabled=lambda: None,
        _set_config_visual=lambda *_args: None,
        _ensure_roscore_ready=lambda callback: callback(),
    )
    harness._config_label = MethodType(MainWindow._config_label, harness)
    harness._run_config_command = MethodType(
        MainWindow._run_config_command,
        harness,
    )
    return harness


@pytest.mark.parametrize(
    ('command', 'expected_entry'),
    [
        ('rosrun rqt_gui rqt_gui --standalone dot_graph_visualization',
         RQT_DESKTOP_ID),
        ('roslaunch rqt_simple_launcher_config collect.launch',
         RQT_DESKTOP_ID),
        ('rosrun rae_upom_mobipick sfg_visualizer.py', None),
    ],
)
def test_custom_button_commands_borrow_rqt_identity_only_for_rqt_tools(
    command,
    expected_entry,
):
    events = []
    harness = _config_command_harness(events)

    harness._run_config_command(
        {
            'key': 'custom',
            'label': 'Custom',
            'kind': 'command',
            'command': command,
        }
    )

    compose_calls = [event[1] for event in events if event[0] == 'compose_env']
    assert len(compose_calls) == 1
    assert compose_calls[0]['desktop_entry'] == expected_entry
    assert any(event[0] == 'start_program' for event in events)


def test_sim_launch_keeps_alias_and_lends_rqt_identity_to_helper_panels():
    events = []

    class FakeTab:
        key = 'sim'
        container_name = None
        exec_id = None

        def start_program(self, program, args):
            events.append(('start_program', program, args))

    harness = SimpleNamespace(
        _sim_container_name='mobipick-run',
        _remote_master_enabled=lambda: False,
        _killing=False,
        _confirm_workspace_mismatch_warning=lambda _label: True,
        set_toggle_visual=lambda *_args, **_kwargs: None,
        _current_world=lambda: 'moelk_tables',
        _log_info=lambda message: events.append(('log', message)),
        _ensure_tab=lambda key, label, closable=False: FakeTab(),
        _claim_xhost=lambda *_args, **_kwargs: None,
        _compose_env_args=lambda **kwargs: events.append(
            ('compose_env', kwargs)
        ) or ['--env', 'DISPLAY=:0'],
        _wrap_line_buffered=lambda command: command,
        _workspace_sim_command=lambda: 'roslaunch demo demo_sim.launch',
        _schedule_host_to_container_copy=lambda _tab: None,
        _focus_tab=lambda key: None,
        _ensure_roscore_ready=lambda callback: callback(),
    )
    harness.bring_up_sim = MethodType(MainWindow.bring_up_sim, harness)

    harness.bring_up_sim()

    compose_calls = [event[1] for event in events if event[0] == 'compose_env']
    assert compose_calls == [
        {'ogre_glx': True, 'desktop_entry': RQT_DESKTOP_ID}
    ]
    start = next(event for event in events if event[0] == 'start_program')
    args = start[2]
    assert args[:4] == ['compose', 'run', '--rm', '--use-aliases']
    assert args[args.index('--name') + 1] == 'mobipick-run'


@pytest.mark.parametrize(
    'compose_file',
    [
        'docker-compose.yml',
        'private/jinja_templates/docker-compose.yml',
    ],
)
def test_compose_services_follow_the_host_hostname(compose_file):
    import yaml

    from mobipick_gui.config import PROJECT_ROOT

    path = PROJECT_ROOT / compose_file
    if not path.is_file():
        pytest.skip(f'{compose_file} is not part of this checkout')
    services = yaml.safe_load(path.read_text(encoding='utf-8'))['services']

    # The simulation keeps "mobipick" as the default so manual Compose use
    # still resolves GAZEBO_MASTER_URI; the GUI relies on the network alias.
    assert services['mobipick']['hostname'] == (
        '${MOBIPICK_CONTAINER_HOSTNAME:-mobipick}'
    )
    assert 'mobipick' in (
        services['mobipick']['networks']['default']['aliases']
    )
    assert services['mobipick_cmd']['hostname'] == (
        '${MOBIPICK_CONTAINER_HOSTNAME:-}'
    )
    # Host networking already reports the host's hostname; setting one there
    # is a Docker error.
    assert 'hostname' not in services['mobipick_remote_cmd']
