import subprocess

from mobipick_gui import window_control
from mobipick_gui.window_control import (
    GnomeWaylandWindowBackend,
    WindowInfo,
    X11WindowBackend,
    install_gnome_extension,
    parse_gvariant_tuple,
    select_backend,
    session_type,
)
from mobipick_gui.window_layout import WindowLayoutManager

X11_ENV = {'XDG_SESSION_TYPE': 'x11', 'DISPLAY': ':0'}
WAYLAND_ENV = {'XDG_SESSION_TYPE': 'wayland', 'WAYLAND_DISPLAY': 'wayland-0'}


def test_record_baseline_skips_missing_wmctrl(tmp_path, monkeypatch):
    def fake_which(name):
        return None if name == 'missing-wmctrl' else f'/usr/bin/{name}'

    def fail_run(*args, **kwargs):
        raise AssertionError('wmctrl should not be executed when unavailable')

    warnings: list[str] = []

    monkeypatch.setattr(window_control.shutil, 'which', fake_which)
    monkeypatch.setattr(subprocess, 'run', fail_run)

    manager = WindowLayoutManager(
        tmp_path / 'window_layout.yaml',
        wmctrl_bin='missing-wmctrl',
        log_warning=warnings.append,
        environ=X11_ENV,
    )

    manager.record_baseline()

    assert warnings == []
    assert manager._auto_apply_done is True


def test_has_saved_layout_requires_window_entries(tmp_path):
    manager = WindowLayoutManager(tmp_path / 'window_layout.yaml', environ=X11_ENV)

    assert manager.has_saved_layout() is False
    manager._layout = {'windows': [{'title': 'RViz'}]}
    assert manager.has_saved_layout() is True


def test_session_type_detection():
    assert session_type(X11_ENV) == 'x11'
    assert session_type(WAYLAND_ENV) == 'wayland'
    assert session_type({'WAYLAND_DISPLAY': 'wayland-0'}) == 'wayland'
    assert session_type({'DISPLAY': ':1'}) == 'x11'
    assert session_type({'XDG_SESSION_TYPE': 'tty', 'WAYLAND_DISPLAY': 'wayland-0'}) == 'wayland'
    assert session_type({'QT_QPA_PLATFORM': 'wayland-egl'}) == 'wayland'
    assert session_type({'QT_QPA_PLATFORM': 'xcb'}) == 'x11'
    assert session_type(
        {'QT_QPA_PLATFORM': 'xcb', 'WAYLAND_DISPLAY': 'wayland-0'}
    ) == 'x11'
    assert session_type({}) == ''


def test_parse_gvariant_tuple_handles_gdbus_output():
    assert parse_gvariant_tuple("(true,)") == (True,)
    assert parse_gvariant_tuple("(false,)") == (False,)
    assert parse_gvariant_tuple("(1,)") == (1,)
    assert parse_gvariant_tuple("('[{\"id\": \"5\"}]',)") == ('[{"id": "5"}]',)
    # booleans inside the JSON payload must survive untouched
    payload = '[{"maximized":true,"minimized":false,"title":"it\'s true"}]'
    assert parse_gvariant_tuple(f"({payload!r},)") == (payload,)
    assert parse_gvariant_tuple('') is None
    assert parse_gvariant_tuple('garbage(') is None


def test_select_backend_uses_wmctrl_on_x11(monkeypatch):
    monkeypatch.setattr(window_control.shutil, 'which', lambda name: f'/usr/bin/{name}')
    monkeypatch.setattr(
        subprocess,
        'run',
        lambda *a, **k: (_ for _ in ()).throw(AssertionError('no probe expected on X11')),
    )
    backend = select_backend(environ=X11_ENV)
    assert isinstance(backend, X11WindowBackend)
    assert backend.available is True


def test_select_backend_prefers_gnome_extension_on_wayland(monkeypatch):
    calls: list[list[str]] = []

    def fake_run(cmd, **kwargs):
        calls.append(list(cmd))
        return subprocess.CompletedProcess(cmd, 0, stdout='(1,)\n', stderr='')

    monkeypatch.setattr(window_control.shutil, 'which', lambda name: f'/usr/bin/{name}')
    monkeypatch.setattr(subprocess, 'run', fake_run)
    backend = select_backend(environ=WAYLAND_ENV)
    assert isinstance(backend, GnomeWaylandWindowBackend)
    assert backend.available is True
    assert calls and calls[0][0] == 'gdbus'
    assert calls[0][-1].endswith('MobipickWinCtl.Version')


def test_select_backend_falls_back_to_wmctrl_when_extension_missing(monkeypatch):
    def fake_run(cmd, **kwargs):
        return subprocess.CompletedProcess(
            cmd, 1, stdout='', stderr='Error: Object does not exist at path'
        )

    monkeypatch.setattr(window_control.shutil, 'which', lambda name: f'/usr/bin/{name}')
    monkeypatch.setattr(subprocess, 'run', fake_run)
    backend = select_backend(environ=WAYLAND_ENV)
    assert isinstance(backend, GnomeWaylandWindowBackend)
    backend._probe_thread.join(timeout=1)
    assert backend._extension_available is False
    assert isinstance(backend._fallback, X11WindowBackend)


def test_wayland_without_any_tool_reports_extension_install_hint(monkeypatch):
    def fake_which(name):
        return '/usr/bin/gdbus' if name == 'gdbus' else None

    def fake_run(cmd, **kwargs):
        return subprocess.CompletedProcess(cmd, 1, stdout='', stderr='nope')

    monkeypatch.setattr(window_control.shutil, 'which', fake_which)
    monkeypatch.setattr(subprocess, 'run', fake_run)
    backend = select_backend(environ=WAYLAND_ENV)
    assert isinstance(backend, GnomeWaylandWindowBackend)
    assert backend.available is False
    missing = backend.missing_tools()
    assert any('--install-gnome-window-extension' in item for item in missing)


def test_gnome_backend_lists_windows_from_extension_json(monkeypatch):
    payload = (
        '[{"id": "12345", "title": "RViz", "pid": 42, "desktop": 1, '
        '"x": 10, "y": 20, "width": 800, "height": 600, '
        '"wm_class": ["rviz", "rviz"], "stack_index": 3}]'
    )

    def fake_run(cmd, **kwargs):
        if cmd[-1].endswith('.Version'):
            return subprocess.CompletedProcess(cmd, 0, stdout='(1,)\n', stderr='')
        assert cmd[-1].endswith('.ListWindows')
        return subprocess.CompletedProcess(cmd, 0, stdout=f"({payload!r},)\n", stderr='')

    monkeypatch.setattr(window_control.shutil, 'which', lambda name: f'/usr/bin/{name}')
    monkeypatch.setattr(subprocess, 'run', fake_run)
    backend = GnomeWaylandWindowBackend()
    windows = backend.list_windows(include_classes=True, include_stack=True)
    assert windows == [
        WindowInfo(
            wid='12345',
            title='RViz',
            desktop=1,
            pid=42,
            x=10,
            y=20,
            width=800,
            height=600,
            wm_class=['rviz', 'rviz'],
            stack_index=3,
        )
    ]


def test_gnome_backend_apply_calls_extension_methods(monkeypatch, tmp_path):
    calls: list[list[str]] = []
    windows_json = (
        '[{"id": "777", "title": "Gazebo", "pid": 9, "desktop": 0, '
        '"x": 0, "y": 0, "width": 100, "height": 100, '
        '"wm_class": ["gzclient", "gzclient"], "stack_index": 0}]'
    )

    def fake_run(cmd, **kwargs):
        calls.append(list(cmd))
        method = cmd[-1] if cmd[-1].startswith('org.gnome') else next(
            arg for arg in cmd if arg.startswith('org.gnome.Shell.Extensions.MobipickWinCtl.')
        )
        if method.endswith('.Version'):
            return subprocess.CompletedProcess(cmd, 0, stdout='(1,)\n', stderr='')
        if method.endswith('.ListWindows'):
            return subprocess.CompletedProcess(cmd, 0, stdout=f"({windows_json!r},)\n", stderr='')
        return subprocess.CompletedProcess(cmd, 0, stdout='(true,)\n', stderr='')

    monkeypatch.setattr(window_control.shutil, 'which', lambda name: f'/usr/bin/{name}')
    monkeypatch.setattr(subprocess, 'run', fake_run)

    manager = WindowLayoutManager(tmp_path / 'layout.yaml', environ=WAYLAND_ENV)
    assert manager.backend_name == 'gnome-shell extension'
    manager._layout = {
        'windows': [
            {
                'title': 'Gazebo',
                'wm_class': ['gzclient'],
                'desktop': 2,
                'geometry': {'x': 50, 'y': 60, 'width': 640, 'height': 480},
                'stack_index': 0,
            }
        ]
    }
    manager._auto_apply_done = False
    manager.maybe_apply_saved_layout()

    methods = [
        next(arg for arg in cmd if arg.startswith('org.gnome.Shell.Extensions.MobipickWinCtl.'))
        .rsplit('.', 1)[1]
        for cmd in calls
    ]
    assert methods == [
        'Version',
        'ListWindows',
        'Unmaximize',
        'MoveResize',
        'SetWorkspace',
        'Activate',
    ]
    move = calls[methods.index('MoveResize')]
    assert move[-5:] == ["'777'", '50', '60', '640', '480']
    assert calls[methods.index('SetWorkspace')][-2:] == ["'777'", '2']
    assert manager._applied_ids == {'777'}


def test_layout_unmaximizes_all_windows_before_resizing(tmp_path):
    calls: list[tuple] = []

    class FakeBackend:
        available = True
        name = 'fake'

        def list_windows(self, **_kwargs):
            return [
                WindowInfo('1', 'First', 0, 1, 0, 0, 100, 100, ['first']),
                WindowInfo('2', 'Second', 0, 2, 0, 0, 100, 100, ['second']),
            ]

        def unmaximize(self, wid):
            calls.append(('unmaximize', wid))
            return True

        def move_resize(self, wid, x, y, width, height):
            calls.append(('move_resize', wid, x, y, width, height))

        def set_desktop(self, wid, desktop):
            calls.append(('set_desktop', wid, desktop))

        def restack(self, wids):
            calls.append(('restack', tuple(wids)))

    manager = WindowLayoutManager(tmp_path / 'layout.yaml', backend=FakeBackend())
    manager._layout = {
        'windows': [
            {
                'title': 'First',
                'geometry': {'x': 10, 'y': 20, 'width': 300, 'height': 400},
            },
            {
                'title': 'Second',
                'geometry': {'x': 50, 'y': 60, 'width': 700, 'height': 800},
            },
        ]
    }
    manager._auto_apply_done = False

    manager.maybe_apply_saved_layout()

    assert calls[:2] == [('unmaximize', '1'), ('unmaximize', '2')]
    assert calls[2][:2] == ('move_resize', '1')
    assert calls[3][:2] == ('move_resize', '2')


def test_install_gnome_extension_copies_files_and_enables(tmp_path, monkeypatch):
    calls: list[list[str]] = []

    def fake_run(cmd, **kwargs):
        calls.append(list(cmd))
        if cmd[:2] == ['gsettings', 'get']:
            return subprocess.CompletedProcess(
                cmd, 0, stdout="['ubuntu-dock@ubuntu.com']\n", stderr=''
            )
        return subprocess.CompletedProcess(cmd, 0, stdout='', stderr='')

    monkeypatch.setattr(window_control.shutil, 'which', lambda name: f'/usr/bin/{name}')
    messages: list[str] = []
    target = install_gnome_extension(
        environ={'XDG_DATA_HOME': str(tmp_path)},
        run=fake_run,
        log=messages.append,
    )
    assert target == tmp_path / 'gnome-shell' / 'extensions' / window_control.GNOME_EXTENSION_UUID
    assert (target / 'metadata.json').is_file()
    assert (target / 'extension.js').is_file()
    set_call = next(cmd for cmd in calls if cmd[:2] == ['gsettings', 'set'])
    assert set_call[-1] == "['ubuntu-dock@ubuntu.com', 'winctl@mobipick-labs-docker-gui']"
    assert any('Log out' in msg for msg in messages)


def test_wmctrl_fallback_on_wayland_explains_empty_capture(monkeypatch, tmp_path):
    def fake_run(cmd, **kwargs):
        if cmd[0] == 'gdbus':
            return subprocess.CompletedProcess(cmd, 1, stdout='', stderr='no object')
        return subprocess.CompletedProcess(cmd, 0, stdout='', stderr='')

    monkeypatch.setattr(window_control.shutil, 'which', lambda name: f'/usr/bin/{name}')
    monkeypatch.setattr(subprocess, 'run', fake_run)
    warnings: list[str] = []
    manager = WindowLayoutManager(
        tmp_path / 'layout.yaml', environ=WAYLAND_ENV, log_warning=warnings.append
    )
    manager.backend._probe_thread.join(timeout=1)
    assert manager.backend_name == 'gnome-shell extension'
    assert warnings == []  # nothing logged at construction time
    assert manager.capture_layout() is None
    assert len(warnings) == 1
    assert 'No windows found' in warnings[0]
    assert '--install-gnome-window-extension' in warnings[0]


def test_find_own_window_prefers_pid_match(monkeypatch):
    import os

    class FakeBackend:
        def list_windows(self, **kwargs):
            return [
                WindowInfo('1', 'Auto Launch Progress', 0, 999, 0, 0, 1, 1, []),
                WindowInfo('2', 'Auto Launch Progress', 0, os.getpid(), 0, 0, 1, 1, []),
                WindowInfo('3', 'Other', 0, os.getpid(), 0, 0, 1, 1, []),
            ]

    from mobipick_gui.window_control import find_own_window

    assert find_own_window(FakeBackend(), 'Auto Launch Progress').wid == '2'
    assert find_own_window(FakeBackend(), 'Other').wid == '3'
    assert find_own_window(FakeBackend(), 'Missing') is None
    assert find_own_window(FakeBackend(), '') is None


def test_gnome_backend_set_above_calls_extension(monkeypatch):
    calls: list[list[str]] = []

    def fake_run(cmd, **kwargs):
        calls.append(list(cmd))
        return subprocess.CompletedProcess(cmd, 0, stdout='(true,)\n', stderr='')

    monkeypatch.setattr(window_control.shutil, 'which', lambda name: f'/usr/bin/{name}')
    monkeypatch.setattr(subprocess, 'run', fake_run)
    backend = GnomeWaylandWindowBackend()
    assert backend.set_above('42', True) is True
    assert calls[-1][-3:] == ['org.gnome.Shell.Extensions.MobipickWinCtl.SetAbove', "'42'", 'true']
    assert backend.set_above('42', False) is True
    assert calls[-1][-1] == 'false'


def test_gnome_backend_unmaximize_calls_extension(monkeypatch):
    calls: list[list[str]] = []

    def fake_run(cmd, **kwargs):
        calls.append(list(cmd))
        return subprocess.CompletedProcess(cmd, 0, stdout='(true,)\n', stderr='')

    monkeypatch.setattr(window_control.shutil, 'which', lambda name: f'/usr/bin/{name}')
    monkeypatch.setattr(subprocess, 'run', fake_run)

    assert GnomeWaylandWindowBackend().unmaximize('42') is True
    assert calls[-1][-2:] == [
        'org.gnome.Shell.Extensions.MobipickWinCtl.Unmaximize',
        "'42'",
    ]


def test_gnome_backend_clear_attention_calls_extension(monkeypatch):
    calls: list[list[str]] = []

    def fake_run(cmd, **kwargs):
        calls.append(list(cmd))
        return subprocess.CompletedProcess(cmd, 0, stdout='(true,)\n', stderr='')

    monkeypatch.setattr(window_control.shutil, 'which', lambda name: f'/usr/bin/{name}')
    monkeypatch.setattr(subprocess, 'run', fake_run)

    assert GnomeWaylandWindowBackend().clear_attention('42') is True
    assert calls[-1][-2:] == [
        'org.gnome.Shell.Extensions.MobipickWinCtl.ClearAttention',
        "'42'",
    ]


def test_x11_backend_set_above_uses_wmctrl(monkeypatch):
    calls: list[list[str]] = []

    def fake_run(cmd, **kwargs):
        calls.append(list(cmd))
        return subprocess.CompletedProcess(cmd, 0, stdout='', stderr='')

    monkeypatch.setattr(window_control.shutil, 'which', lambda name: f'/usr/bin/{name}')
    monkeypatch.setattr(subprocess, 'run', fake_run)
    backend = X11WindowBackend()
    assert backend.set_above('0x1', True) is True
    assert calls[-1] == ['wmctrl', '-i', '-r', '0x1', '-b', 'add,above']


def test_x11_backend_unmaximize_clears_fullscreen_and_maximized(monkeypatch):
    calls: list[list[str]] = []

    def fake_run(cmd, **kwargs):
        calls.append(list(cmd))
        return subprocess.CompletedProcess(cmd, 0, stdout='', stderr='')

    monkeypatch.setattr(window_control.shutil, 'which', lambda name: f'/usr/bin/{name}')
    monkeypatch.setattr(subprocess, 'run', fake_run)

    assert X11WindowBackend().unmaximize('0x1') is True
    assert calls[-2:] == [
        [
            'wmctrl',
            '-i',
            '-r',
            '0x1',
            '-b',
            'remove,maximized_vert,maximized_horz',
        ],
        ['wmctrl', '-i', '-r', '0x1', '-b', 'remove,fullscreen'],
    ]


def test_x11_backend_clear_attention_uses_wmctrl(monkeypatch):
    calls: list[list[str]] = []

    def fake_run(cmd, **kwargs):
        calls.append(list(cmd))
        return subprocess.CompletedProcess(cmd, 0, stdout='', stderr='')

    monkeypatch.setattr(window_control.shutil, 'which', lambda name: f'/usr/bin/{name}')
    monkeypatch.setattr(subprocess, 'run', fake_run)

    assert X11WindowBackend().clear_attention('0x1') is True
    assert calls[-1] == [
        'wmctrl', '-i', '-r', '0x1', '-b',
        'remove,demands_attention',
    ]


def test_attention_suppression_only_clears_new_windows(tmp_path):
    class FakeBackend:
        available = True
        name = 'fake'

        def __init__(self):
            self.windows = [
                WindowInfo('old', 'Editor', 0, 1, 0, 0, 100, 100, []),
            ]
            self.cleared: list[str] = []

        def list_windows(self, **_kwargs):
            return list(self.windows)

        def clear_attention(self, wid):
            self.cleared.append(wid)
            return True

    backend = FakeBackend()
    manager = WindowLayoutManager(tmp_path / 'layout.yaml', backend=backend)

    assert manager.begin_attention_suppression() is True
    backend.windows.append(
        WindowInfo('new', 'Command GUI', 0, 2, 0, 0, 100, 100, [])
    )

    assert manager.suppress_new_window_attention() == 1
    assert manager.suppress_new_window_attention() == 0
    assert backend.cleared == ['new']

    manager.end_attention_suppression()
    backend.windows.append(
        WindowInfo('later', 'Browser', 0, 3, 0, 0, 100, 100, [])
    )
    assert manager.suppress_new_window_attention() == 0
    assert backend.cleared == ['new']


def test_window_backends_activate_without_qt_request(monkeypatch):
    calls: list[list[str]] = []

    def fake_run(cmd, **kwargs):
        calls.append(list(cmd))
        return subprocess.CompletedProcess(cmd, 0, stdout='(true,)\n', stderr='')

    monkeypatch.setattr(window_control.shutil, 'which', lambda name: f'/usr/bin/{name}')
    monkeypatch.setattr(subprocess, 'run', fake_run)

    assert X11WindowBackend().activate('0x1') is True
    assert calls[-1] == ['wmctrl', '-i', '-a', '0x1']
    assert GnomeWaylandWindowBackend().activate('42') is True
    assert calls[-1][-2:] == [
        'org.gnome.Shell.Extensions.MobipickWinCtl.Activate',
        "'42'",
    ]
