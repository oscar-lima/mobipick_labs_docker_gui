from subprocess import CompletedProcess

import pytest

from mobipick_gui import desktop_launcher


def test_install_user_desktop_entry_uses_requested_launcher(tmp_path):
    launcher = tmp_path / 'checkout' / 'gui.py'

    desktop_file = desktop_launcher.install_user_desktop_entry(
        environ={'XDG_DATA_HOME': str(tmp_path / 'data')},
        argv0=str(launcher),
    )

    content = desktop_file.read_text(encoding='utf-8')
    assert desktop_file.name == desktop_launcher.APPLICATION_DESKTOP_FILE
    assert f'Exec="{desktop_launcher.sys.executable}" "{launcher}"\n' in content
    assert f'Icon={desktop_launcher.APPLICATION_ICON.resolve()}\n' in content
    assert 'StartupWMClass=mobipick-labs-docker-gui\n' in content


@pytest.mark.parametrize('value', ['[]', '@as []'])
def test_pin_launcher_adds_desktop_file_to_empty_favorites(monkeypatch, value):
    calls = []

    def run_gsettings(arguments):
        calls.append(arguments)
        return CompletedProcess(
            ['gsettings'],
            0,
            stdout=value if arguments[0] == 'get' else '',
            stderr='',
        )

    monkeypatch.setattr(desktop_launcher, '_run_gsettings', run_gsettings)

    assert desktop_launcher.pin_launcher_to_gnome_dock() is True
    assert calls[-1] == [
        'set',
        'org.gnome.shell',
        'favorite-apps',
        "['mobipick-labs-docker-gui.desktop']",
    ]


def test_pin_launcher_preserves_existing_favorites_and_is_idempotent(monkeypatch):
    calls = []

    def run_gsettings(arguments):
        calls.append(arguments)
        return CompletedProcess(
            ['gsettings'],
            0,
            stdout=(
                "['org.gnome.Nautilus.desktop', "
                "'mobipick-labs-docker-gui.desktop']"
            ),
            stderr='',
        )

    monkeypatch.setattr(desktop_launcher, '_run_gsettings', run_gsettings)

    assert desktop_launcher.pin_launcher_to_gnome_dock() is False
    assert len(calls) == 1


def test_install_tool_desktop_entries_writes_hidden_icon_entries(tmp_path):
    environ = {'XDG_DATA_HOME': str(tmp_path / 'data')}

    installed = desktop_launcher.install_tool_desktop_entries(environ=environ)

    applications = tmp_path / 'data' / 'applications'
    assert installed == [
        applications / 'mobipick-rviz.desktop',
        applications / 'mobipick-rqt.desktop',
        applications / 'mobipick-gazebo.desktop',
    ]
    expected_wm_classes = {
        'mobipick-rviz.desktop': 'rviz',
        'mobipick-rqt.desktop': 'python3',
        'mobipick-gazebo.desktop': 'gazebo',
    }
    for entry, desktop_file in zip(
        desktop_launcher.TOOL_DESKTOP_ENTRIES,
        installed,
    ):
        content = desktop_file.read_text(encoding='utf-8')
        assert content.startswith('[Desktop Entry]\nType=Application\n')
        assert f'Name={entry.name}\n' in content
        assert 'Exec=true\n' in content
        assert 'NoDisplay=true\n' in content
        assert 'SingleMainWindow=true\n' in content
        assert (
            f'StartupWMClass={expected_wm_classes[desktop_file.name]}\n'
            in content
        )
        assert f'Icon={entry.icon.resolve()}\n' in content
        assert entry.icon.is_file(), entry.icon


def test_install_tool_desktop_entries_is_idempotent(tmp_path):
    environ = {'XDG_DATA_HOME': str(tmp_path / 'data')}

    first = desktop_launcher.install_tool_desktop_entries(environ=environ)
    stamps = [path.stat().st_mtime_ns for path in first]
    second = desktop_launcher.install_tool_desktop_entries(environ=environ)

    assert second == first
    assert [path.stat().st_mtime_ns for path in second] == stamps


@pytest.mark.parametrize(
    'command',
    [
        'rqt',
        'rosrun rqt_gui rqt_gui --standalone dot_graph_visualization',
        'rosrun rqt_graph rqt_graph',
        'roslaunch rqt_simple_launcher_config collect_objs_moelk.launch',
        '/opt/ros/noetic/lib/rqt_gui/rqt_gui',
        'rosrun my_pkg rqt_thing.py __ns:=mobipick',
    ],
)
def test_rqt_commands_borrow_the_rqt_desktop_entry(command):
    assert desktop_launcher.desktop_entry_for_command(command) == (
        desktop_launcher.RQT_DESKTOP_ID
    )


@pytest.mark.parametrize(
    'command',
    [
        '',
        'rosrun rviz rviz -d config.rviz',
        'roslaunch tables_demo_bringup demo_sim.launch',
        'rosrun rae_upom_mobipick sfg_visualizer.py',
        'rosrun torque_pkg torqt_node',
        'python3 myrqt.py',
    ],
)
def test_other_commands_keep_their_native_window_identity(command):
    assert desktop_launcher.desktop_entry_for_command(command) is None


def test_tool_window_environment_names_the_desktop_entry_as_x11_instance():
    assert desktop_launcher.tool_window_environment(
        desktop_launcher.RQT_DESKTOP_ID
    ) == {'RESOURCE_NAME': 'mobipick-rqt'}


def test_install_desktop_launcher_installs_before_pinning(monkeypatch, tmp_path):
    events = []
    target = tmp_path / 'mobipick-labs-docker-gui.desktop'
    monkeypatch.setattr(
        desktop_launcher,
        'install_user_desktop_entry',
        lambda **_kwargs: events.append('install') or target,
    )
    monkeypatch.setattr(
        desktop_launcher,
        'install_tool_desktop_entries',
        lambda **_kwargs: events.append('tools') or [],
    )
    monkeypatch.setattr(
        desktop_launcher,
        'pin_launcher_to_gnome_dock',
        lambda: events.append('pin') or True,
    )

    assert desktop_launcher.install_desktop_launcher() == (target, True)
    assert events == ['install', 'tools', 'pin']


def test_parse_gsettings_rejects_non_string_arrays():
    with pytest.raises(OSError, match='unexpected GNOME dock favorites'):
        desktop_launcher._parse_gsettings_string_array("['valid', 3]")


def test_module_invocation_records_a_launcher_that_actually_starts(tmp_path):
    """``python -m mobipick_gui`` must not pin ``__main__.py`` itself.

    Running that file as a script fails with "attempted relative import with
    no known parent package", which leaves the dock icon doing nothing.
    """
    checkout = tmp_path / 'checkout'
    package = checkout / 'mobipick_gui'
    package.mkdir(parents=True)
    (package / '__main__.py').touch()
    shim = checkout / 'gui.py'
    shim.touch()

    command = desktop_launcher.desktop_launch_command(
        str(package / '__main__.py')
    )

    assert command == f'"{desktop_launcher.sys.executable}" "{shim}"'


def test_module_invocation_without_a_shim_falls_back_to_the_module(tmp_path):
    package = tmp_path / 'site-packages' / 'mobipick_gui'
    package.mkdir(parents=True)
    (package / '__main__.py').touch()

    command = desktop_launcher.desktop_launch_command(
        str(package / '__main__.py')
    )

    assert command == (
        f'"{desktop_launcher.sys.executable}" "-m" "mobipick_gui"'
    )
