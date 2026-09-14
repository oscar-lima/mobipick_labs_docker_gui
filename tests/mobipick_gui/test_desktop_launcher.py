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
        'pin_launcher_to_gnome_dock',
        lambda: events.append('pin') or True,
    )

    assert desktop_launcher.install_desktop_launcher() == (target, True)
    assert events == ['install', 'pin']


def test_parse_gsettings_rejects_non_string_arrays():
    with pytest.raises(OSError, match='unexpected GNOME dock favorites'):
        desktop_launcher._parse_gsettings_string_array("['valid', 3]")
