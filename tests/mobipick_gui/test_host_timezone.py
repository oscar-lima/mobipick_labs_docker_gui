"""Containers get the host's time zone, so the local times they print, speak or
put into file names match the clock on the wall instead of UTC."""

import os

from mobipick_gui.config import CONFIG
from mobipick_gui import main_window as main_window_module
from mobipick_gui.host_timezone import host_timezone_environment, host_timezone_name

from test_remote_ros_master import _create_window


def _localtime_link(tmp_path, target):
    link = tmp_path / 'localtime'
    os.symlink(target, link)
    return link


def test_the_zone_comes_from_tz_then_the_localtime_link_then_etc_timezone(tmp_path):
    timezone_file = tmp_path / 'timezone'
    timezone_file.write_text('America/New_York\n', encoding='utf-8')
    link = _localtime_link(tmp_path, '/usr/share/zoneinfo/Europe/Berlin')
    missing = tmp_path / 'missing'

    def name(environ, localtime=link, timezone=timezone_file):
        return host_timezone_name(environ=environ, timezone_file=timezone, localtime_file=localtime)

    assert name({'TZ': 'Asia/Tokyo'}) == 'Asia/Tokyo'
    assert name({}) == 'Europe/Berlin'
    assert name({}, localtime=missing) == 'America/New_York'
    assert name({}, localtime=missing, timezone=missing) == ''


def test_a_tz_path_is_never_passed_because_it_would_name_the_containers_own_utc_file(tmp_path):
    link = _localtime_link(tmp_path, '/usr/share/zoneinfo/Europe/Berlin')
    for path_value in (':/etc/localtime', '/etc/localtime', '  '):
        assert host_timezone_name(
            environ={'TZ': path_value}, timezone_file=tmp_path / 'missing', localtime_file=link,
        ) == 'Europe/Berlin'
    assert host_timezone_environment(
        environ={}, timezone_file=tmp_path / 'missing', localtime_file=tmp_path / 'missing',
    ) == {}


def test_every_container_the_gui_starts_gets_the_host_zone_and_a_configured_tz_wins(tmp_path, monkeypatch):
    app, window = _create_window(monkeypatch, tmp_path)
    monkeypatch.setattr(main_window_module, 'host_timezone_environment', lambda: {'TZ': 'Europe/Berlin'})

    assert 'TZ=Europe/Berlin' in window._compose_env_args()

    configured = dict(CONFIG['process']['compose_run_env'], TZ='Etc/UTC')
    monkeypatch.setitem(CONFIG['process'], 'compose_run_env', configured)
    env_args = window._compose_env_args()
    assert 'TZ=Etc/UTC' in env_args and 'TZ=Europe/Berlin' not in env_args
    assert 'TZ=Asia/Tokyo' in window._compose_env_args({'TZ': 'Asia/Tokyo'})

    window.deleteLater()
    app.processEvents()
