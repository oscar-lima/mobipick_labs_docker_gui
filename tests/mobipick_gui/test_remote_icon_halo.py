"""The window icon halo follows the remote API state and nothing else.

Design (README "Window icon glow"): no halo while the remote API is off;
a static light-blue halo while it listens with nobody present; green once a
client declared presence; a pulse only while a request is being served.
A test run must never drive the halo of the developer's own GUI.
"""
from __future__ import annotations

from pathlib import Path

import pytest


def _plain_window(tmp_path, monkeypatch, **kwargs):
    from PyQt5.QtWidgets import QApplication

    from mobipick_gui.config import CONFIG
    from mobipick_gui.main_window import MainWindow

    monkeypatch.setenv('MOBIPICK_WORKSPACE_CONFIG', str(tmp_path / 'workspaces.yaml'))
    monkeypatch.setitem(CONFIG, 'selections', {})
    monkeypatch.setattr(
        MainWindow,
        '_discover_filtered_image_records',
        lambda self: ([{'ref': CONFIG['images']['default']}], None),
    )
    monkeypatch.setattr(MainWindow, 'update_sim_status_from_poll', lambda self, force=False: None)
    app = QApplication.instance() or QApplication([])
    window = MainWindow(verbosity=1, **kwargs)
    window.poll_timer.stop()
    window._sigint_timer.stop()
    return app, window


def test_suite_never_reads_the_developer_configuration():
    from mobipick_gui.config import CONFIG, USER_CONFIG_FILE

    assert not str(USER_CONFIG_FILE).startswith(str(Path.home() / '.config'))
    # The packaged default: the API, and with it the halo, is off.
    assert not CONFIG.get('remote_control', {}).get('enabled')


def test_default_window_has_no_halo_and_no_api(tmp_path, monkeypatch):
    """``MainWindow(verbosity=1)`` as most tests build it: no API, no glow."""
    app, window = _plain_window(tmp_path, monkeypatch)
    try:
        assert window.remote_control is None
        assert window._remote_icon_timer is None
        assert window._remote_icon_base is None
        assert window._remote_app_glow is None
    finally:
        window._stop_remote_control()
        window.deleteLater()
        app.processEvents()


def test_halo_is_static_without_a_client_and_never_reaches_the_desktop(tmp_path, monkeypatch):
    from mobipick_gui import main_window as mw
    from mobipick_gui.window_control import GnomeAppGlow

    dbus_calls: list[tuple] = []

    def fake_call(self, method, *args):
        dbus_calls.append((method, args))
        return (4,) if method == 'Version' else (1,)

    monkeypatch.setattr(GnomeAppGlow, '_call', fake_call)
    app, window = _plain_window(
        tmp_path,
        monkeypatch,
        remote_control={'enabled': True, 'host': '127.0.0.1', 'port': 0},
    )
    try:
        server = window.remote_control
        assert server is not None and server.running
        assert server.active_requests == 0 and not server.clients()
        idle = (
            round(mw.REMOTE_ICON_GLOW_IDLE * mw.REMOTE_ICON_GLOW_LEVELS),
            mw.REMOTE_ICON_GLOW_COLOR.rgb(),
        )
        assert window._remote_icon_level == idle
        # Many ticks (more than a full pulse period) with nobody connected
        # and nothing served: the level never changes, so the icon never
        # blinks.
        seen = set()
        for _ in range(int(2 * mw.REMOTE_ICON_GLOW_PULSE_S * 1000 / mw.REMOTE_ICON_GLOW_TICK_MS)):
            window._update_remote_icon_glow()
            seen.add(window._remote_icon_level)
        assert seen == {idle}
        assert window._remote_icon_pulse_started == 0.0
    finally:
        window._stop_remote_control()
        window.deleteLater()
        app.processEvents()
    # The glow client was created, but this test process only ever talked
    # to the stub, never to GNOME Shell: the stubbed probe answered, and only
    # one level (plus the final clear) was sent, nothing repeated.
    glow_levels = [args[1] for method, args in dbus_calls if method == 'SetAppGlow']
    idle_fraction = idle[0] / mw.REMOTE_ICON_GLOW_LEVELS
    assert glow_levels in ([], [0.0], [idle_fraction, 0.0]), glow_levels


@pytest.mark.parametrize('kwargs', [{}, {'remote_control': {'enabled': False}}])
def test_window_never_contacts_gnome_shell_when_api_is_off(tmp_path, monkeypatch, kwargs):
    from mobipick_gui.window_control import GnomeAppGlow

    calls: list[str] = []
    monkeypatch.setattr(
        GnomeAppGlow, '_call', lambda self, method, *args: calls.append(method) or (4,)
    )
    app, window = _plain_window(tmp_path, monkeypatch, **kwargs)
    try:
        assert window.remote_control is None
    finally:
        window._stop_remote_control()
        window.deleteLater()
        app.processEvents()
    assert calls == []
