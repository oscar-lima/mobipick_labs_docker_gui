"""Keep the GUI test suite off the developer's desktop and configuration.

``mobipick_gui.config`` reads ``~/.config/mobipick-labs-docker-gui/
gui_settings.yaml`` when it is imported.  With ``remote_control.enabled:
true`` in that file every ``MainWindow(verbosity=1)`` a test builds starts
the real remote API on the real port and glows the real GNOME dock icon
through the shell extension (``SetAppGlow`` over D-Bus), so a test run shows
up on the desktop as a light-blue halo that blinks with every test window,
and test windows save their settings into the developer's own file.

The environment is pinned here, before any test module imports the
package, so ``USER_CONFIG_FILE`` lands in a throw-away directory; the
D-Bus glow client is stubbed for every test so no call reaches GNOME Shell.
"""
from __future__ import annotations

import os
import sys
import tempfile

import pytest

if 'mobipick_gui.config' in sys.modules:  # pragma: no cover - import order guard
    raise RuntimeError(
        'mobipick_gui.config was imported before the test conftest isolated '
        'XDG_CONFIG_HOME; the suite would read the developer configuration'
    )

TEST_CONFIG_HOME = tempfile.mkdtemp(prefix='mobipick-gui-tests-config-')
os.environ['XDG_CONFIG_HOME'] = TEST_CONFIG_HOME


@pytest.fixture(autouse=True)
def _no_desktop_glow(monkeypatch):
    """Never style the real dock icon from a test.

    The stub answers nothing, so ``GnomeAppGlow`` treats the extension as
    unavailable.  Tests of the client itself replace ``_call`` again with
    their own fakes, which take precedence.
    """
    from mobipick_gui.window_control import GnomeAppGlow

    monkeypatch.setattr(GnomeAppGlow, '_call', lambda self, method, *args: None)


@pytest.fixture(autouse=True)
def _finish_host_tasks():
    """Deliver each test's background host work (mobipick-host-task threads, e.g. the docker ps a closing window
    starts) within that test: a result that reaches a closed test window during a later test can crash the run."""
    yield
    import threading
    tasks = [thread for thread in threading.enumerate() if thread.name.startswith('mobipick-host-task-')]
    for thread in tasks:
        thread.join(10)
    if tasks and 'PyQt5.QtWidgets' in sys.modules:
        from PyQt5.QtWidgets import QApplication
        app = QApplication.instance()
        if app is not None:
            app.processEvents()
