import os
import time

from PyQt5.QtWidgets import QApplication, QWidget


def _pump(condition, timeout_s: float = 3.0) -> None:
    app = QApplication.instance() or QApplication([])
    deadline = time.monotonic() + timeout_s
    while not condition() and time.monotonic() < deadline:
        app.processEvents()
        app.sendPostedEvents()
        time.sleep(0.02)
    app.processEvents()

from mobipick_gui.main_window import MainWindow
from mobipick_gui.window_control import WindowInfo


class FakeBackend:
    available = True

    def __init__(self, found_after: int = 0):
        self.found_after = found_after
        self.list_calls = 0
        self.above: list[tuple[str, bool]] = []
        self.activated: list[str] = []

    def list_windows(self, **kwargs):
        self.list_calls += 1
        if self.list_calls <= self.found_after:
            return []
        return [WindowInfo('77', 'Auto Launch Progress', 0, os.getpid(), 0, 0, 1, 1, [])]

    def set_above(self, wid, above=True):
        self.above.append((wid, above))
        return True

    def activate(self, wid):
        self.activated.append(wid)
        return True


class FakeManager:
    def __init__(self, backend):
        self.backend = backend


def _make_window(monkeypatch, backend, session='wayland'):
    monkeypatch.setenv('XDG_SESSION_TYPE', session)
    window = MainWindow.__new__(MainWindow)
    window._window_layout_manager = FakeManager(backend)
    logs: list[tuple[int, str]] = []
    window._console_log = lambda level, msg: logs.append((level, msg))
    return window, logs


def test_keep_window_above_retries_until_window_appears(monkeypatch):
    app = QApplication.instance() or QApplication([])  # noqa: F841 keep alive
    backend = FakeBackend(found_after=2)
    window, logs = _make_window(monkeypatch, backend)
    widget = QWidget()
    widget.setWindowTitle('Auto Launch Progress')
    widget.show()

    window.keep_window_above(widget)
    _pump(lambda: backend.above == [('77', True)])
    widget.close()
    assert backend.above == [('77', True)]
    assert backend.list_calls == 3
    assert any('pinned window' in msg for _, msg in logs)


def test_keep_window_above_is_noop_on_x11(monkeypatch):
    app = QApplication.instance() or QApplication([])  # noqa: F841 keep alive
    backend = FakeBackend()
    window, _logs = _make_window(monkeypatch, backend, session='x11')
    widget = QWidget()
    widget.setWindowTitle('Auto Launch Progress')
    widget.show()

    window.keep_window_above(widget)
    _pump(lambda: False, timeout_s=0.1)
    widget.close()
    assert backend.above == []
    assert backend.list_calls == 0


def test_keep_window_above_skips_hidden_widget(monkeypatch):
    app = QApplication.instance() or QApplication([])  # noqa: F841 keep alive
    backend = FakeBackend()
    window, _logs = _make_window(monkeypatch, backend)
    widget = QWidget()
    widget.setWindowTitle('Auto Launch Progress')

    window.keep_window_above(widget)  # never shown
    _pump(lambda: False, timeout_s=0.1)
    assert backend.above == []


def test_bring_window_to_front_uses_compositor_on_wayland(monkeypatch):
    app = QApplication.instance() or QApplication([])  # noqa: F841 keep alive
    backend = FakeBackend()
    window, _logs = _make_window(monkeypatch, backend)
    widget = QWidget()
    widget.setWindowTitle('Auto Launch Progress')
    widget.show()

    window.bring_window_to_front(widget)
    _pump(lambda: backend.activated == ['77'])
    widget.close()

    assert backend.activated == ['77']
