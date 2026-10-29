"""A window whose exit has begun is no longer the application's event filter: PyQt would wrap every object that gets
an event for it, also widgets in the middle of their destruction."""
import os

os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')

from PyQt5.QtCore import QEvent, QObject
from PyQt5.QtWidgets import QApplication

from test_remote_ros_master import _create_window


def test_the_application_event_filter_goes_when_the_exit_begins(monkeypatch, tmp_path):
    app, window = _create_window(monkeypatch, tmp_path)
    seen = []
    monkeypatch.setattr(window, 'eventFilter', lambda watched, event: seen.append(event.type()) or False)
    other = QObject()
    QApplication.sendEvent(other, QEvent(QEvent.User))
    assert QEvent.User in seen   # a live window filters the whole application (Ctrl+W)

    window.close()   # begins the exit sequence
    seen.clear()
    QApplication.sendEvent(other, QEvent(QEvent.User))
    assert QEvent.User not in seen
    app.processEvents()
