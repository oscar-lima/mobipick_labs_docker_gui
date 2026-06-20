import os
from types import MethodType

os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')

from PyQt5.QtCore import Qt
from PyQt5.QtWidgets import QApplication, QMainWindow

import mobipick_gui.main_window as main_window_module
from mobipick_gui.main_window import MainWindow


def test_save_window_state_persists_geometry(monkeypatch):
    saved_updates = []
    monkeypatch.setattr(
        main_window_module,
        'save_user_config_update',
        lambda updates: saved_updates.append(updates),
    )

    app = QApplication.instance() or QApplication([])
    window = QMainWindow()
    window._console_log = MethodType(lambda self, *_args: None, window)
    window.setGeometry(12, 34, 640, 480)

    MainWindow._save_window_state(window)

    assert saved_updates == [
        {
            'window': {
                'geometry': [12, 34, 640, 480],
                'maximized': False,
            },
        },
    ]

    window.deleteLater()
    app.processEvents()


def test_restore_window_state_applies_geometry_and_maximized():
    app = QApplication.instance() or QApplication([])
    window = QMainWindow()

    MainWindow._restore_window_state(
        window,
        {'geometry': ['20', '40', '800', '600'], 'maximized': True},
    )

    assert window.geometry().x() == 20
    assert window.geometry().y() == 40
    assert window.geometry().width() == 800
    assert window.geometry().height() == 600
    assert window.windowState() & Qt.WindowMaximized

    window.deleteLater()
    app.processEvents()
