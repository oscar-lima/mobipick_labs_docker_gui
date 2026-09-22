"""Window helpers shared by the Qt widgets."""
from __future__ import annotations

from typing import Mapping

from PyQt5.QtCore import QEvent, QObject, QPoint, QRect, QTimer, Qt
from PyQt5.QtGui import QGuiApplication
from PyQt5.QtWidgets import QDialog as QtDialog
from PyQt5.QtWidgets import QWidget

from .config import CONFIG, save_user_config_update
from .window_control import session_type


def available_geometry(
    window: QWidget,
    position: QPoint | None = None,
) -> QRect | None:
    """Return the usable area of the screen a window should appear on."""
    screen = None
    if position is not None:
        screen = QGuiApplication.screenAt(position)
    if screen is None:
        screen = window.screen() if hasattr(window, 'screen') else None
    if screen is None:
        screen = QGuiApplication.primaryScreen()
    if screen is None:
        return None
    bounds = screen.availableGeometry()
    return bounds if bounds.isValid() else None


def fit_geometry_to_screen(
    window: QWidget,
    x: int,
    y: int,
    width: int,
    height: int,
) -> tuple[int, int, int, int]:
    """Shrink and move a saved geometry until it fits the current screen.

    Geometry saved on a large external monitor is usually both too big and
    off-screen for a laptop panel, which leaves the window unreachable and
    impossible to resize with the window manager.
    """
    bounds = available_geometry(window, QPoint(int(x), int(y)))
    if bounds is None:
        return x, y, width, height
    frame = window.frameGeometry()
    inner = window.geometry()
    frame_width = max(0, frame.width() - inner.width())
    frame_height = max(0, frame.height() - inner.height())
    width = max(1, min(width, bounds.width() - frame_width))
    height = max(1, min(height, bounds.height() - frame_height))
    x = min(
        max(x, bounds.x()),
        bounds.x() + bounds.width() - frame_width - width,
    )
    y = min(
        max(y, bounds.y()),
        bounds.y() + bounds.height() - frame_height - height,
    )
    return x, y, width, height


def restore_window_geometry(
    window: QWidget,
    state: Mapping | None,
    *,
    desktop_session: str | None = None,
) -> bool:
    """Restore a window's normal geometry and return its maximized state.

    Wayland compositors own top-level window placement, so only restore the
    saved size there. X11 permits restoring the full position and size.
    """
    state = state if isinstance(state, Mapping) else {}
    geometry = state.get('geometry', [])
    if isinstance(geometry, (list, tuple)) and len(geometry) == 4:
        try:
            x, y, width, height = (int(value) for value in geometry)
        except (TypeError, ValueError):
            pass
        else:
            if width > 0 and height > 0:
                x, y, width, height = fit_geometry_to_screen(
                    window, x, y, width, height
                )
                current_session = desktop_session or session_type()
                if current_session == 'wayland':
                    window.resize(width, height)
                else:
                    window.setGeometry(x, y, width, height)
    return bool(state.get('maximized'))


def saved_window_state(
    window: QWidget,
    previous_state: Mapping | None = None,
    *,
    desktop_session: str | None = None,
) -> dict:
    """Return serializable normal geometry and window-manager state."""
    geometry = window.normalGeometry()
    if geometry.isNull():
        geometry = window.geometry()
    x = geometry.x()
    y = geometry.y()
    current_session = desktop_session or session_type()
    if current_session == 'wayland' and isinstance(previous_state, Mapping):
        previous_geometry = previous_state.get('geometry', [])
        if (
            isinstance(previous_geometry, (list, tuple))
            and len(previous_geometry) == 4
        ):
            try:
                x, y = int(previous_geometry[0]), int(previous_geometry[1])
            except (TypeError, ValueError):
                pass
    return {
        'geometry': [x, y, geometry.width(), geometry.height()],
        'maximized': window.isMaximized(),
    }


class _MaximizeOnExpose(QObject):
    """Apply a maximize request once the native window has been exposed."""

    def __init__(self, window: QWidget):
        super().__init__(window)
        self._window = window
        self._handle = window.windowHandle()
        if self._handle is not None:
            self._handle.installEventFilter(self)

    def eventFilter(self, watched: QObject, event: QEvent) -> bool:  # noqa: N802
        if watched is self._handle and event.type() == QEvent.Expose:
            self._queue_maximize()
        return False

    def start(self) -> None:
        """Handle a window exposed before the filter was installed."""
        if self._handle is not None and self._handle.isExposed():
            self._queue_maximize()

    def _queue_maximize(self) -> None:
        if self._handle is not None:
            self._handle.removeEventFilter(self)
            self._handle = None
        QTimer.singleShot(0, self._maximize)

    def _maximize(self) -> None:
        if self._window.isVisible():
            self._window.showMaximized()
        if getattr(self._window, '_maximize_on_expose', None) is self:
            self._window._maximize_on_expose = None
        self.deleteLater()


def maximize_after_window_is_exposed(window: QWidget) -> None:
    """Maximize ``window`` after its native surface is ready for the request.

    A zero-delay callback after ``show()`` can still run before an X11 window
    manager has mapped the native window. Waiting for Qt's expose event makes
    the restore reliable without imposing an arbitrary machine-dependent
    delay, and uses the same path on Wayland.
    """
    pending = _MaximizeOnExpose(window)
    # Keep the filter alive until it has handled the expose event. Parenting
    # alone owns the C++ object, but this Python reference is also required.
    window._maximize_on_expose = pending
    pending.start()


class PersistentWindowStateMixin:
    """Remember geometry and maximized state for a reusable top-level window."""

    persistent_window_key: str | None = None

    def _persistent_state_key(self) -> str:
        explicit_key = self.property('persistentWindowKey')
        if explicit_key:
            return str(explicit_key)
        if self.persistent_window_key:
            return self.persistent_window_key
        if type(self) is MaximizableDialog and self.windowTitle():
            return f'dialog.{self.windowTitle()}'
        return f'{type(self).__module__}.{type(self).__qualname__}'

    def _persistent_state(self) -> Mapping:
        states = CONFIG.get('window_states', {})
        if not isinstance(states, Mapping):
            return {}
        state = states.get(self._persistent_state_key(), {})
        return state if isinstance(state, Mapping) else {}

    def _restore_persistent_window_state(self) -> None:
        if getattr(self, '_persistent_window_state_restored', False):
            return
        self._persistent_window_state_restored = True
        self._persistent_restore_maximized = restore_window_geometry(
            self,
            self._persistent_state(),
        )

    def _save_persistent_window_state(self) -> None:
        if not getattr(self, '_persistent_window_state_restored', False):
            return
        key = self._persistent_state_key()
        state = saved_window_state(self, self._persistent_state())
        try:
            save_user_config_update({'window_states': {key: state}})
        except OSError:
            # Window shutdown should never be blocked by an unwritable config.
            pass

    def showEvent(self, event: QEvent) -> None:  # noqa: N802 - Qt API
        self._restore_persistent_window_state()
        super().showEvent(event)
        if getattr(self, '_persistent_restore_maximized', False):
            self._persistent_restore_maximized = False
            maximize_after_window_is_exposed(self)

    def hideEvent(self, event: QEvent) -> None:  # noqa: N802 - Qt API
        self._save_persistent_window_state()
        super().hideEvent(event)


def configure_maximizable_window(window: QWidget) -> None:
    """Give a top-level window standard minimize/maximize controls."""
    window.setWindowFlag(Qt.WindowMinimizeButtonHint, True)
    window.setWindowFlag(Qt.WindowMaximizeButtonHint, True)
    window.setWindowFlag(Qt.WindowCloseButtonHint, True)
    if isinstance(window, QtDialog):
        window.setSizeGripEnabled(True)


class MaximizableDialog(PersistentWindowStateMixin, QtDialog):
    """Persistent QDialog with normal window-manager maximize affordances."""

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        configure_maximizable_window(self)
