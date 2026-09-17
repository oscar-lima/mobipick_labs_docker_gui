"""Window helpers shared by the Qt widgets."""
from __future__ import annotations

from typing import Mapping

from PyQt5.QtCore import QEvent, QTimer, Qt
from PyQt5.QtWidgets import QDialog as QtDialog
from PyQt5.QtWidgets import QWidget

from .config import CONFIG, save_user_config_update
from .window_control import session_type


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
            QTimer.singleShot(0, self.showMaximized)

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
