"""Action dispatcher used by the web UI server."""
from __future__ import annotations

from typing import Any, Callable

from .main_window import MainWindow
from .web_bridge import WebBridge


class WebController:
    """Dispatches actions from the web UI into the Qt GUI thread."""

    def __init__(self, window: MainWindow, bridge: WebBridge):
        self._window = window
        self._bridge = bridge

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------
    def handle(self, action: str, payload: dict[str, Any] | None = None) -> dict[str, Any]:
        payload = payload or {}
        handlers: dict[str, Callable[[dict[str, Any]], dict[str, Any]]] = {
            'toggle_roscore': self._toggle_roscore,
            'toggle_sim': self._toggle_sim,
            'toggle_tables': self._toggle_tables,
            'toggle_rviz': self._toggle_rviz,
            'toggle_rqt': self._toggle_rqt,
            'toggle_terminal': self._toggle_terminal,
            'toggle_script': self._toggle_script,
            'refresh_status': self._refresh_status,
            'refresh_images': self._refresh_images,
            'refresh_scripts': self._refresh_scripts,
            'set_combo': self._set_combo,
            'invoke_custom': self._invoke_custom,
        }
        handler = handlers.get(action)
        if handler is None:
            raise ValueError(f'Unknown action: {action}')
        return handler(payload)

    # ------------------------------------------------------------------
    # Action handlers
    # ------------------------------------------------------------------
    def _guard_busy(self, key: str) -> bool:
        state = self._window._toggle_states.get(key)
        if state == 'yellow':
            self._window._append_gui_html('log', f'<i>{key} is busy, please wait...</i>')
            return False
        return True

    def _toggle_roscore(self, _payload: dict[str, Any]) -> dict[str, Any]:
        def _do():
            if not self._guard_busy('roscore'):
                return
            self._window._log_event('web toggled roscore')
            self._window.toggle_roscore()

        self._bridge.invoke(_do)
        return {'ok': True}

    def _toggle_sim(self, _payload: dict[str, Any]) -> dict[str, Any]:
        def _do():
            if not self._guard_busy('sim'):
                return
            self._window._log_event('web toggled sim')
            self._window.toggle_sim()

        self._bridge.invoke(_do)
        return {'ok': True}

    def _toggle_tables(self, _payload: dict[str, Any]) -> dict[str, Any]:
        def _do():
            if not self._guard_busy('tables'):
                return
            self._window._log_event('web toggled tables demo')
            self._window.toggle_tables_demo()

        self._bridge.invoke(_do)
        return {'ok': True}

    def _toggle_rviz(self, _payload: dict[str, Any]) -> dict[str, Any]:
        def _do():
            if not self._guard_busy('rviz'):
                return
            self._window._log_event('web toggled rviz')
            self._window.toggle_rviz()

        self._bridge.invoke(_do)
        return {'ok': True}

    def _toggle_rqt(self, _payload: dict[str, Any]) -> dict[str, Any]:
        def _do():
            if not self._guard_busy('rqt'):
                return
            self._window._log_event('web toggled rqt tables')
            self._window.toggle_rqt_tables_demo()

        self._bridge.invoke(_do)
        return {'ok': True}

    def _toggle_terminal(self, _payload: dict[str, Any]) -> dict[str, Any]:
        def _do():
            if not self._guard_busy('terminal'):
                return
            self._window._log_event('web toggled terminal')
            self._window.toggle_terminal()

        self._bridge.invoke(_do)
        return {'ok': True}

    def _toggle_script(self, _payload: dict[str, Any]) -> dict[str, Any]:
        def _do():
            if not self._guard_busy('script'):
                return
            self._window._log_event('web toggled script execution')
            self._window.toggle_script_execution()

        self._bridge.invoke(_do)
        return {'ok': True}

    def _refresh_status(self, _payload: dict[str, Any]) -> dict[str, Any]:
        self._bridge.invoke(lambda: self._window.update_sim_status_from_poll(force=True))
        return {'ok': True}

    def _refresh_images(self, _payload: dict[str, Any]) -> dict[str, Any]:
        self._bridge.invoke(self._window._reload_images)
        return {'ok': True}

    def _refresh_scripts(self, _payload: dict[str, Any]) -> dict[str, Any]:
        self._bridge.invoke(self._window._on_refresh_scripts_clicked)
        return {'ok': True}

    def _set_combo(self, payload: dict[str, Any]) -> dict[str, Any]:
        name = payload.get('name')
        value = payload.get('value')
        if not name:
            raise ValueError('Missing combo name')

        def _do():
            combo = getattr(self._window, f'{name}_combo', None)
            if combo is None:
                raise ValueError(f'Unknown combo: {name}')
            index = combo.findText(str(value)) if value is not None else -1
            if index < 0:
                self._window._append_gui_html('log', f'<i>{name} option {value!r} not available.</i>')
                return
            combo.setCurrentIndex(index)

        self._bridge.invoke(_do)
        return {'ok': True}

    def _invoke_custom(self, payload: dict[str, Any]) -> dict[str, Any]:
        target = payload.get('target')
        if not target or not hasattr(self._window, target):
            raise ValueError('Invalid custom target')
        func = getattr(self._window, target)
        if not callable(func):
            raise ValueError('Target is not callable')
        args = payload.get('args') or []
        kwargs = payload.get('kwargs') or {}
        self._bridge.invoke(lambda: func(*args, **kwargs))
        return {'ok': True}


__all__ = ["WebController"]

