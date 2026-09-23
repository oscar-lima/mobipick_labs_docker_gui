"""Run blocking host work without ever waiting in the Qt event thread."""
from __future__ import annotations

import itertools
import threading
from typing import Any, Callable

from PyQt5.QtCore import QObject, pyqtSignal


class AsyncTaskRunner(QObject):
    """Run host work on daemon threads and deliver results through Qt."""

    _completed = pyqtSignal(int, object, object)
    _posted = pyqtSignal(object)

    def __init__(self, parent: QObject | None = None):
        super().__init__(parent)
        self._ids = itertools.count(1)
        self._closed = False
        self._callbacks: dict[
            int,
            tuple[Callable[[Any], None] | None, Callable[[BaseException], None] | None],
        ] = {}
        self._completed.connect(self._deliver)
        self._posted.connect(lambda function: function())

    def submit(
        self,
        function: Callable[[], Any],
        *,
        on_result: Callable[[Any], None] | None = None,
        on_error: Callable[[BaseException], None] | None = None,
    ) -> int:
        task_id = next(self._ids)
        if self._closed:
            raise RuntimeError('async task runner is closed')
        self._callbacks[task_id] = (on_result, on_error)

        def run() -> None:
            try:
                result, error = function(), None
            except BaseException as exc:  # noqa: BLE001 - returned to GUI callback
                result, error = None, exc
            self._completed.emit(task_id, result, error)

        threading.Thread(
            target=run,
            name=f'mobipick-host-task-{task_id}',
            daemon=True,
        ).start()
        return task_id

    def post(self, function: Callable[[], None]) -> None:
        """Queue a callable on the object's Qt thread."""
        self._posted.emit(function)

    def _deliver(self, task_id: int, result: object, error: object) -> None:
        callbacks = self._callbacks.pop(task_id, None)
        if callbacks is None:
            return
        on_result, on_error = callbacks
        if error is not None:
            if on_error is not None:
                on_error(error)  # type: ignore[arg-type]
            return
        if on_result is not None:
            on_result(result)

    def close(self) -> None:
        """Reject new work; running jobs finish without blocking the caller."""
        self._closed = True
        self._callbacks.clear()


__all__ = ['AsyncTaskRunner']
