"""Run blocking host work without ever waiting in the Qt event thread."""
from __future__ import annotations

import itertools
import threading
from typing import Any, Callable

from PyQt5.QtCore import QObject, pyqtSignal


class _Liveness:
    """Whether the runner may still be signalled, shared with its task threads (they must not keep the runner's
    lifetime). The lock makes a thread's emit and the runner's destruction take turns."""

    def __init__(self) -> None:
        self.lock = threading.Lock()
        self.alive = True
        self.owner = threading.get_ident()   # the runner's own (GUI) thread

    def end(self, *_args) -> None:
        with self.lock:
            self.alive = False


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
        # a task that ends after the window (and with it this runner) was closed or deleted is dropped; emitting
        # into a runner that is being destroyed on the GUI thread could crash the process
        self._liveness = _Liveness()
        self.destroyed.connect(self._liveness.end)

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
            self._emit_if_alive('_completed', task_id, result, error)

        threading.Thread(
            target=run,
            name=f'mobipick-host-task-{task_id}',
            daemon=True,
        ).start()
        return task_id

    def post(self, function: Callable[[], None]) -> None:
        """Queue a callable on the object's Qt thread (dropped once the runner is closed or gone)."""
        self._emit_if_alive('_posted', function)

    def _emit_if_alive(self, signal: str, *args) -> None:
        """Emit ``signal`` (looked up only here: on a deleted runner even the lookup fails) unless the runner is
        closed or gone."""
        liveness = self._liveness
        if threading.get_ident() == liveness.owner:
            # the runner's own thread cannot destroy it meanwhile; no lock, so a slot Qt calls directly here may
            # close the window or post again
            if liveness.alive:
                getattr(self, signal).emit(*args)
            return
        with liveness.lock:
            if not liveness.alive:
                return
            try:
                getattr(self, signal).emit(*args)
            except RuntimeError:   # the wrapper was marked deleted just before ``destroyed`` cleared the flag
                pass

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
        self._liveness.end()


__all__ = ['AsyncTaskRunner']
