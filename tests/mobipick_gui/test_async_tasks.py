"""AsyncTaskRunner: a task that ends after its runner is gone (the window closed or is being torn down) is dropped
quietly, without an exception in its thread and without racing the runner's destruction."""
import os
import threading

os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')

from PyQt5 import sip
from PyQt5.QtCore import QObject
from PyQt5.QtWidgets import QApplication

from mobipick_gui.async_tasks import AsyncTaskRunner


def _tasks():
    return [thread for thread in threading.enumerate() if thread.name.startswith('mobipick-host-task-')]


def test_a_result_reaches_its_callback():
    app = QApplication.instance() or QApplication([])
    runner = AsyncTaskRunner()
    results = []
    runner.submit(lambda: 42, on_result=results.append)
    for thread in _tasks():
        thread.join(5)
    app.processEvents()
    assert results == [42]


def test_a_task_that_ends_after_its_runner_was_deleted_is_dropped_quietly(monkeypatch):
    app = QApplication.instance() or QApplication([])
    errors = []
    monkeypatch.setattr(threading, 'excepthook', lambda args: errors.append(args.exc_value))
    for _round in range(20):
        parent = QObject()
        runner = AsyncTaskRunner(parent)
        release = threading.Event()
        results = []
        runner.submit(lambda: release.wait(5) or 'late', on_result=results.append)
        sip.delete(parent)   # the window goes while the task still runs; the runner goes with it
        release.set()
        for thread in _tasks():
            thread.join(5)
        app.processEvents()
        assert results == [] and errors == []


def test_tasks_ending_while_their_runner_is_being_deleted_do_not_crash(monkeypatch):
    app = QApplication.instance() or QApplication([])
    errors = []
    monkeypatch.setattr(threading, 'excepthook', lambda args: errors.append(args.exc_value))
    for _round in range(200):
        parent = QObject()
        runner = AsyncTaskRunner(parent)
        start = threading.Event()
        for _task in range(4):
            runner.submit(lambda: start.wait(5))
        start.set()   # the tasks end and report while the runner is deleted on this thread
        sip.delete(parent)
        for thread in _tasks():
            thread.join(5)
        app.processEvents()
    assert errors == []


def test_post_on_the_gui_thread_runs_at_once_and_may_close_the_runner():
    app = QApplication.instance() or QApplication([])
    parent = QObject()
    runner = AsyncTaskRunner(parent)
    ran = []
    runner.post(lambda: (ran.append(1), sip.delete(parent)))   # a slot that tears the window down must not deadlock
    assert ran == [1] and sip.isdeleted(parent)
    app.processEvents()
