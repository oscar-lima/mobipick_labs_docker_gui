"""Shared pytest configuration for the GUI test suite."""
from __future__ import annotations

import os
import stat
import tempfile

import pytest

# Every test window is a real MainWindow: closing it runs the exit sequence, which stops the workspace containers by
# their fixed names (mobipick-roscore, mobipick-run, ...), and some tests start a roscore. The containers belong to
# the one Docker daemon of the machine, so a test run would stop a simulation another user or agent runs. A fake
# ``docker`` first on PATH (it only logs its arguments to DOCKER_CALLS_LOG and fails) and a DOCKER_HOST without a
# daemon keep the whole suite off the real daemon, also in the ``bash -lc`` commands the GUI runs.
_FAKE_DOCKER_DIR = tempfile.mkdtemp(prefix='mobipick-gui-tests-docker-')
DOCKER_CALLS_LOG = os.path.join(_FAKE_DOCKER_DIR, 'calls.log')
_fake_docker = os.path.join(_FAKE_DOCKER_DIR, 'docker')
with open(_fake_docker, 'w', encoding='utf-8') as _file:
    _file.write(f'#!/bin/sh\necho "$@" >> "{DOCKER_CALLS_LOG}"\nexit 1\n')
os.chmod(_fake_docker, os.stat(_fake_docker).st_mode | stat.S_IXUSR)
os.environ['PATH'] = _FAKE_DOCKER_DIR + os.pathsep + os.environ.get('PATH', '')
os.environ['DOCKER_HOST'] = 'unix://' + os.path.join(_FAKE_DOCKER_DIR, 'no-daemon.sock')


@pytest.fixture(autouse=True)
def _isolated_user_data_dir(tmp_path_factory, monkeypatch):
    """Keep GUI data (log files, recordings) out of the developer's home.

    ``default_user_data_dir`` honours ``XDG_DATA_HOME`` at call time, so the
    main window writes its session logs under a temporary directory. Tests
    that need a specific location set the variable themselves.
    """
    data_home = tmp_path_factory.mktemp('xdg-data-home')
    monkeypatch.setenv('XDG_DATA_HOME', str(data_home))
