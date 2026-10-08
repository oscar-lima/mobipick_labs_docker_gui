"""Shared pytest configuration for the GUI test suite."""
from __future__ import annotations

import pytest


@pytest.fixture(autouse=True)
def _isolated_user_data_dir(tmp_path_factory, monkeypatch):
    """Keep GUI data (log files, recordings) out of the developer's home.

    ``default_user_data_dir`` honours ``XDG_DATA_HOME`` at call time, so the
    main window writes its session logs under a temporary directory. Tests
    that need a specific location set the variable themselves.
    """
    data_home = tmp_path_factory.mktemp('xdg-data-home')
    monkeypatch.setenv('XDG_DATA_HOME', str(data_home))
