import subprocess

from mobipick_gui import window_layout
from mobipick_gui.window_layout import WindowLayoutManager


def test_record_baseline_skips_missing_wmctrl(tmp_path, monkeypatch):
    def fake_which(name):
        return None if name == 'missing-wmctrl' else f'/usr/bin/{name}'

    def fail_run(*args, **kwargs):
        raise AssertionError('wmctrl should not be executed when unavailable')

    warnings: list[str] = []

    monkeypatch.setattr(window_layout.shutil, 'which', fake_which)
    monkeypatch.setattr(subprocess, 'run', fail_run)

    manager = WindowLayoutManager(
        tmp_path / 'window_layout.yaml',
        wmctrl_bin='missing-wmctrl',
        log_warning=warnings.append,
    )

    manager.record_baseline()

    assert warnings == []
    assert manager._auto_apply_done is True
