from mobipick_gui.log_files import (
    LogFileManager,
    LogFileWriter,
    ansi_to_plain,
    html_to_plain,
)


def test_html_to_plain_strips_tags_and_entities():
    assert html_to_plain('<span style="color:#fff"><i>&gt; ls -la</i></span><br>') == '> ls -la\n'
    assert html_to_plain('a&nbsp;b') == 'a b'


def test_ansi_to_plain_strips_sgr_and_osc_sequences():
    assert ansi_to_plain('\x1b[32mok\x1b[0m \x1b]0;title\x07done') == 'ok done'


def test_writer_flushes_at_most_every_interval(tmp_path):
    now = [0.0]
    writer = LogFileWriter(tmp_path / 'a.log', flush_interval_s=1.0, clock=lambda: now[0])
    writer.write('first')
    assert (tmp_path / 'a.log').read_text() == ''
    now[0] = 1.0
    writer.write('second\n')
    assert (tmp_path / 'a.log').read_text() == 'first\nsecond\n'
    writer.write('third')
    writer.flush()
    assert (tmp_path / 'a.log').read_text() == 'first\nsecond\nthird\n'
    writer.close()
    assert not writer.is_open


def test_writer_reports_errors_once_and_disables_itself(tmp_path):
    errors = []
    blocker = tmp_path / 'file'
    blocker.write_text('x')
    writer = LogFileWriter(blocker / 'sub' / 'a.log', on_error=errors.append)
    assert writer.failed
    assert not writer.is_open
    assert len(errors) == 1
    writer.write('ignored')
    assert len(errors) == 1


def test_manager_paths_follow_the_session_layout(tmp_path):
    manager = LogFileManager(tmp_path / 'logs', session='20260101-120000')
    assert manager.gui_log_path == tmp_path / 'logs' / 'gui-20260101-120000.log'
    gui = manager.gui_writer()
    assert manager.gui_writer() is gui
    tab = manager.open_tab_log('Tables Demo')
    assert tab.path.parent == tmp_path / 'logs' / '20260101-120000'
    assert tab.path.name.startswith('Tables_Demo-')
    assert tab.path.suffix == '.log'
    again = manager.open_tab_log('Tables Demo')
    assert again.path != tab.path
    gui.write('hello')
    manager.flush_all()
    assert manager.gui_log_path.read_text() == 'hello\n'
    manager.close_all()
    assert not gui.is_open and not tab.is_open


def test_prune_keeps_only_the_newest_sessions(tmp_path):
    root = tmp_path / 'logs'
    root.mkdir()
    for day in range(1, 6):
        session = f'2026010{day}-120000'
        (root / session).mkdir()
        (root / session / 'sim-120001.log').write_text('x')
        (root / f'gui-{session}.log').write_text('x')
    (root / 'unrelated.txt').write_text('keep me')

    manager = LogFileManager(root, keep_sessions=3, session='20260106-120000')
    removed = manager.prune()

    assert removed == ['20260101-120000', '20260102-120000', '20260103-120000']
    assert manager.sessions() == ['20260104-120000', '20260105-120000']
    assert (root / 'unrelated.txt').exists()
    assert not (root / 'gui-20260101-120000.log').exists()


def test_main_window_writes_its_log_tab_to_a_session_file(tmp_path, monkeypatch):
    import os

    os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')
    from PyQt5.QtWidgets import QApplication

    from mobipick_gui.config import CONFIG
    from mobipick_gui.main_window import MainWindow

    monkeypatch.setenv('XDG_DATA_HOME', str(tmp_path / 'data'))
    monkeypatch.setenv('MOBIPICK_WORKSPACE_CONFIG', str(tmp_path / 'workspaces.yaml'))
    monkeypatch.setattr(
        MainWindow,
        '_discover_filtered_image_records',
        lambda self: ([{'ref': CONFIG['images']['default']}], None),
    )
    monkeypatch.setattr(
        MainWindow,
        'update_sim_status_from_poll',
        lambda self, force=False: None,
    )

    app = QApplication.instance() or QApplication([])
    window = MainWindow(verbosity=1)
    window.poll_timer.stop()
    window._sigint_timer.stop()
    try:
        logs_dir = tmp_path / 'data' / 'mobipick-labs-docker-gui' / 'logs'
        assert window._log_files is not None
        assert window._log_files.root == logs_dir

        window._log_info('hello from the test')
        window._log_files.flush_all()

        gui_logs = sorted(logs_dir.glob('gui-*.log'))
        assert len(gui_logs) == 1
        content = gui_logs[0].read_text(encoding='utf-8')
        assert f'logs are written to {logs_dir}' in content
        assert '[INFO] hello from the test' in content
        assert '<' not in content
        assert window.tasks['log'].disk_log_path() == gui_logs[0]
    finally:
        window._close_log_files()
        window.deleteLater()
        app.processEvents()
