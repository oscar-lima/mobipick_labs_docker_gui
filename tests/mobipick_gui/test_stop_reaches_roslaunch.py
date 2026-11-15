"""#273: a GUI stop must reach roslaunch inside the container (bash as PID 1 forwards no signal, so a stop ended in
SIGKILL and the sim's nodes stayed registered), and the local master is cleaned of unreachable nodes before each sim
start and after each sim stop (a stale twin of a node name makes the master shut the new node down)."""
from types import MethodType, SimpleNamespace

from mobipick_gui.main_window import MainWindow


def _harness(**extra):
    h = SimpleNamespace(_roscore_container_name='mobipick-roscore', _docker_stop_timeout=10, **extra)
    for name in ('_safe_docker_cmd', '_interrupt_container_cmds', '_container_commands_for_ids',
                 '_wait_for_container_exit_cmd', '_docker_stop_args', '_local_master_cleanup_cmd',
                 '_cleanup_local_master', '_local_master_snapshot_cmd', '_snapshot_local_master',
                 '_sim_run_nodes'):
        setattr(h, name, MethodType(getattr(MainWindow, name), h))
    h._LOCAL_MASTER_CLEANUP_PY = MainWindow._LOCAL_MASTER_CLEANUP_PY
    h._LOCAL_MASTER_SNAPSHOT_PY = MainWindow._LOCAL_MASTER_SNAPSHOT_PY
    h._ROSLAUNCH_STARTED_RE = MainWindow._ROSLAUNCH_STARTED_RE
    h._ROSLAUNCH_SERVER_RE = MainWindow._ROSLAUNCH_SERVER_RE
    return h


def test_the_sigint_goes_to_roslaunch_first_then_to_the_container():
    cmds = _harness()._interrupt_container_cmds('abc123')
    assert cmds[0][-1].startswith('docker exec abc123 pkill -INT -x roslaunch')
    assert cmds[1][-1].startswith('docker kill -s INT abc123')
    assert all(c[:2] == ['bash', '-lc'] and c[-1].endswith('|| true') for c in cmds)   # never fails the sequence


def test_stop_sequence_order_roslaunch_container_wait_stop():
    cmds = _harness()._container_commands_for_ids(['abc123'], grace_s=20.0, include_int=True)
    shells = [c[-1] for c in cmds]
    assert shells[0].startswith('docker exec abc123 pkill -INT -x roslaunch')
    assert shells[1].startswith('docker kill -s INT abc123')
    assert shells[2].startswith('timeout 20 docker wait abc123')
    assert shells[3].startswith('docker stop --time 10 abc123')
    plain = _harness()._container_commands_for_ids(['abc123'], grace_s=0.0, include_int=False)
    assert [c[-1].split(' >')[0] for c in plain] == ['docker stop --time 10 abc123']   # no INT asked: unchanged


def test_cleanup_runs_in_the_roscore_container_bounded_and_answers_like_rosnode_cleanup():
    cmd = _harness()._local_master_cleanup_cmd()
    assert cmd[:5] == ['docker', 'exec', 'mobipick-roscore', 'bash', '-lc']
    assert 'timeout 30 python3 -c' in cmd[5] and cmd[5].endswith('|| true')
    code = MainWindow._LOCAL_MASTER_CLEANUP_PY
    compile(code, 'cleanup', 'exec')
    assert 'rosnode.rosnode_ping(name, max_count=1)' in code and 'cleanup_master_blacklist(master, dead)' in code
    assert 'ThreadPoolExecutor' in code   # all nodes at once, not 3 s each


def _cleanup_harness(remote, roscore=True):
    calls = []
    h = _harness(
        tasks={},
        _remote_master_enabled=lambda: remote,
        is_roscore_running=lambda: roscore,
        _append_gui_html=lambda key, text: calls.append(('html', key)),
        _run_command_sequence=lambda commands, log_key=None, on_finished=None: (
            calls.append(('run', commands[0][2], log_key)), on_finished and on_finished()),
    )
    return h, calls


def test_cleanup_only_against_a_running_local_master(monkeypatch):
    from mobipick_gui import main_window
    monkeypatch.setattr(main_window.QTimer, 'singleShot', lambda ms, fn: fn())
    done = []
    h, calls = _cleanup_harness(remote=False)
    h._cleanup_local_master(lambda: done.append('local'), log_key='sim')
    assert ('run', 'mobipick-roscore', 'sim') in calls and done == ['local']
    for remote, roscore in ((True, True), (False, False)):
        h, calls = _cleanup_harness(remote=remote, roscore=roscore)
        h._cleanup_local_master(lambda: done.append('skipped'))
        assert calls == []
    assert done == ['local', 'skipped', 'skipped']


def test_the_sim_starts_only_after_the_cleanup():
    order = []
    h = SimpleNamespace(
        _remote_master_enabled=lambda: False, _killing=False,
        _confirm_workspace_mismatch_warning=lambda what: True,
        set_toggle_visual=lambda *a, **k: None,
        _cleanup_local_master=lambda then, **k: (order.append('cleanup'), then()),
        _ensure_roscore_ready=lambda cb: (order.append('roscore'), cb()),
        _current_world=lambda: (_ for _ in ()).throw(RuntimeError('sim start reached')),
        _log_info=lambda text: None,
    )
    try:
        MainWindow.bring_up_sim(h)
    except RuntimeError as exc:
        order.append(str(exc))
    assert order == ['roscore', 'cleanup', 'sim start reached']


# rospy nodes keep a cached-parameter subscription after they exit (also a clean exit), so the master keeps
# their entry, which rosnode list and rosnode cleanup do not see; a new node of the same name then makes the master
# shut down whichever new node holds the old port (the recreated container has the same IP)

def test_snapshot_runs_in_the_roscore_container_bounded_and_records_name_and_address():
    cmd = _harness()._local_master_snapshot_cmd()
    assert cmd[:5] == ['docker', 'exec', 'mobipick-roscore', 'bash', '-lc']
    assert 'timeout 5 python3 -c' in cmd[5] and cmd[5].endswith('|| true')   # a hung master delays a stop 5 s at most
    code = MainWindow._LOCAL_MASTER_SNAPSHOT_PY
    compile(code, 'snapshot', 'exec')
    assert "master.lookupNode(name)" in code and "getParam('/run_id')" in code
    assert "'/tmp/mobipick_gui_master_nodes.json'" in code and "'/tmp/mobipick_gui_master_nodes.json'" in (
        MainWindow._LOCAL_MASTER_CLEANUP_PY)


def test_cleanup_purges_silent_entries_without_calling_anyone_first():
    code = MainWindow._LOCAL_MASTER_CLEANUP_PY
    compile(code, 'cleanup', 'exec')
    # the snapshot counts only for the same master run; candidates: dead nodes, snapshot, the sim run's node names
    assert "snapshot.get('run_id') == run_id" in code
    assert 'names = sorted(set(dead) | set(known) | launched)' in code
    assert "launched = set(sys.argv[2:]) if host else set()" in code   # launch names only with the sim host known
    # left alone: another host, an entry at a new address (a snapshot name that registered again)
    assert "urlparse(apis[n]).hostname != host" in code and 'apis[n] != known.get(n)' in code
    # silent = nobody there; a timeout counts only on the sim host (its container is down during the cleanup)
    assert 'GONE = (errno.ECONNREFUSED, errno.EHOSTUNREACH, errno.ENETUNREACH, errno.EHOSTDOWN)' in code
    assert "ok[n] == 'no' or (ok[n] == 'timeout' and host" in code
    assert "except socket.timeout:" in code and "return 'no' if exc.errno in GONE else 'yes'" in code
    assert code.count("return 'yes'") == 2   # an answer, or any other error: alive
    # first the call that makes the master call no one, then the dummy takeover only if the entry is still there
    first = code.index("unsubscribeParam(apis[name], '/rosout_disable_topics_generation')")
    fallback = code.index("registerSubscriber('/mobipick_gui_purge'")
    assert first < fallback and 'if api_of(name) == apis[name]:' in code[first:fallback]
    # lookups one by one (a master proxy shared by threads mixed up the answers)
    assert 'apis = {n: api_of(n) for n in names}' in code
    # what was left alone, and why, is printed
    assert 'left alone: %d answering, %d on another host' in code


def test_the_probe_timeout_is_set_after_the_ping_pass():
    # rosnode_ping sets socket.setdefaulttimeout(3.0) for the process; a stopped container's address fails at ~3 s
    code = MainWindow._LOCAL_MASTER_CLEANUP_PY
    assert code.index('socket.setdefaulttimeout(6.0)') > code.index('rosnode.rosnode_ping(name, max_count=1)')
    assert code.index('socket.setdefaulttimeout(6.0)') > code.index('cleanup_master_blacklist(master, dead)')
    assert code.index('socket.setdefaulttimeout(6.0)') < code.index('pool.map(answers,')


def test_sim_run_nodes_come_from_the_sim_tabs_log_file(tmp_path):
    log = tmp_path / 'sim.log'
    log.write_text(
        '> docker compose run ...\n'
        'started roslaunch server http://172.21.0.5:38715/\n'
        'process[mobipick/arm_controller_spawner-5]: started with pid [547]\n'
        'process[spawn_klt_1-30]: started with pid [700]\n'
        '[spawn_klt_1-30] process has finished cleanly\n'
        'process[mobipick/arm_controller_spawner-5]: started with pid [999]\n'
        '[INFO] [x] [/mobipick/move_group]: process[fake-1]: started with pid [1]\n')
    flushed = []
    writer = SimpleNamespace(path=log, is_open=True, flush=lambda: flushed.append(1))
    h = _harness(tasks={'sim': SimpleNamespace(_disk_log=writer)})
    assert h._sim_run_nodes() == ('172.21.0.5', ['/mobipick/arm_controller_spawner', '/spawn_klt_1'])
    assert flushed == [1]
    assert _harness(tasks={})._sim_run_nodes() == ('', [])
    missing = SimpleNamespace(path=tmp_path / 'gone.log', is_open=False)
    assert _harness(tasks={'sim': SimpleNamespace(_disk_log=missing)})._sim_run_nodes() == ('', [])


def test_cleanup_command_passes_the_sim_host_and_names_quoted():
    cmd = _harness()._local_master_cleanup_cmd('172.21.0.5', ['/a', '/mobipick/b'])
    assert " 172.21.0.5 /a /mobipick/b 2>&1 |" in cmd[5]
    assert " '' 2>&1 |" in _harness()._local_master_cleanup_cmd()[5]   # nothing known: an empty host, no names


def test_snapshot_only_against_a_running_local_master(monkeypatch):
    from mobipick_gui import main_window
    monkeypatch.setattr(main_window.QTimer, 'singleShot', lambda ms, fn: fn())
    done = []
    h, calls = _cleanup_harness(remote=False)
    h._snapshot_local_master(lambda: done.append('local'), log_key='sim')
    assert ('run', 'mobipick-roscore', 'sim') in calls and done == ['local']
    for remote, roscore in ((True, True), (False, False)):
        h, calls = _cleanup_harness(remote=remote, roscore=roscore)
        h._snapshot_local_master(lambda: done.append('skipped'))
        assert calls == []
    assert done == ['local', 'skipped', 'skipped']


def test_sim_stop_takes_the_snapshot_before_the_container_stop_and_cleans_after(monkeypatch):
    from mobipick_gui import main_window
    monkeypatch.setattr(main_window.QTimer, 'singleShot', lambda ms, fn: fn())
    order = []
    tab = SimpleNamespace(run_generation=1, exec_id='e1', key='sim')
    h = SimpleNamespace(
        _sim_container_name='mobipick-run', _killing=False,
        _log_info=lambda text: None, set_toggle_visual=lambda *a, **k: None,
        _ensure_tab=lambda *a, **k: tab,
        _snapshot_local_master=lambda then, **k: (order.append('snapshot'), then()),
        _graceful_stop_container=lambda name, t, exec_id=None, on_finished=None: (
            order.append('stop ' + name), on_finished()),
        _release_xhost=lambda *a, **k: None, _reap_detached_client=lambda t: None,
        _cleanup_local_master=lambda *a, **k: order.append('cleanup'),
    )
    MainWindow.shutdown_sim(h)
    assert order == ['snapshot', 'stop mobipick-run', 'cleanup']
