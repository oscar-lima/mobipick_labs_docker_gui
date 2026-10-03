"""#273: a GUI stop must reach roslaunch inside the container (bash as PID 1 forwards no signal, so a stop ended in
SIGKILL and the sim's nodes stayed registered), and the local master is cleaned of unreachable nodes before each sim
start and after each sim stop (a stale twin of a node name makes the master shut the new node down)."""
from types import MethodType, SimpleNamespace

from mobipick_gui.main_window import MainWindow


def _harness(**extra):
    h = SimpleNamespace(_roscore_container_name='mobipick-roscore', _docker_stop_timeout=10, **extra)
    for name in ('_safe_docker_cmd', '_interrupt_container_cmds', '_container_commands_for_ids',
                 '_wait_for_container_exit_cmd', '_docker_stop_args', '_local_master_cleanup_cmd',
                 '_cleanup_local_master'):
        setattr(h, name, MethodType(getattr(MainWindow, name), h))
    h._LOCAL_MASTER_CLEANUP_PY = MainWindow._LOCAL_MASTER_CLEANUP_PY
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
