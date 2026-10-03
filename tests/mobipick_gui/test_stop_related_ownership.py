import subprocess
from types import MethodType, SimpleNamespace

from mobipick_gui.main_window import MainWindow

# docker ps -a --format '{{.ID}}|{{.Names}}|{{.Image}}|{{.Labels}}|{{.Status}}' (#300)
PS_ROWS = [
    'a1|mobipick-run|ozkrelo/x_mobipick_labs:tag|'
    'com.docker.compose.oneoff=True,com.docker.compose.project=mobipick|Up 5 minutes',
    'a2|mpcmd-0d240701fb|ozkrelo/x_mobipick_labs:tag|mobipick.exec=abc,mobipick.tab=log|Up 1 minute',
    'a3|mobipick-roscore|ozkrelo/x_mobipick_labs:tag|com.docker.compose.project=mobipick|Exited (0) 1 hour ago',
    'b1|f59d_281_race_patched|ozkrelo/mobipick_labs:noetic-v2.0|'
    'org.opencontainers.image.ref.name=ubuntu,org.opencontainers.image.version=20.04|Up 2 minutes',
    'b2|mobipick-graspness|mobipick_graspness|org.opencontainers.image.ref.name=ubuntu|Up 3 hours',
    'b3|piper|piper-tts:local|com.docker.compose.project=piper|Up 1 hour',
    'b4|gifted_meninsky|ozkrelo/x_mobipick_labs:tag|org.opencontainers.image.ref.name=ubuntu|Up 13 minutes',
]


def _harness(owned_names=()):
    harness = SimpleNamespace(
        _images_cfg={'owned_container_names': list(owned_names)},
        _ros_shutdown_grace_s=0.0,
        _sp_run=lambda *args, **kwargs: subprocess.CompletedProcess(
            args, 0, stdout='\n'.join(PS_ROWS) + '\n', stderr=''),
        _safe_docker_cmd=lambda *args: ['docker', *args],
        _docker_stop_args=lambda cid: ('stop', cid),
        _wait_for_container_exit_cmd=lambda cid, grace: ['wait', cid],
        _append_gui_html=lambda *args, **kwargs: None,
        _console_log=lambda *args, **kwargs: None,
    )
    harness._gui_owns_container = MethodType(MainWindow._gui_owns_container, harness)
    harness._interrupt_container_cmds = lambda cid: [['docker', 'kill', '-s', 'INT', cid]]   # #273
    return harness


def _stopped_ids(commands):
    return sorted({cmd[-1] for cmd in commands if cmd[:2] == ['docker', 'stop']})


def test_stop_all_related_stops_only_containers_the_gui_started():
    commands = MainWindow._stop_all_related(_harness(), None, grace_s=0.0)
    assert _stopped_ids(commands) == ['a1', 'a2']


def test_a_foreign_container_of_the_mobipick_image_survives_the_stop():
    commands = MainWindow._stop_all_related(_harness(), None, grace_s=0.0)
    flat = [part for cmd in commands for part in cmd]
    for foreign in ('b1', 'b2', 'b3', 'b4'):
        assert foreign not in flat


def test_owned_container_names_adds_exact_names_only():
    commands = MainWindow._stop_all_related(_harness(['mobipick-graspness', 'mobipick']), None, grace_s=0.0)
    assert _stopped_ids(commands) == ['a1', 'a2', 'b2']


def test_ownership_needs_the_label_not_a_name_or_image_substring():
    harness = _harness()
    assert harness._gui_owns_container('anything', 'mobipick.exec=1,mobipick.tab=x')
    assert harness._gui_owns_container('anything', 'com.docker.compose.project=mobipick')
    assert not harness._gui_owns_container('anything', 'com.docker.compose.project=mobipick2')
    assert not harness._gui_owns_container('mobipick-run-by-hand', '')
    assert not harness._gui_owns_container('x', 'note=mobipick.exec')
