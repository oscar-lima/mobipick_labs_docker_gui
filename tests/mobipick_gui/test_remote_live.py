"""Live checks against a running GUI with the remote-control API enabled.

These tests are skipped unless ``MOBIPICK_GUI_REMOTE_LIVE=1`` is set, because
they need a running GUI (``--remote-control``) with roscore and the simulator
already up. Run them with::

    MOBIPICK_GUI_REMOTE_LIVE=1 MOBIPICK_GUI_REMOTE_URL=http://127.0.0.1:8765 \\
        pytest tests/mobipick_gui/test_remote_live.py -v

They open a fresh remote shell, verify that TCPROS subscriptions receive data
(``/clock`` from Gazebo), and close the shell again. Nothing is started or
stopped through the GUI.
"""
from __future__ import annotations

import os

import pytest

from mobipick_gui.remote_client import RemoteClient

pytestmark = pytest.mark.skipif(
    os.environ.get('MOBIPICK_GUI_REMOTE_LIVE', '').strip().lower() not in {'1', 'true', 'yes'},
    reason='set MOBIPICK_GUI_REMOTE_LIVE=1 with a running GUI, roscore, and sim',
)


@pytest.fixture
def live_client():
    client = RemoteClient(timeout=60.0)
    status = client.call('GET', '/status')
    assert status.get('ok'), status
    if not (status.get('roscore_running') and status.get('sim_running')):
        pytest.skip('roscore and sim must be running for live topic checks')
    return client


@pytest.fixture
def fresh_shell(live_client):
    opened = live_client.call('POST', '/shell', body={'name': 'pytest-live', 'stream': True}, timeout=240.0)
    assert opened.get('ok') and opened.get('ready'), opened
    session_id = opened['session']['id']
    try:
        yield live_client, session_id, opened
    finally:
        live_client.call('DELETE', f'/shell/{session_id}')


def _exec(client, session_id, command, timeout=60.0):
    result = client.call(
        'POST',
        f'/shell/{session_id}/exec',
        body={'command': command, 'stream': True, 'timeout': timeout},
        timeout=timeout + 15.0,
    )
    assert result.get('ok'), result
    assert not result.get('timed_out'), result
    return result['command']['exit_code'], result.get('output', [])


def test_fresh_shell_reports_workspace_and_master(fresh_shell):
    _client, _session_id, opened = fresh_shell
    startup = '\n'.join(opened['startup_output'])
    assert 'remote shell workspace:' in startup
    assert 'ROS_MASTER_URI=' in startup


def test_fresh_shell_receives_clock_over_tcpros(fresh_shell):
    client, session_id, _ = fresh_shell
    code, output = _exec(client, session_id, 'timeout 15 rostopic echo -n1 /clock')
    assert code == 0, output
    assert any(line.startswith('clock:') for line in output), output
    assert any('secs' in line for line in output), output


def test_fresh_shell_python_subscriber_gets_messages(fresh_shell):
    client, session_id, _ = fresh_shell
    script = (
        'import rospy; from rosgraph_msgs.msg import Clock; '
        'rospy.init_node("remote_live_probe", anonymous=True); '
        'm = rospy.wait_for_message("/clock", Clock, timeout=20); '
        'print("clock", m.clock.secs)'
    )
    code, output = _exec(client, session_id, f'timeout 40 python3 -c {script!r}')
    assert code == 0, output
    assert any(line.startswith('clock ') for line in output), output


def test_fresh_shell_sees_gazebo_publisher_and_joint_states(fresh_shell):
    client, session_id, _ = fresh_shell
    code, output = _exec(client, session_id, 'rostopic info /clock')
    assert code == 0 and any('/gazebo' in line for line in output), output
    code, output = _exec(client, session_id, 'timeout 20 rostopic echo -n1 /mobipick/joint_states/name')
    assert code == 0 and any('joint' in line for line in output), output
