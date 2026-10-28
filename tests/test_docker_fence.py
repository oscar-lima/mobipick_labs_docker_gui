"""The suite never reaches the machine's Docker daemon (see the fence in tests/conftest.py)."""
import os
import shutil
import subprocess


def _calls_log():
    return os.path.join(os.path.dirname(shutil.which('docker')), 'calls.log')


def test_docker_resolves_to_the_fake_that_only_logs():
    assert os.path.basename(os.path.dirname(shutil.which('docker'))).startswith('mobipick-gui-tests-docker-')
    assert os.environ['DOCKER_HOST'].endswith('no-daemon.sock')


def test_a_login_shell_still_reaches_the_fake():
    result = subprocess.run(['bash', '-lc', 'docker ps --fence-check'], capture_output=True, text=True)
    assert result.returncode == 1
    with open(_calls_log(), encoding='utf-8') as calls:
        assert 'ps --fence-check' in calls.read()
