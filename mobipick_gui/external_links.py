"""Helpers for opening external links without leaking child stderr."""
from __future__ import annotations

import shutil
import subprocess
import sys

from PyQt5.QtCore import QUrl
from PyQt5.QtGui import QDesktopServices


def open_external_url(url: QUrl | str) -> bool:
    """Open an external URL while keeping browser warnings out of the GUI tty."""
    qurl = url if isinstance(url, QUrl) else QUrl(str(url))
    encoded_url = _encoded_url(qurl)
    if not encoded_url:
        return False

    command = _launcher_command(encoded_url)
    if command and _start_detached(command):
        return True

    return QDesktopServices.openUrl(qurl)


def _encoded_url(url: QUrl) -> str:
    return bytes(url.toEncoded()).decode('utf-8')


def _launcher_command(encoded_url: str) -> list[str] | None:
    if sys.platform.startswith('linux'):
        opener = shutil.which('xdg-open')
    elif sys.platform == 'darwin':
        opener = shutil.which('open')
    else:
        opener = None
    if not opener:
        return None
    return [opener, encoded_url]


def _start_detached(command: list[str]) -> bool:
    try:
        subprocess.Popen(
            command,
            stdin=subprocess.DEVNULL,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            close_fds=True,
            start_new_session=True,
        )
    except OSError:
        return False
    return True
