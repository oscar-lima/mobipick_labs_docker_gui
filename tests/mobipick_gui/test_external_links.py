import os
import subprocess

os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')

from PyQt5.QtCore import QUrl
from PyQt5.QtWidgets import QApplication

from mobipick_gui import bug_report, external_links
from mobipick_gui.bug_report import BUG_REPORT_EMAIL, BugReportDialog


def test_open_external_url_uses_xdg_open_with_detached_streams(monkeypatch):
    popen_call = {}

    monkeypatch.setattr(external_links.sys, 'platform', 'linux')
    monkeypatch.setattr(
        external_links.shutil,
        'which',
        lambda name: '/usr/bin/xdg-open' if name == 'xdg-open' else None,
    )

    def fake_popen(command, **kwargs):
        popen_call['command'] = command
        popen_call['kwargs'] = kwargs
        return object()

    monkeypatch.setattr(external_links.subprocess, 'Popen', fake_popen)

    assert external_links.open_external_url(
        QUrl('mailto:user@example.test?subject=hello%20world')
    )

    assert popen_call['command'] == [
        '/usr/bin/xdg-open',
        'mailto:user@example.test?subject=hello%20world',
    ]
    assert popen_call['kwargs']['stdin'] == subprocess.DEVNULL
    assert popen_call['kwargs']['stdout'] == subprocess.DEVNULL
    assert popen_call['kwargs']['stderr'] == subprocess.DEVNULL
    assert popen_call['kwargs']['start_new_session'] is True


def test_open_external_url_falls_back_to_qt_when_launcher_missing(monkeypatch):
    opened = {}

    monkeypatch.setattr(external_links.sys, 'platform', 'linux')
    monkeypatch.setattr(external_links.shutil, 'which', lambda _name: None)
    monkeypatch.setattr(
        external_links.QDesktopServices,
        'openUrl',
        lambda url: opened.setdefault('url', url.toString()) or True,
    )

    assert external_links.open_external_url(QUrl('https://example.test/path'))

    assert opened['url'] == 'https://example.test/path'


def test_bug_report_email_uses_quiet_external_link(monkeypatch):
    opened = {}
    monkeypatch.setattr(BugReportDialog, '_start_collectors', lambda self: None)
    monkeypatch.setattr(
        bug_report,
        'open_external_url',
        lambda url: opened.setdefault(
            'url',
            bytes(url.toEncoded()).decode('utf-8'),
        )
        or True,
    )

    app = QApplication.instance() or QApplication([])
    dialog = BugReportDialog(lambda: {'gui_version': '1.2.3'})

    dialog.open_email()

    assert opened['url'].startswith(f'mailto:{BUG_REPORT_EMAIL}')
    assert 'Mobipick%20Labs%20Docker%20GUI%20bug%20report' in opened['url']

    dialog.deleteLater()
    app.processEvents()
