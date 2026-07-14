import os
import subprocess
from urllib.parse import quote

os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')

from PyQt5.QtCore import QUrl
from PyQt5.QtWidgets import QApplication

from mobipick_gui import bug_report, external_links
from mobipick_gui.bug_report import (
    BUG_REPORT_EMAIL,
    BUG_REPORT_GITHUB_ISSUE_URL,
    GITHUB_TRUNCATION_NOTICE,
    MAX_GITHUB_ISSUE_URL_LENGTH,
    BugReportDialog,
    build_github_issue_url,
)


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


def test_bug_report_github_issue_uses_quiet_external_link(monkeypatch):
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
    dialog = BugReportDialog(
        lambda: {
            'gui_version': '1.2.3',
            'setup_diagnostics': (
                'docker compose failed for local-user@local-pc: '
                '/home/local-user/ros_ws'
            ),
        },
        initial_checked_keys={'setup_diagnostics'},
    )

    dialog.open_github_issue()

    assert opened['url'].startswith(BUG_REPORT_GITHUB_ISSUE_URL)
    assert 'Mobipick%20Labs%20Docker%20GUI%20bug%20report' in opened['url']
    assert 'docker%20compose%20failed' in opened['url']
    assert 'local-user' not in opened['url']
    assert 'local-pc' not in opened['url']
    assert 'ros_ws' not in opened['url']

    dialog.deleteLater()
    app.processEvents()


def test_github_issue_url_keeps_encoded_url_below_compatibility_limit():
    report = 'diagnostic line with spaces and ünicode\n' * 300

    url, remainder = build_github_issue_url('Bug report', report)
    encoded_url = bytes(url.toEncoded()).decode('utf-8')

    assert len(encoded_url) <= MAX_GITHUB_ISSUE_URL_LENGTH
    assert remainder
    assert quote(GITHUB_TRUNCATION_NOTICE, safe='') in encoded_url
    included_length = len(report) - len(remainder)
    assert report == report[:included_length] + remainder


def test_long_github_report_offers_copy_of_remaining_text(monkeypatch):
    opened = {}
    messages = []
    monkeypatch.setattr(BugReportDialog, '_start_collectors', lambda self: None)
    monkeypatch.setattr(
        bug_report,
        'open_external_url',
        lambda url: opened.setdefault('url', bytes(url.toEncoded())) or True,
    )
    monkeypatch.setattr(
        bug_report.QMessageBox,
        'information',
        lambda _parent, title, message: messages.append((title, message)),
    )

    app = QApplication.instance() or QApplication([])
    dialog = BugReportDialog(
        lambda: {'setup_diagnostics': 'failure detail\n' * 500},
        initial_checked_keys={'setup_diagnostics'},
    )

    dialog.open_github_issue()

    assert len(opened['url']) <= MAX_GITHUB_ISSUE_URL_LENGTH
    assert dialog._copy_button.text() == 'Copy Remaining'
    assert messages and 'too long' in messages[0][1]
    remainder = dialog._github_report_remainder

    dialog.copy_to_clipboard()

    assert QApplication.clipboard().text() == remainder
    assert 'Paste it at the end' in dialog.status_label.text()

    dialog.deleteLater()
    app.processEvents()
