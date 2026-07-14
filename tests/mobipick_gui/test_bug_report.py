import os

os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')

from PyQt5.QtWidgets import QApplication

from mobipick_gui.bug_report import (
    BUG_REPORT_SECTIONS,
    BugReportDialog,
    anonymize_bug_report,
    filter_mobipick_docker_images,
    format_bug_report,
    workspace_graph_ascii,
)


def test_bug_report_defaults_include_version_workspace_and_image_match():
    defaults = {
        section.key
        for section in BUG_REPORT_SECTIONS
        if section.default_checked
    }

    assert defaults == {
        'gui_version',
        'selected_workspace',
        'selected_image_match',
    }


def test_workspace_graph_ascii_renders_parent_to_child_edges():
    graph = workspace_graph_ascii(
        [
            {'name': 'base_ws', 'extends': []},
            {'name': 'overlay_ws', 'extends': ['base_ws']},
        ]
    )

    assert 'base_ws -> overlay_ws' in graph
    assert 'child extends parent' in graph


def test_filter_mobipick_docker_images_keeps_matching_lines():
    output = filter_mobipick_docker_images(
        'ubuntu:22.04 abc\n'
        'ozkrelo/x_mobipick_labs:noetic-v1.2 def\n'
        'local/mobipick-dev:latest ghi\n'
    )

    assert 'ubuntu:22.04' not in output
    assert 'ozkrelo/x_mobipick_labs:noetic-v1.2' in output
    assert 'local/mobipick-dev:latest' in output


def test_format_bug_report_uses_selected_sections():
    report = format_bug_report(
        {
            'generated_at': '2026-06-18T10:00:00+02:00',
            'gui_version': '1.2.3',
            'selected_workspace': {
                'name': 'demo_ws',
                'path': '/tmp/demo_ws',
                'extends': ['base_ws'],
                'image': 'local/mobipick:dev',
                'runtime_built': False,
                'active': True,
            },
            'selected_image': 'local/mobipick:dev',
            'active_workspace_name': 'demo_ws',
            'workspace_match': 'workspace match',
            'host_workspace_mount': 'enabled',
            'log_tab_text': 'important log line',
            'setup_diagnostics': 'Docker Compose plugin missing',
        },
        {'gui_version', 'selected_workspace', 'setup_diagnostics', 'user_notes'},
        {},
        'clicked run and saw an error',
    )

    assert 'Version' in report
    assert '1.2.3' in report
    assert 'demo_ws' in report
    assert 'Docker Compose plugin missing' in report
    assert 'clicked run and saw an error' in report
    assert 'important log line' not in report


def test_anonymize_bug_report_removes_local_identity_and_paths():
    report = anonymize_bug_report(
        'user=alice host=lab-pc\n'
        'workspace: /home/alice/MyProject/ros_ws\n'
        'quoted: "/home/alice/My Project/secret.txt"\n'
        r'windows: C:\\Users\\alice\\project\\config.yaml' '\n'
        'uri: file:///home/alice/Downloads/report.txt\n',
        user_names={'alice'},
        host_names={'lab-pc'},
    )

    assert 'alice' not in report
    assert 'lab-pc' not in report
    assert 'MyProject' not in report
    assert 'secret.txt' not in report
    assert 'config.yaml' not in report
    assert '<user name>' in report
    assert '<host name>' in report
    assert report.count('<local path>') == 4


def test_anonymize_bug_report_masks_private_values_with_length_clues():
    report = anonymize_bug_report(
        'UID=1001 GID: 42 password=hunter2 token: abc-123\n'
        'ROS_IP=192.168.12.7 MAC=aa:bb:cc:dd:ee:ff',
        user_names=set(),
        host_names=set(),
    )

    assert '1001' not in report
    assert 'hunter2' not in report
    assert '192.168.12.7' not in report
    assert 'UID=****' in report
    assert 'GID: **' in report
    assert 'password=*******' in report
    assert 'token: ***-***' in report
    assert 'ROS_IP=***.***.**.*' in report
    assert 'MAC=**:**:**:**:**:**' in report


def test_format_bug_report_anonymizes_all_selected_free_text():
    report = format_bug_report(
        {
            'generated_at': '2026-06-18T10:00:00+02:00',
            'gui_version': '1.2.3',
            'setup_diagnostics': 'failed in /home/private-user/ros_ws',
            'log_tab_text': 'prompt private-user@private-pc:~/ros_ws$',
        },
        {'setup_diagnostics', 'log_tab', 'user_notes'},
        {},
        'See /tmp/private-user/report.txt\nhostname: private-pc',
    )

    assert 'private-user' not in report
    assert 'private-pc' not in report
    assert 'report.txt' not in report
    assert report.count('<local path>') >= 3


def test_bug_report_dialog_uses_default_checked_sections(monkeypatch):
    monkeypatch.setattr(BugReportDialog, '_start_collectors', lambda self: None)
    app = QApplication.instance() or QApplication([])

    dialog = BugReportDialog(
        lambda: {
            'generated_at': '2026-06-18T10:00:00+02:00',
            'gui_version': '1.2.3',
            'selected_image': 'local/mobipick:dev',
            'workspace_match': 'workspace match',
            'host_workspace_mount': 'enabled',
        }
    )

    preview = dialog.preview_edit.toPlainText()
    assert '## GUI Version' in preview
    assert '1.2.3' in preview
    assert '## Selected Docker Image / Workspace Match' in preview
    assert '## Ubuntu Version' not in preview

    dialog.deleteLater()
    app.processEvents()


def test_bug_report_dialog_can_preselect_setup_diagnostics(monkeypatch):
    monkeypatch.setattr(BugReportDialog, '_start_collectors', lambda self: None)
    app = QApplication.instance() or QApplication([])

    dialog = BugReportDialog(
        lambda: {
            'gui_version': '1.2.3',
            'setup_diagnostics': 'docker ps failed: permission denied',
        },
        initial_notes='Setup failed after install.',
        initial_checked_keys={'setup_diagnostics'},
    )

    preview = dialog.preview_edit.toPlainText()
    assert '## Setup Checks' in preview
    assert 'docker ps failed: permission denied' in preview
    assert 'Setup failed after install.' in preview

    dialog.deleteLater()
    app.processEvents()
