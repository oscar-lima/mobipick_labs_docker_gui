"""First-run setup wizard and Docker image builder settings."""
from __future__ import annotations

import re
from dataclasses import dataclass, field
from typing import Callable, Iterable

from PyQt5.QtWidgets import (
    QCheckBox,
    QComboBox,
    QFormLayout,
    QLabel,
    QLineEdit,
    QPushButton,
    QTextEdit,
    QVBoxLayout,
    QWizard,
    QWizardPage,
)

from .log_widget import LogTextEdit


@dataclass
class SetupWizardSelection:
    """Configuration selected in the setup wizard."""

    pull_public_images: bool
    public_images: list[str]
    default_image: str
    build_custom_image: bool
    host_user: str
    host_uid: str
    host_gid: str
    base_image: str
    target_image: str
    compatible_workspace: str
    remember_completion: bool
    image_blacklist: list[str] = field(default_factory=list)
    install_source_workspace: bool = False
    source_master_folder: str = ''
    source_workspace_name: str = ''
    source_repository: str = ''
    source_branch: str = ''
    source_image: str = ''
    activate_source_workspace: bool = False
    public_image_pull_mode: str = 'gui'


class ImageSetupWizard(QWizard):
    """Collect initial Docker image setup choices."""

    def __init__(
        self,
        *,
        public_images: Iterable[str],
        default_image: str,
        host_user: str,
        host_uid: str,
        host_gid: str,
        base_image: str,
        target_image: str,
        workspace_names: Iterable[str],
        active_workspace: str = '',
        build_custom_default: bool = False,
        configuration_paths: Iterable[tuple[str, str]] = (),
        source_master_folder: str = '',
        source_workspace_name: str = '',
        source_repository: str = '',
        source_branch: str = '',
        source_image: str = '',
        install_source_default: bool = False,
        image_blacklist: Iterable[str] = (),
        parent=None,
    ):
        super().__init__(parent)
        self.setWindowTitle('Mobipick Setup Wizard')
        self.setWizardStyle(QWizard.ModernStyle)
        self.resize(820, 640)
        self._setup_start_handler: (
            Callable[[SetupWizardSelection], bool] | None
        ) = None
        self._setup_started = False
        self._setup_complete = False

        self.pull_public_images = QCheckBox('Pull public Mobipick images')
        self.pull_public_images.setChecked(True)
        self.build_custom_image = QCheckBox('Build a host-user development image')
        self.build_custom_image.setChecked(build_custom_default)
        self.install_source_workspace = QCheckBox(
            'Clone and build mobipick_labs from source on this PC'
        )
        self.install_source_workspace.setChecked(install_source_default)
        self.remember_completion = QCheckBox('Do not show this wizard on startup again')
        self.remember_completion.setChecked(True)

        intro = QWizardPage()
        intro.setTitle('Setup Guide')
        intro_layout = QVBoxLayout(intro)
        intro_label = QLabel(
            'Prepare Docker images, host workspace settings, and optional '
            'source builds. Every step can be skipped.'
        )
        intro_label.setWordWrap(True)
        intro_layout.addWidget(intro_label)
        docker_cp_label = QLabel(
            'No Docker cp paths are configured by default. If you need files '
            'copied into or out of containers, configure them later from '
            'Tools > Docker > Configure Docker cp Paths.'
        )
        docker_cp_label.setWordWrap(True)
        intro_layout.addWidget(docker_cp_label)
        paths_label = QLabel(
            'The wizard can update image defaults and profiles, setup '
            'defaults, the workspace registry, and generated build folders.'
        )
        paths_label.setWordWrap(True)
        intro_layout.addWidget(paths_label)
        paths = [
            f'{label}: {path}'
            for label, path in configuration_paths
        ]
        if paths:
            paths_edit = QTextEdit()
            paths_edit.setAcceptRichText(False)
            paths_edit.setReadOnly(True)
            paths_edit.setPlainText('\n'.join(paths))
            paths_edit.setMinimumHeight(120)
            intro_layout.addWidget(paths_edit)
        intro_layout.addWidget(self.pull_public_images)
        intro_layout.addWidget(self.build_custom_image)
        intro_layout.addWidget(self.install_source_workspace)
        intro_layout.addWidget(self.remember_completion)
        self._intro_page_id = self.addPage(intro)

        image_page = QWizardPage()
        image_page.setTitle('Public Images')
        image_layout = QFormLayout(image_page)
        image_hint = QLabel(
            'These image refs are pulled with docker pull and the default '
            'image is saved to the GUI settings.'
        )
        image_hint.setWordWrap(True)
        image_layout.addRow(image_hint)
        self.public_images_edit = QTextEdit()
        self.public_images_edit.setAcceptRichText(False)
        self.public_images_edit.setPlainText('\n'.join(public_images))
        self.public_images_edit.setMinimumHeight(110)
        image_layout.addRow('Images to pull:', self.public_images_edit)
        self.default_image_edit = QLineEdit(default_image)
        image_layout.addRow('Default image:', self.default_image_edit)
        self.public_image_pull_mode = QComboBox()
        self.public_image_pull_mode.addItem(
            'Pull on this PC and stream output',
            'gui',
        )
        self.public_image_pull_mode.addItem(
            'I will pull manually and confirm when done',
            'manual',
        )
        image_layout.addRow('Pull method:', self.public_image_pull_mode)
        self.image_blacklist_edit = QTextEdit()
        self.image_blacklist_edit.setAcceptRichText(False)
        self.image_blacklist_edit.setPlainText('\n'.join(image_blacklist))
        self.image_blacklist_edit.setMinimumHeight(80)
        image_layout.addRow('Image blacklist:', self.image_blacklist_edit)
        self._add_skip_button(image_layout, self.pull_public_images)
        self._image_page_id = self.addPage(image_page)

        dev_page = QWizardPage()
        dev_page.setTitle('Development Image')
        dev_layout = QFormLayout(dev_page)
        dev_hint = QLabel(
            'This optional image adds a container user matching the host so '
            'mounted workspace files remain editable on the host.'
        )
        dev_hint.setWordWrap(True)
        dev_layout.addRow(dev_hint)
        self.host_user_edit = QLineEdit(host_user)
        dev_layout.addRow('Host user:', self.host_user_edit)
        self.host_uid_edit = QLineEdit(host_uid)
        dev_layout.addRow('Host UID:', self.host_uid_edit)
        self.host_gid_edit = QLineEdit(host_gid)
        dev_layout.addRow('Host GID:', self.host_gid_edit)
        self.base_image_edit = QLineEdit(base_image)
        dev_layout.addRow('Base image:', self.base_image_edit)
        self.target_image_edit = QLineEdit(target_image)
        dev_layout.addRow('Target image:', self.target_image_edit)
        self.workspace_combo = QComboBox()
        self.workspace_combo.addItem('(no green workspace match)', '')
        for name in workspace_names:
            value = str(name).strip()
            if value:
                self.workspace_combo.addItem(value, value)
        preferred = active_workspace or ''
        index = self.workspace_combo.findData(preferred)
        if index >= 0:
            self.workspace_combo.setCurrentIndex(index)
        dev_layout.addRow('Workspace match:', self.workspace_combo)
        self._add_skip_button(dev_layout, self.build_custom_image)
        self._dev_page_id = self.addPage(dev_page)

        source_page = QWizardPage()
        source_page.setTitle('Source Workspace')
        source_layout = QFormLayout(source_page)
        source_hint = QLabel(
            'This creates <master folder>/<workspace>/src/mobipick_labs on '
            'the host, then runs install-deps.sh and build.sh in Docker with '
            'live output.'
        )
        source_hint.setWordWrap(True)
        source_layout.addRow(source_hint)
        self.source_master_folder_edit = QLineEdit(source_master_folder)
        source_layout.addRow('Master folder:', self.source_master_folder_edit)
        self.source_workspace_name_edit = QLineEdit(source_workspace_name)
        source_layout.addRow('Workspace name:', self.source_workspace_name_edit)
        self.source_repository_edit = QLineEdit(source_repository)
        source_layout.addRow('Repository:', self.source_repository_edit)
        self.source_branch_edit = QLineEdit(source_branch)
        source_layout.addRow('Branch/tag:', self.source_branch_edit)
        self.source_image_edit = QLineEdit(source_image)
        source_layout.addRow('Docker image:', self.source_image_edit)
        self.activate_source_workspace = QCheckBox(
            'Ask to switch to this workspace when the build finishes'
        )
        self.activate_source_workspace.setChecked(False)
        source_layout.addRow(self.activate_source_workspace)
        self._add_skip_button(source_layout, self.install_source_workspace)
        source_page.setFinalPage(True)
        self._source_page_id = self.addPage(source_page)

        progress_page = QWizardPage()
        progress_page.setTitle('Run Setup')
        progress_layout = QVBoxLayout(progress_page)
        self.progress_status_label = QLabel(
            'Setup will start when you press Start Setup.'
        )
        self.progress_status_label.setWordWrap(True)
        progress_layout.addWidget(self.progress_status_label)
        self.progress_log = LogTextEdit()
        self.progress_log.setMinimumHeight(360)
        progress_layout.addWidget(self.progress_log)
        self._progress_page_id = self.addPage(progress_page)

        summary_page = QWizardPage()
        summary_page.setTitle('Setup Summary')
        summary_layout = QVBoxLayout(summary_page)
        self.summary_label = QLabel('Setup is complete.')
        self.summary_label.setWordWrap(True)
        summary_layout.addWidget(self.summary_label)
        self.summary_edit = QTextEdit()
        self.summary_edit.setAcceptRichText(False)
        self.summary_edit.setReadOnly(True)
        self.summary_edit.setMinimumHeight(260)
        summary_layout.addWidget(self.summary_edit)
        summary_page.setFinalPage(True)
        self._summary_page_id = self.addPage(summary_page)

        self.currentIdChanged.connect(self._update_navigation_buttons)
        self._update_navigation_buttons(self.currentId())

    def set_setup_start_handler(
        self,
        handler: Callable[[SetupWizardSelection], bool],
    ) -> None:
        """Set the callback that starts setup from the final input page."""
        self._setup_start_handler = handler

    def accept(self) -> None:
        """Start setup from the input page; close only from the summary page."""
        if self.currentId() == self._source_page_id and not self._setup_started:
            if self._setup_start_handler is None:
                return
            if self._setup_start_handler(self.selection()):
                self._setup_started = True
            return
        super().accept()

    def begin_setup(self) -> None:
        """Show the progress page and reset prior streamed setup output."""
        self._setup_started = True
        self._setup_complete = False
        self.progress_log.clear()
        self.summary_edit.clear()
        self.progress_status_label.setText('Setup is running...')
        while self.currentId() != self._progress_page_id:
            current_id = self.currentId()
            if current_id < 0 or current_id >= self._progress_page_id:
                break
            self.next()
            if self.currentId() == current_id:
                break
        self._update_navigation_buttons(self.currentId())

    def append_progress_html(self, html_text: str) -> None:
        """Append a GUI-authored line to the setup progress log."""
        self.progress_log.enqueue(True, html_text + '<br>')

    def complete_setup(
        self,
        *,
        success: bool,
        summary_lines: Iterable[str],
    ) -> None:
        """Unlock the summary page after setup commands have finished."""
        self._setup_complete = True
        if success:
            self.progress_status_label.setText(
                'Setup finished successfully. Continue to review the summary.'
            )
            self.summary_label.setText('Setup finished successfully.')
        else:
            self.progress_status_label.setText(
                'Setup finished with errors. Continue to review the summary.'
            )
            self.summary_label.setText('Setup finished with errors.')
        self.summary_edit.setPlainText('\n'.join(summary_lines))
        self._update_navigation_buttons(self.currentId())

    def selection(self) -> SetupWizardSelection:
        """Return normalized choices from the wizard fields."""
        return SetupWizardSelection(
            pull_public_images=self.pull_public_images.isChecked(),
            public_images=self._parse_image_list(
                self.public_images_edit.toPlainText()
            ),
            default_image=self.default_image_edit.text().strip(),
            build_custom_image=self.build_custom_image.isChecked(),
            host_user=self.host_user_edit.text().strip(),
            host_uid=self.host_uid_edit.text().strip(),
            host_gid=self.host_gid_edit.text().strip(),
            base_image=self.base_image_edit.text().strip(),
            target_image=self.target_image_edit.text().strip(),
            compatible_workspace=str(self.workspace_combo.currentData() or ''),
            remember_completion=self.remember_completion.isChecked(),
            image_blacklist=self._parse_image_list(
                self.image_blacklist_edit.toPlainText()
            ),
            install_source_workspace=self.install_source_workspace.isChecked(),
            source_master_folder=self.source_master_folder_edit.text().strip(),
            source_workspace_name=self.source_workspace_name_edit.text().strip(),
            source_repository=self.source_repository_edit.text().strip(),
            source_branch=self.source_branch_edit.text().strip(),
            source_image=self.source_image_edit.text().strip(),
            activate_source_workspace=self.activate_source_workspace.isChecked(),
            public_image_pull_mode=str(
                self.public_image_pull_mode.currentData() or 'gui'
            ),
        )

    @staticmethod
    def _parse_image_list(text: str) -> list[str]:
        images: list[str] = []
        for item in re.split(r'[\n,]+', text or ''):
            value = item.strip()
            if value and value not in images:
                images.append(value)
        return images

    def _add_skip_button(self, layout: QFormLayout, checkbox: QCheckBox) -> None:
        button = QPushButton('Skip This Step')
        button.clicked.connect(
            lambda _checked=False: self._skip_step(checkbox)
        )
        layout.addRow('', button)

    def _skip_step(self, checkbox: QCheckBox) -> None:
        checkbox.setChecked(False)
        if self.currentId() == self._source_page_id and not self._setup_started:
            self.accept()
            return
        self.next()

    def _update_navigation_buttons(self, page_id: int) -> None:
        self.setButtonText(QWizard.FinishButton, 'Start Setup')
        if page_id == self._progress_page_id:
            self.button(QWizard.BackButton).setEnabled(False)
            self.button(QWizard.NextButton).setEnabled(self._setup_complete)
            return
        if page_id == self._summary_page_id:
            self.setButtonText(QWizard.FinishButton, 'Finish Setup')
            self.button(QWizard.BackButton).setEnabled(False)


__all__ = ['ImageSetupWizard', 'SetupWizardSelection']
