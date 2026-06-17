"""First-run setup wizard and Docker image builder settings."""
from __future__ import annotations

import re
from dataclasses import dataclass
from typing import Iterable

from PyQt5.QtWidgets import (
    QCheckBox,
    QComboBox,
    QFormLayout,
    QLabel,
    QLineEdit,
    QTextEdit,
    QVBoxLayout,
    QWizard,
    QWizardPage,
)


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
        parent=None,
    ):
        super().__init__(parent)
        self.setWindowTitle('Mobipick Setup Wizard')
        self.setWizardStyle(QWizard.ModernStyle)

        self.pull_public_images = QCheckBox('Pull public Mobipick images')
        self.pull_public_images.setChecked(True)
        self.build_custom_image = QCheckBox('Build a host-user development image')
        self.build_custom_image.setChecked(build_custom_default)
        self.remember_completion = QCheckBox('Do not show this wizard on startup again')
        self.remember_completion.setChecked(True)

        intro = QWizardPage()
        intro.setTitle('Setup')
        intro_layout = QVBoxLayout(intro)
        intro_label = QLabel(
            'Prepare Docker images for testing with public root images or '
            'for development with a host-user image.'
        )
        intro_label.setWordWrap(True)
        intro_layout.addWidget(intro_label)
        intro_layout.addWidget(self.pull_public_images)
        intro_layout.addWidget(self.build_custom_image)
        intro_layout.addWidget(self.remember_completion)
        self.addPage(intro)

        image_page = QWizardPage()
        image_page.setTitle('Public Images')
        image_layout = QFormLayout(image_page)
        self.public_images_edit = QTextEdit()
        self.public_images_edit.setAcceptRichText(False)
        self.public_images_edit.setPlainText('\n'.join(public_images))
        self.public_images_edit.setMinimumHeight(110)
        image_layout.addRow('Images to pull:', self.public_images_edit)
        self.default_image_edit = QLineEdit(default_image)
        image_layout.addRow('Default image:', self.default_image_edit)
        self.addPage(image_page)

        dev_page = QWizardPage()
        dev_page.setTitle('Development Image')
        dev_layout = QFormLayout(dev_page)
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
        self.addPage(dev_page)

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
        )

    @staticmethod
    def _parse_image_list(text: str) -> list[str]:
        images: list[str] = []
        for item in re.split(r'[\n,]+', text or ''):
            value = item.strip()
            if value and value not in images:
                images.append(value)
        return images


__all__ = ['ImageSetupWizard', 'SetupWizardSelection']
