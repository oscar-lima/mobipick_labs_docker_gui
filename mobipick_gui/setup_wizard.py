"""First-run setup wizard and Docker image builder settings."""
from __future__ import annotations

import re
import shlex
from dataclasses import dataclass, field
from typing import Callable, Iterable

from PyQt5.QtWidgets import (
    QApplication,
    QCheckBox,
    QComboBox,
    QDialog,
    QDialogButtonBox,
    QFormLayout,
    QHBoxLayout,
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


@dataclass
class HostDependency:
    """Host package surfaced by the setup wizard dependency page."""

    key: str
    label: str
    package: str
    installed: bool
    reason: str
    required: bool = False
    check_commands: list[str] = field(default_factory=list)


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
        host_dependencies: Iterable[HostDependency] = (),
        host_dependency_refresher: Callable[[], Iterable[HostDependency]] | None = None,
        host_dependency_report_handler: Callable[[str], None] | None = None,
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
        self._host_dependencies: list[HostDependency] = list(host_dependencies)
        self._host_dependency_refresher = host_dependency_refresher
        self._host_dependency_report_handler = host_dependency_report_handler
        self._dependency_checkboxes: dict[str, QCheckBox] = {}
        self._dependency_status_labels: dict[str, QLabel] = {}
        self._last_dependency_report = ''
        self._dependency_details_dialog: QDialog | None = None

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

        dependency_page = QWizardPage()
        dependency_page.setTitle('Host Dependencies')
        dependency_layout = QVBoxLayout(dependency_page)
        dependency_hint = QLabel(
            'Install missing Ubuntu host packages before using Docker, window '
            'layout capture, workspace graphs, or screen recording. The GUI '
            'only copies a terminal command; you choose what to run.'
        )
        dependency_hint.setWordWrap(True)
        dependency_layout.addWidget(dependency_hint)
        for dep in self._host_dependencies:
            checkbox = QCheckBox(dep.label)
            checkbox.setChecked(not dep.installed)
            checkbox.toggled.connect(self._update_dependency_command)
            checkbox.setToolTip(self._dependency_tooltip(dep))
            dependency_layout.addWidget(checkbox)
            reason = QLabel(self._dependency_status_text(dep))
            reason.setWordWrap(True)
            reason.setContentsMargins(22, 0, 0, 6)
            dependency_layout.addWidget(reason)
            self._dependency_checkboxes[dep.key] = checkbox
            self._dependency_status_labels[dep.key] = reason
        if not self._host_dependencies:
            none_label = QLabel('No host dependency checks are configured.')
            none_label.setWordWrap(True)
            dependency_layout.addWidget(none_label)
        self.dependency_command_edit = QTextEdit()
        self.dependency_command_edit.setAcceptRichText(False)
        self.dependency_command_edit.setReadOnly(True)
        self.dependency_command_edit.setMinimumHeight(120)
        dependency_layout.addWidget(self.dependency_command_edit)
        self.dependency_result_label = QLabel('')
        self.dependency_result_label.setWordWrap(True)
        dependency_layout.addWidget(self.dependency_result_label)
        dependency_buttons = QHBoxLayout()
        self.copy_dependency_command_button = QPushButton('Copy Command')
        self.copy_dependency_command_button.clicked.connect(
            self._copy_dependency_command
        )
        dependency_buttons.addWidget(self.copy_dependency_command_button)
        self.dependency_done_button = QPushButton('Run Checks')
        self.dependency_done_button.clicked.connect(
            self._mark_selected_dependencies_done
        )
        dependency_buttons.addWidget(self.dependency_done_button)
        self.dependency_report_button = QPushButton('Open Bug Report')
        self.dependency_report_button.clicked.connect(
            self._open_dependency_report
        )
        self.dependency_report_button.setEnabled(False)
        dependency_buttons.addWidget(self.dependency_report_button)
        dependency_buttons.addStretch(1)
        dependency_layout.addLayout(dependency_buttons)
        self._dependency_page_id = self.addPage(dependency_page)
        self._update_dependency_command()

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

    def _selected_host_dependencies(self) -> list[HostDependency]:
        selected: list[HostDependency] = []
        for dep in self._host_dependencies:
            checkbox = self._dependency_checkboxes.get(dep.key)
            if checkbox is not None and checkbox.isChecked():
                selected.append(dep)
        return selected

    def _dependency_install_command(self) -> str:
        selected = self._selected_host_dependencies()
        selected_keys = {dep.key for dep in selected}
        needs_docker = bool({'docker', 'docker_compose'} & selected_keys)
        packages: list[str] = []
        for dep in selected:
            if needs_docker and dep.key in {'docker', 'docker_compose'}:
                continue
            if dep.package and dep.package not in packages:
                packages.append(dep.package)
        if not packages and not needs_docker:
            return '# Select one or more host dependencies to build a command.'

        lines: list[str] = []
        if needs_docker:
            lines.extend(self._docker_official_install_command(packages))
        elif packages:
            lines.extend(self._host_packages_install_command(packages))
        return '\n'.join(lines)

    @staticmethod
    def _script_array(name: str, values: list[str]) -> str:
        quoted = ' '.join(shlex.quote(value) for value in values)
        return f'{name}=({quoted})'

    @staticmethod
    def _script_confirmation_helpers() -> list[str]:
        return [
            'confirm_step() {',
            '  local title="$1"',
            '  local why="$2"',
            '  shift 2',
            '  echo',
            '  echo "==> ${title}"',
            '  echo "Why: ${why}"',
            '  echo "Commands to run:"',
            '  printf "  %s\\n" "$@"',
            '  local answer',
            '  read -r -p "Run this step? [y/N] " answer',
            '  case "${answer}" in',
            '    y|Y|yes|YES|Yes) ;;',
            '    *) echo "Stopped before: ${title}"; exit 130 ;;',
            '  esac',
            '}',
        ]

    @classmethod
    def _host_packages_install_command(cls, packages: list[str]) -> list[str]:
        return [
            "bash <<'MOBIPICK_HOST_PACKAGE_INSTALL'",
            'set -Eeuo pipefail',
            '',
            *cls._script_confirmation_helpers(),
            '',
            cls._script_array('host_packages', packages),
            '',
            'confirm_step \\',
            '  "Refresh Ubuntu package indexes" \\',
            '  "apt needs current indexes before installing selected host tools." \\',
            '  "sudo apt update"',
            'sudo apt update',
            '',
            'confirm_step \\',
            '  "Install selected host tools" \\',
            '  "These optional tools enable GUI features such as window layouts, graphs, or recording." \\',
            '  "sudo apt install -y ${host_packages[*]}"',
            'sudo apt install -y "${host_packages[@]}"',
            '',
            'echo "Selected host tool installation finished."',
            'MOBIPICK_HOST_PACKAGE_INSTALL',
        ]

    @classmethod
    def _docker_official_install_command(cls, support_packages: list[str]) -> list[str]:
        return [
            "bash <<'MOBIPICK_DOCKER_INSTALL'",
            'set -Eeuo pipefail',
            '',
            'fail() { echo "ERROR: $*" >&2; exit 1; }',
            'warn() { echo "WARNING: $*" >&2; }',
            *cls._script_confirmation_helpers(),
            '',
            'if [[ ! -r /etc/os-release ]]; then',
            '  fail "/etc/os-release is missing; cannot detect Ubuntu release."',
            'fi',
            '. /etc/os-release',
            'if [[ "${ID:-}" != "ubuntu" ]]; then',
            '  warn "This script is intended for Ubuntu; detected ${PRETTY_NAME:-unknown}."',
            'fi',
            'codename="${UBUNTU_CODENAME:-${VERSION_CODENAME:-}}"',
            'if [[ -z "${codename}" ]]; then',
            '  fail "Could not determine Ubuntu codename from /etc/os-release."',
            'fi',
            'arch="$(dpkg --print-architecture)"',
            'case "${arch}" in',
            '  amd64|arm64|armhf|s390x) ;;',
            '  *) warn "Architecture ${arch} may not be supported by Docker packages." ;;',
            'esac',
            '',
            'if command -v snap >/dev/null 2>&1 && snap list docker >/dev/null 2>&1; then',
            '  warn "Snap Docker is installed. If apt Docker fails, remove it with: sudo snap remove docker"',
            'fi',
            '',
            'docker_packages=(docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin)',
            cls._script_array('support_packages', support_packages),
            '',
            'confirm_step \\',
            '  "Install apt prerequisites" \\',
            '  "Docker\'s official apt repository needs ca-certificates, curl, and gnupg to download and install the repository signing key." \\',
            '  "sudo apt update" \\',
            '  "sudo apt install -y ca-certificates curl gnupg"',
            'sudo apt update',
            'sudo apt install -y ca-certificates curl gnupg',
            '',
            'tmp_key="$(mktemp)"',
            'trap \'rm -f "${tmp_key}"\' EXIT',
            'confirm_step \\',
            '  "Replace Docker apt repository key and source file" \\',
            '  "This removes stale Docker repository files, downloads Docker\'s current GPG key, and writes the apt source for this Ubuntu release." \\',
            '  "sudo rm -f /etc/apt/keyrings/docker.gpg /etc/apt/keyrings/docker.asc /etc/apt/sources.list.d/docker.list" \\',
            '  "curl -fsSL https://download.docker.com/linux/ubuntu/gpg -o ${tmp_key}" \\',
            '  "sudo gpg --dearmor --yes -o /etc/apt/keyrings/docker.gpg ${tmp_key}" \\',
            '  "write /etc/apt/sources.list.d/docker.list"',
            'sudo install -m 0755 -d /etc/apt/keyrings',
            'sudo rm -f /etc/apt/keyrings/docker.gpg /etc/apt/keyrings/docker.asc /etc/apt/sources.list.d/docker.list',
            'curl -fsSL https://download.docker.com/linux/ubuntu/gpg -o "${tmp_key}"',
            'sudo gpg --dearmor --yes -o /etc/apt/keyrings/docker.gpg "${tmp_key}"',
            'sudo chmod a+r /etc/apt/keyrings/docker.gpg',
            '',
            'repo_line="deb [arch=${arch} signed-by=/etc/apt/keyrings/docker.gpg] https://download.docker.com/linux/ubuntu ${codename} stable"',
            'echo "${repo_line}" | sudo tee /etc/apt/sources.list.d/docker.list > /dev/null',
            'echo "==> Docker repo file:"',
            'cat /etc/apt/sources.list.d/docker.list',
            '',
            'confirm_step \\',
            '  "Update apt indexes and verify Docker package candidates" \\',
            '  "This proves apt can see Docker\'s repository before any Docker packages are installed." \\',
            '  "sudo apt update" \\',
            '  "apt-cache policy ${docker_packages[*]}"',
            'sudo apt update',
            '',
            'echo "==> Checking Docker package candidates"',
            'apt-cache policy "${docker_packages[@]}"',
            'for package in "${docker_packages[@]}"; do',
            '  candidate="$(apt-cache policy "${package}" | awk \'/Candidate:/ {print $2; exit}\')"',
            '  if [[ -z "${candidate}" || "${candidate}" == "(none)" ]]; then',
            '    fail "${package} has no apt candidate. Check the Docker repository output above."',
            '  fi',
            'done',
            '',
            'confirm_step \\',
            '  "Install Docker Engine and Compose plugin" \\',
            '  "This installs Docker Engine, containerd, buildx, and the Docker Compose plugin from Docker\'s official repository." \\',
            '  "sudo apt install -y ${docker_packages[*]}"',
            'sudo apt install -y "${docker_packages[@]}"',
            '',
            'if (( ${#support_packages[@]} > 0 )); then',
            '  confirm_step \\',
            '    "Install selected optional host tools" \\',
            '    "These packages enable optional GUI features such as window layout replay, workspace graphs, or screen recording." \\',
            '    "sudo apt install -y ${support_packages[*]}"',
            '  sudo apt install -y "${support_packages[@]}"',
            'fi',
            '',
            'if [[ "$(ps -p 1 -o comm= 2>/dev/null || true)" == "systemd" ]]; then',
            '  confirm_step \\',
            '    "Enable and restart Docker services" \\',
            '    "Docker needs containerd and the docker daemon running before the GUI can launch containers." \\',
            '    "sudo systemctl daemon-reload" \\',
            '    "sudo systemctl enable containerd || true" \\',
            '    "sudo systemctl restart containerd" \\',
            '    "sudo systemctl enable docker || true" \\',
            '    "sudo systemctl restart docker"',
            '  sudo systemctl daemon-reload',
            '  sudo systemctl enable containerd || true',
            '  sudo systemctl restart containerd',
            '  sudo systemctl enable docker || true',
            '  sudo systemctl restart docker',
            'else',
            '  warn "systemd is not PID 1; skipping systemctl service management."',
            'fi',
            '',
            'confirm_step \\',
            '  "Configure docker group access" \\',
            '  "This lets the current user run Docker without sudo after group membership is refreshed; it also normalizes the Docker socket group if present." \\',
            '  "sudo groupadd -f docker" \\',
            '  "sudo usermod -aG docker $USER" \\',
            '  "sudo chgrp docker /var/run/docker.sock || true" \\',
            '  "sudo chmod 660 /var/run/docker.sock || true"',
            'sudo groupadd -f docker',
            'sudo usermod -aG docker "$USER"',
            'if [[ -S /var/run/docker.sock ]]; then',
            '  sudo chgrp docker /var/run/docker.sock || true',
            '  sudo chmod 660 /var/run/docker.sock || true',
            'fi',
            '',
            'confirm_step \\',
            '  "Test Docker with sudo" \\',
            '  "This checks whether the daemon and Compose plugin work independently of current-user group membership." \\',
            '  "sudo docker images" \\',
            '  "sudo docker compose version"',
            'sudo docker images',
            'sudo docker compose version',
            '',
            'confirm_step \\',
            '  "Test Docker as current user" \\',
            '  "This checks whether the current shell can access Docker without sudo; if it fails, a logout/login or newgrp docker is usually needed." \\',
            '  "docker images" \\',
            '  "docker compose version"',
            'if docker images >/dev/null 2>&1; then',
            '  docker images',
            'else',
            '  warn "Plain docker still cannot access the daemon in this shell."',
            '  warn "Log out and back in, or run: newgrp docker"',
            'fi',
            'docker compose version',
            '',
            'echo "Docker installation checks finished."',
            'MOBIPICK_DOCKER_INSTALL',
        ]

    @staticmethod
    def _dependency_status_text(dep: HostDependency) -> str:
        status = 'installed' if dep.installed else 'missing'
        return f'{status}: {dep.reason}'

    @staticmethod
    def _dependency_tooltip(dep: HostDependency) -> str:
        required = 'Required' if dep.required else 'Optional'
        if dep.package:
            return f'{required}. Apt package: {dep.package}'
        return required

    def _update_dependency_command(self) -> None:
        if not hasattr(self, 'dependency_command_edit'):
            return
        command = self._dependency_install_command()
        self.dependency_command_edit.setPlainText(command)
        self.copy_dependency_command_button.setEnabled(
            bool(self._selected_host_dependencies())
        )
        if hasattr(self, 'dependency_report_button'):
            self.dependency_report_button.setEnabled(
                bool(
                    self._last_dependency_report
                    and self._host_dependency_report_handler
                )
            )

    def _copy_dependency_command(self) -> None:
        QApplication.clipboard().setText(self._dependency_install_command())

    def _mark_selected_dependencies_done(self) -> None:
        if self._host_dependency_refresher is not None:
            refreshed = {
                dep.key: dep
                for dep in self._host_dependency_refresher()
            }
            for dep in self._host_dependencies:
                fresh = refreshed.get(dep.key)
                if fresh is None:
                    continue
                dep.label = fresh.label
                dep.package = fresh.package
                dep.installed = fresh.installed
                dep.reason = fresh.reason
                dep.required = fresh.required
                dep.check_commands = list(fresh.check_commands)
                checkbox = self._dependency_checkboxes.get(dep.key)
                label = self._dependency_status_labels.get(dep.key)
                if checkbox is not None:
                    checkbox.setText(dep.label)
                    checkbox.setToolTip(self._dependency_tooltip(dep))
                    checkbox.setChecked(not dep.installed)
                if label is not None:
                    label.setText(self._dependency_status_text(dep))
            self._update_dependency_command()
            self._finish_dependency_check()
            return

        for dep in self._selected_host_dependencies():
            checkbox = self._dependency_checkboxes.get(dep.key)
            label = self._dependency_status_labels.get(dep.key)
            dep.installed = True
            if checkbox is not None:
                checkbox.setChecked(False)
            if label is not None:
                label.setText(f'done: {dep.reason}')
        self._update_dependency_command()
        self._finish_dependency_check()

    def _finish_dependency_check(self) -> None:
        failures = [dep for dep in self._host_dependencies if not dep.installed]
        if not failures:
            self._last_dependency_report = ''
            self.dependency_result_label.setText(
                'All configured host dependency checks passed.'
            )
            self.dependency_report_button.setEnabled(False)
            self._show_dependency_check_details()
            return
        self._last_dependency_report = self._dependency_report_text(failures)
        self.dependency_result_label.setText(
            'Some host dependency checks still failed. Open the bug report to '
            'copy, email, save, or create a GitHub issue with the details.'
        )
        self.dependency_report_button.setEnabled(
            self._host_dependency_report_handler is not None
        )
        self._show_dependency_check_details()

    def _dependency_check_details_text(self) -> str:
        lines = [
            'Host dependency check results',
            '',
            'These are the checks the setup wizard just ran. The Evidence line '
            'uses the exact status text returned by the checker.',
        ]
        if not self._host_dependencies:
            lines.extend([
                '',
                'No host dependency checks are configured.',
            ])
            return '\n'.join(lines)
        for dep in self._host_dependencies:
            result = 'OK' if dep.installed else 'FAILED'
            required = 'required' if dep.required else 'optional'
            package = dep.package or 'no apt package configured'
            lines.extend([
                '',
                f'Check: {dep.label}',
                f'Result: {result}',
                'Why: This is a '
                f'{required} host dependency. Apt package: {package}.',
                f'Evidence: {dep.reason}',
            ])
        return '\n'.join(lines)

    def _dependency_check_commands_text(self) -> str:
        lines = [
            '# Exact Bash probe commands used by Run Checks',
        ]
        if not self._host_dependencies:
            lines.append('# No host dependency checks are configured.')
            return '\n'.join(lines)
        for dep in self._host_dependencies:
            result = 'OK' if dep.installed else 'FAILED'
            lines.extend([
                '',
                f'# {dep.label} ({result})',
            ])
            commands = dep.check_commands or [
                '# No explicit probe command was recorded for this check.'
            ]
            lines.extend(commands)
        return '\n'.join(lines)

    def _show_dependency_check_details(self) -> None:
        if self._dependency_details_dialog is not None:
            self._dependency_details_dialog.close()
        dialog = QDialog(self)
        dialog.setWindowTitle('Host Dependency Check Details')
        dialog.resize(760, 520)
        layout = QVBoxLayout(dialog)
        intro = QLabel(
            'Review exactly what was checked, why it matters, and the evidence '
            'behind each result.'
        )
        intro.setWordWrap(True)
        layout.addWidget(intro)
        details = QTextEdit()
        details.setAcceptRichText(False)
        details.setReadOnly(True)
        details.setMinimumWidth(680)
        details.setMinimumHeight(220)
        details.setPlainText(self._dependency_check_details_text())
        layout.addWidget(details)
        command_label = QLabel('Exact Bash probe commands')
        command_label.setStyleSheet(
            'QLabel {'
            'background: #111;'
            'color: #f3f3f3;'
            'font-family: monospace;'
            'font-weight: bold;'
            'padding: 8px;'
            '}'
        )
        layout.addWidget(command_label)
        command_edit = QTextEdit()
        command_edit.setAcceptRichText(False)
        command_edit.setReadOnly(True)
        command_edit.setMinimumHeight(180)
        command_edit.setStyleSheet(
            'QTextEdit {'
            'background: #111;'
            'color: #f3f3f3;'
            'font-family: monospace;'
            'selection-background-color: #385a7c;'
            '}'
        )
        command_edit.setPlainText(self._dependency_check_commands_text())
        layout.addWidget(command_edit)
        buttons = QDialogButtonBox(QDialogButtonBox.Close)
        buttons.rejected.connect(dialog.close)
        layout.addWidget(buttons)
        dialog.finished.connect(self._clear_dependency_details_dialog)
        self._dependency_details_dialog = dialog
        if QApplication.platformName() != 'offscreen':
            dialog.show()

    def _clear_dependency_details_dialog(self, _result: int) -> None:
        self._dependency_details_dialog = None

    def _dependency_report_text(self, failures: Iterable[HostDependency]) -> str:
        lines = [
            'Host dependency checks failed after running the setup command.',
            '',
            'Failed checks:',
        ]
        for dep in failures:
            required = 'required' if dep.required else 'optional'
            lines.append(f'- {dep.label} ({required}): {dep.reason}')
        lines.extend([
            '',
            'Generated install command:',
            self._dependency_install_command(),
            '',
            'All current checks:',
        ])
        for dep in self._host_dependencies:
            status = 'OK' if dep.installed else 'FAILED'
            lines.append(f'- {status}: {dep.label}: {dep.reason}')
        return '\n'.join(lines)

    def _open_dependency_report(self) -> None:
        if not self._last_dependency_report:
            self._finish_dependency_check()
        if self._last_dependency_report and self._host_dependency_report_handler:
            self._host_dependency_report_handler(self._last_dependency_report)

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


__all__ = ['HostDependency', 'ImageSetupWizard', 'SetupWizardSelection']
