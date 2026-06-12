"""Qt dialog for configuring host-side ROS 1 workspaces."""
from __future__ import annotations

import shutil
import subprocess
import tempfile
from pathlib import Path
from typing import Callable

import yaml
from PyQt5.QtCore import Qt, pyqtSignal
from PyQt5.QtGui import QPixmap
from PyQt5.QtWidgets import (
    QDialog,
    QDialogButtonBox,
    QFileDialog,
    QFormLayout,
    QGroupBox,
    QHBoxLayout,
    QInputDialog,
    QLabel,
    QLineEdit,
    QMessageBox,
    QPushButton,
    QScrollArea,
    QTreeWidget,
    QTreeWidgetItem,
    QVBoxLayout,
)

from .settings_transfer import export_settings, import_settings
from .workspaces import RosWorkspace, WorkspaceRegistry


class WorkspaceManagerDialog(QDialog):
    """Manage workspace locations, profiles, inheritance, and builds."""

    workspace_activated = pyqtSignal(str)
    build_requested = pyqtSignal(str)
    settings_imported = pyqtSignal()

    def __init__(
        self,
        registry: WorkspaceRegistry,
        parent=None,
        *,
        replace_allowed: Callable[[], bool] | None = None,
    ):
        super().__init__(parent)
        self.registry = registry
        self.replace_allowed = replace_allowed
        self.setWindowTitle('ROS 1 Workspace Manager')
        self.resize(1000, 650)

        root = QVBoxLayout(self)

        self.master_label = QLabel()
        self.master_label.setTextInteractionFlags(Qt.TextSelectableByMouse)
        root.addWidget(self.master_label)

        master_actions = QHBoxLayout()
        choose_master = QPushButton('Choose Master Folder')
        choose_master.clicked.connect(self._choose_master_folder)
        master_actions.addWidget(choose_master)
        create_master = QPushButton('Create Master Folder')
        create_master.clicked.connect(self._create_master_folder)
        master_actions.addWidget(create_master)
        discover = QPushButton('Discover Workspaces')
        discover.clicked.connect(self._discover_workspaces)
        master_actions.addWidget(discover)
        master_actions.addStretch(1)
        root.addLayout(master_actions)

        self.tree = QTreeWidget()
        self.tree.setHeaderLabels(
            ['Workspace', 'Path', 'Docker Image', 'Extends', 'Status']
        )
        self.tree.setRootIsDecorated(False)
        self.tree.setAlternatingRowColors(True)
        self.tree.itemSelectionChanged.connect(self._load_selected)
        root.addWidget(self.tree, 2)

        workspace_actions = QHBoxLayout()
        add_existing = QPushButton('Add Existing')
        add_existing.clicked.connect(self._add_existing)
        workspace_actions.addWidget(add_existing)
        create_workspace = QPushButton('Create Workspace')
        create_workspace.clicked.connect(self._create_workspace)
        workspace_actions.addWidget(create_workspace)
        remove_workspace = QPushButton('Remove')
        remove_workspace.clicked.connect(self._remove_workspace)
        workspace_actions.addWidget(remove_workspace)
        set_active = QPushButton('Set Active')
        set_active.clicked.connect(self._set_active)
        workspace_actions.addWidget(set_active)
        use_default = QPushButton('Use Image Default')
        use_default.clicked.connect(self._use_image_default)
        workspace_actions.addWidget(use_default)
        show_graph = QPushButton('Show Graph')
        show_graph.clicked.connect(self._show_graph)
        workspace_actions.addWidget(show_graph)
        export_button = QPushButton('Export Settings')
        export_button.clicked.connect(self._export_settings)
        workspace_actions.addWidget(export_button)
        import_button = QPushButton('Import Settings')
        import_button.clicked.connect(self._import_settings)
        workspace_actions.addWidget(import_button)
        build = QPushButton('Build Selected')
        build.clicked.connect(self._build_selected)
        workspace_actions.addWidget(build)
        workspace_actions.addStretch(1)
        root.addLayout(workspace_actions)

        editor = QGroupBox('Selected Workspace')
        form = QFormLayout(editor)
        self.path_edit = QLineEdit()
        form.addRow('Path:', self.path_edit)
        self.extends_edit = QLineEdit()
        self.extends_edit.setPlaceholderText('Comma-separated parent workspaces')
        form.addRow('Extends:', self.extends_edit)
        self.image_edit = QLineEdit()
        self.image_edit.setPlaceholderText(
            'Docker image:tag used when this workspace is activated'
        )
        form.addRow('Docker image:', self.image_edit)
        self.sim_command_edit = QLineEdit()
        self.sim_command_edit.setPlaceholderText(
            'Default: roslaunch tables_demo_bringup demo_sim.launch ...'
        )
        form.addRow('Simulation command:', self.sim_command_edit)

        self.button_config_edit = QLineEdit()
        button_row = QHBoxLayout()
        button_row.addWidget(self.button_config_edit)
        browse_buttons = QPushButton('Browse')
        browse_buttons.clicked.connect(
            lambda: self._browse_yaml(self.button_config_edit)
        )
        button_row.addWidget(browse_buttons)
        form.addRow('Button profile:', button_row)

        self.launch_config_edit = QLineEdit()
        launch_row = QHBoxLayout()
        launch_row.addWidget(self.launch_config_edit)
        browse_launch = QPushButton('Browse')
        browse_launch.clicked.connect(
            lambda: self._browse_yaml(self.launch_config_edit)
        )
        launch_row.addWidget(browse_launch)
        form.addRow('Auto-launch profile:', launch_row)

        save_profile = QPushButton('Save Workspace Settings')
        save_profile.clicked.connect(self._save_selected)
        form.addRow('', save_profile)
        root.addWidget(editor)

        buttons = QDialogButtonBox(QDialogButtonBox.Close)
        buttons.rejected.connect(self.reject)
        root.addWidget(buttons)
        self.refresh()

    def _selected_name(self) -> str:
        items = self.tree.selectedItems()
        return items[0].data(0, Qt.UserRole) if items else ''

    def _selected_workspace(self) -> RosWorkspace | None:
        return self.registry.get(self._selected_name())

    def refresh(self, select_name: str = '') -> None:
        self.master_label.setText(
            'Master folder: '
            + (self.registry.master_folder or '(none; standalone workspaces are allowed)')
        )
        current = select_name or self._selected_name()
        self.tree.clear()
        selected_item = None
        for workspace in self.registry.workspaces:
            status = 'Active' if workspace.name == self.registry.active else ''
            if not workspace.directory.is_dir():
                status = (status + ', ' if status else '') + 'Missing'
            elif not self.registry.is_runtime_built(workspace):
                status = (status + ', ' if status else '') + 'Not built'
            item = QTreeWidgetItem(
                [
                    workspace.name,
                    str(workspace.directory),
                    workspace.image or '(GUI default)',
                    ', '.join(workspace.extends),
                    status,
                ]
            )
            item.setData(0, Qt.UserRole, workspace.name)
            self.tree.addTopLevelItem(item)
            if workspace.name == current:
                selected_item = item
        for column in range(5):
            self.tree.resizeColumnToContents(column)
        if selected_item:
            self.tree.setCurrentItem(selected_item)
        elif self.tree.topLevelItemCount():
            self.tree.setCurrentItem(self.tree.topLevelItem(0))
        else:
            self._clear_editor()

    def _clear_editor(self) -> None:
        for editor in (
            self.path_edit,
            self.extends_edit,
            self.image_edit,
            self.sim_command_edit,
            self.button_config_edit,
            self.launch_config_edit,
        ):
            editor.clear()

    def _load_selected(self) -> None:
        workspace = self._selected_workspace()
        if not workspace:
            self._clear_editor()
            return
        self.path_edit.setText(str(workspace.directory))
        self.extends_edit.setText(', '.join(workspace.extends))
        self.image_edit.setText(workspace.image)
        self.sim_command_edit.setText(workspace.sim_command)
        self.button_config_edit.setText(workspace.button_config)
        self.launch_config_edit.setText(workspace.launch_config)

    def _choose_master_folder(self) -> None:
        start = self.registry.master_folder or str(Path.home())
        selected = QFileDialog.getExistingDirectory(
            self,
            'Choose ROS workspace master folder',
            start,
        )
        if not selected:
            return
        self.registry.master_folder = str(Path(selected).resolve())
        self.registry.save()
        self.refresh()

    def _create_master_folder(self) -> None:
        parent = QFileDialog.getExistingDirectory(
            self,
            'Choose parent folder',
            str(Path.home()),
        )
        if not parent:
            return
        name, accepted = QInputDialog.getText(
            self,
            'Create Master Folder',
            'Folder name:',
            text='ros1_ws',
        )
        if not accepted or not name.strip():
            return
        if '/' in name or name in {'.', '..'}:
            QMessageBox.warning(self, 'Master Folder', 'Enter a single folder name.')
            return
        path = Path(parent) / name.strip()
        try:
            path.mkdir(parents=True, exist_ok=True)
            self.registry.master_folder = str(path.resolve())
            self.registry.save()
        except OSError as exc:
            QMessageBox.critical(self, 'Master Folder', str(exc))
            return
        self.refresh()

    def _discover_workspaces(self) -> None:
        if not self.registry.master_folder:
            self._choose_master_folder()
            if not self.registry.master_folder:
                return
        try:
            discovered = self.registry.discover(
                Path(self.registry.master_folder)
            )
            self.registry.save()
        except (OSError, ValueError) as exc:
            QMessageBox.critical(self, 'Discover Workspaces', str(exc))
            return
        self.refresh()
        QMessageBox.information(
            self,
            'Discover Workspaces',
            f'Found {len(discovered)} workspace(s).',
        )

    def _add_existing(self) -> None:
        start = self.registry.master_folder or str(Path.home())
        selected = QFileDialog.getExistingDirectory(
            self,
            'Choose existing catkin workspace',
            start,
        )
        if not selected:
            return
        directory = Path(selected).resolve()
        if not (directory / 'src').is_dir():
            QMessageBox.warning(
                self,
                'Add Workspace',
                'The selected folder must contain a src directory.',
            )
            return
        try:
            workspace = self.registry.with_inferred_profile(
                RosWorkspace(name=directory.name, path=str(directory))
            )
            self.registry.upsert(workspace)
            self.registry.save()
        except (OSError, ValueError) as exc:
            QMessageBox.critical(self, 'Add Workspace', str(exc))
            return
        self.refresh(workspace.name)

    def _create_workspace(self) -> None:
        name, accepted = QInputDialog.getText(
            self,
            'Create Workspace',
            'Workspace name:',
        )
        if not accepted or not name.strip():
            return
        parent: Path | None = None
        if not self.registry.master_folder:
            selected = QFileDialog.getExistingDirectory(
                self,
                'Choose workspace parent folder',
                str(Path.home()),
            )
            if not selected:
                return
            parent = Path(selected)
        try:
            workspace = self.registry.create(name.strip(), folder=parent)
            self.registry.save()
        except (OSError, ValueError) as exc:
            QMessageBox.critical(self, 'Create Workspace', str(exc))
            return
        self.refresh(workspace.name)

    def _remove_workspace(self) -> None:
        workspace = self._selected_workspace()
        if not workspace:
            return
        answer = QMessageBox.question(
            self,
            'Remove Workspace',
            f'Remove {workspace.name} from the GUI registry?\n'
            'Files on disk will not be deleted.',
        )
        if answer != QMessageBox.Yes:
            return
        try:
            self.registry.remove(workspace.name)
            self.registry.save()
        except ValueError as exc:
            QMessageBox.warning(self, 'Remove Workspace', str(exc))
            return
        self.refresh()

    def _set_active(self) -> None:
        workspace = self._selected_workspace()
        if not workspace:
            return
        self.workspace_activated.emit(workspace.name)

    def _use_image_default(self) -> None:
        self.workspace_activated.emit('')

    def _save_selected(self) -> None:
        workspace = self._selected_workspace()
        if not workspace:
            return
        updated = RosWorkspace(
            name=workspace.name,
            path=self.path_edit.text(),
            image=self.image_edit.text(),
            extends=[
                part.strip()
                for part in self.extends_edit.text().split(',')
                if part.strip()
            ],
            sim_command=self.sim_command_edit.text(),
            button_config=self.button_config_edit.text(),
            launch_config=self.launch_config_edit.text(),
        )
        try:
            self.registry.upsert(updated)
            self.registry.save()
        except (OSError, ValueError) as exc:
            QMessageBox.critical(self, 'Workspace Settings', str(exc))
            return
        self.refresh(updated.name)

    def _browse_yaml(self, target: QLineEdit) -> None:
        start = target.text().strip() or str(Path.home())
        selected, _ = QFileDialog.getOpenFileName(
            self,
            'Choose YAML configuration',
            start,
            'YAML files (*.yaml *.yml);;All files (*)',
        )
        if selected:
            target.setText(selected)

    def _build_selected(self) -> None:
        workspace = self._selected_workspace()
        if workspace:
            self.build_requested.emit(workspace.name)

    def _export_settings(self) -> None:
        selected, _ = QFileDialog.getSaveFileName(
            self,
            'Export Mobipick GUI settings',
            str(Path.home() / 'mobipick-gui-settings.yaml'),
            'YAML files (*.yaml *.yml)',
        )
        if not selected:
            return
        try:
            export_settings(Path(selected), self.registry)
        except (OSError, ValueError) as exc:
            QMessageBox.critical(self, 'Export Settings', str(exc))
            return
        QMessageBox.information(
            self,
            'Export Settings',
            f'Settings exported to:\n{selected}',
        )

    def _import_settings(self) -> None:
        if self.replace_allowed and not self.replace_allowed():
            QMessageBox.warning(
                self,
                'Import Settings',
                'Stop running workspace processes before importing settings.',
            )
            return
        selected, _ = QFileDialog.getOpenFileName(
            self,
            'Import Mobipick GUI settings',
            str(Path.home()),
            'YAML files (*.yaml *.yml);;All files (*)',
        )
        if not selected:
            return
        start = self.registry.master_folder or str(Path.home())
        master = QFileDialog.getExistingDirectory(
            self,
            'Choose the workspace master folder for imported settings',
            start,
        )
        if not master:
            return
        answer = QMessageBox.question(
            self,
            'Import Settings',
            'Importing will replace the configured workspace list and active '
            'workspace. All current tabs and log output will be discarded. '
            'Workspace files on disk will not be changed.\n\n'
            'Continue?',
        )
        if answer != QMessageBox.Yes:
            return
        try:
            import_settings(
                Path(selected),
                self.registry,
                master_folder=Path(master),
            )
        except (OSError, ValueError, yaml.YAMLError) as exc:
            QMessageBox.critical(self, 'Import Settings', str(exc))
            return
        self.refresh(self.registry.active)
        self.settings_imported.emit()
        QMessageBox.information(
            self,
            'Import Settings',
            'Settings imported. Restart the GUI to apply general GUI '
            'configuration overrides.',
        )

    def _show_graph(self) -> None:
        dialog = QDialog(self)
        dialog.setWindowTitle('ROS 1 Workspace Graph')
        dialog.resize(1000, 650)
        layout = QVBoxLayout(dialog)
        dot = self.registry.graph_dot()
        dot_binary = shutil.which('dot')
        if dot_binary:
            temp_dir = Path(tempfile.mkdtemp(prefix='mobipick-workspaces-'))
            dot_path = temp_dir / 'workspaces.dot'
            png_path = temp_dir / 'workspaces.png'
            dot_path.write_text(dot, encoding='utf-8')
            result = subprocess.run(
                [dot_binary, '-Tpng', str(dot_path), '-o', str(png_path)],
                capture_output=True,
                text=True,
                check=False,
            )
            if result.returncode == 0 and png_path.is_file():
                label = QLabel()
                label.setPixmap(QPixmap(str(png_path)))
                label.setAlignment(Qt.AlignCenter)
                scroll = QScrollArea()
                scroll.setWidget(label)
                scroll.setWidgetResizable(True)
                layout.addWidget(scroll)
            else:
                layout.addWidget(QLabel(result.stderr or 'Graphviz failed.'))
            shutil.rmtree(temp_dir, ignore_errors=True)
        else:
            message = QLabel(
                'Graphviz "dot" was not found. Install the graphviz package.\n\n'
                + dot
            )
            message.setTextInteractionFlags(Qt.TextSelectableByMouse)
            message.setWordWrap(True)
            layout.addWidget(message)
        buttons = QDialogButtonBox(QDialogButtonBox.Close)
        buttons.rejected.connect(dialog.reject)
        layout.addWidget(buttons)
        dialog.exec_()


__all__ = ['WorkspaceManagerDialog']
