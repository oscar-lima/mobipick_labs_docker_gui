"""Persistent ROS 1 workspace configuration for the GUI."""
from __future__ import annotations

import json
import os
import re
from dataclasses import asdict, dataclass, field
from pathlib import Path
from typing import Iterable

import yaml

WORKSPACE_NAME_RE = re.compile(r'^[A-Za-z0-9._-]+$')
LEGACY_WORKSPACE_IMAGES = {
    'gpt_ws': 'ozkrelo/x_mobipick_labs:gpt_ws_from_oscar_user',
    'rae_upom_mobipick_ws': (
        'ozkrelo/x_mobipick_labs:rae_ws_from_oscar_user'
    ),
}


def default_registry_path() -> Path:
    """Return the per-user workspace registry path."""
    override = os.environ.get('MOBIPICK_WORKSPACE_CONFIG')
    if override:
        return Path(override).expanduser()
    config_home = os.environ.get('XDG_CONFIG_HOME')
    base = Path(config_home).expanduser() if config_home else Path.home() / '.config'
    return base / 'mobipick-labs-docker-gui' / 'workspaces.yaml'


@dataclass
class RosWorkspace:
    """One host-side catkin workspace and its GUI profile."""

    name: str
    path: str
    extends: list[str] = field(default_factory=list)
    button_config: str = ''
    launch_config: str = ''
    sim_command: str = ''
    image: str = ''

    @property
    def directory(self) -> Path:
        return Path(self.path).expanduser().resolve()

    @property
    def setup_file(self) -> Path:
        return self.directory / 'devel' / 'setup.bash'

    @property
    def is_built(self) -> bool:
        try:
            return self.setup_file.is_file() and self.setup_file.stat().st_size > 0
        except OSError:
            return False

    def normalized(self) -> 'RosWorkspace':
        name = self.name.strip()
        if not WORKSPACE_NAME_RE.fullmatch(name):
            raise ValueError(
                'Workspace names may contain only letters, numbers, dot, '
                'underscore, and hyphen.'
            )
        directory = Path(self.path).expanduser().resolve()
        parents = [
            parent.strip()
            for parent in self.extends
            if parent and parent.strip() and parent.strip() != name
        ]
        return RosWorkspace(
            name=name,
            path=str(directory),
            image=self.image.strip(),
            extends=list(dict.fromkeys(parents)),
            button_config=self.button_config.strip(),
            launch_config=self.launch_config.strip(),
            sim_command=self.sim_command.strip(),
        )


class WorkspaceRegistry:
    """Load, validate, and persist workspace choices."""

    def __init__(
        self,
        path: Path | None = None,
        *,
        resources_root: Path | None = None,
        container_workspace_root: Path | None = None,
    ):
        self.path = Path(path) if path else default_registry_path()
        self.resources_root = Path(resources_root) if resources_root else None
        self.container_workspace_root = Path(
            container_workspace_root or Path.home() / 'ros_ws'
        )
        self.master_folder: str = ''
        self.active: str = ''
        self.workspaces: list[RosWorkspace] = []

    def load(self, *, migrate_legacy: bool = True) -> 'WorkspaceRegistry':
        """Load the registry, optionally importing the old selector."""
        if self.path.is_file():
            with self.path.open('r', encoding='utf-8') as handle:
                data = yaml.safe_load(handle) or {}
            self._load_data(data)
        elif migrate_legacy and self.resources_root:
            self._migrate_legacy()
            if self.workspaces:
                self.save()
        return self

    def _load_data(self, data) -> None:
        if not isinstance(data, dict):
            raise ValueError('Workspace registry root must be a mapping.')
        master_folder = str(data.get('master_folder') or '').strip()
        active = str(data.get('active') or '').strip()
        raw_workspaces = data.get('workspaces') or []
        if not isinstance(raw_workspaces, list):
            raise ValueError('workspaces must be a list.')
        loaded: list[RosWorkspace] = []
        for item in raw_workspaces:
            if not isinstance(item, dict):
                continue
            workspace = RosWorkspace(
                name=str(item.get('name') or ''),
                path=str(item.get('path') or ''),
                image=str(
                    item.get('image')
                    or LEGACY_WORKSPACE_IMAGES.get(
                        str(item.get('name') or '').strip(),
                        '',
                    )
                ),
                extends=self._normalize_extends(item.get('extends')),
                button_config=str(item.get('button_config') or ''),
                launch_config=str(item.get('launch_config') or ''),
                sim_command=str(item.get('sim_command') or ''),
            ).normalized()
            loaded.append(workspace)
        previous = (self.master_folder, self.active, self.workspaces)
        self.master_folder = master_folder
        self.active = active
        self.workspaces = loaded
        try:
            self._validate()
        except ValueError:
            self.master_folder, self.active, self.workspaces = previous
            raise

    @staticmethod
    def _normalize_extends(value) -> list[str]:
        if isinstance(value, str):
            return [part.strip() for part in value.split(',') if part.strip()]
        if isinstance(value, list):
            return [str(part).strip() for part in value if str(part).strip()]
        return []

    def _validate(self) -> None:
        names = [workspace.name for workspace in self.workspaces]
        if len(names) != len(set(names)):
            raise ValueError('Workspace names must be unique.')
        known = set(names)
        for workspace in self.workspaces:
            unknown = set(workspace.extends) - known
            if unknown:
                raise ValueError(
                    f'{workspace.name} extends unknown workspace(s): '
                    f'{", ".join(sorted(unknown))}'
                )
        if self.active and self.active not in known:
            self.active = ''
        self._ensure_acyclic()

    def _ensure_acyclic(self) -> None:
        graph = {
            workspace.name: list(workspace.extends)
            for workspace in self.workspaces
        }
        visiting: set[str] = set()
        visited: set[str] = set()

        def visit(name: str) -> None:
            if name in visiting:
                raise ValueError(f'Workspace inheritance contains a cycle at {name}.')
            if name in visited:
                return
            visiting.add(name)
            for parent in graph.get(name, []):
                visit(parent)
            visiting.remove(name)
            visited.add(name)

        for name in graph:
            visit(name)

    def save(self) -> None:
        """Persist the registry with an atomic replace."""
        self._validate()
        data = {
            'version': 1,
            'master_folder': self.master_folder or None,
            'active': self.active or None,
            'workspaces': [asdict(workspace) for workspace in self.workspaces],
        }
        self.path.parent.mkdir(parents=True, exist_ok=True)
        temporary = self.path.with_suffix(self.path.suffix + '.tmp')
        with temporary.open('w', encoding='utf-8') as handle:
            yaml.safe_dump(data, handle, sort_keys=False)
        temporary.replace(self.path)

    def get(self, name: str | None) -> RosWorkspace | None:
        if not name:
            return None
        return next(
            (workspace for workspace in self.workspaces if workspace.name == name),
            None,
        )

    def active_workspace(self) -> RosWorkspace | None:
        return self.get(self.active)

    def image_for(self, name: str | None, default_image: str = '') -> str:
        """Return the image assigned to a workspace or the GUI default."""
        workspace = self.get(name)
        if workspace and workspace.image:
            return workspace.image
        return str(default_image or '').strip()

    def upsert(self, workspace: RosWorkspace) -> RosWorkspace:
        normalized = workspace.normalized()
        previous = list(self.workspaces)
        existing = self.get(normalized.name)
        if existing:
            index = self.workspaces.index(existing)
            self.workspaces[index] = normalized
        else:
            self.workspaces.append(normalized)
        self.workspaces.sort(key=lambda item: item.name.lower())
        try:
            self._validate()
        except ValueError:
            self.workspaces = previous
            raise
        return normalized

    def remove(self, name: str) -> None:
        dependants = [
            workspace.name
            for workspace in self.workspaces
            if name in workspace.extends
        ]
        if dependants:
            raise ValueError(
                f'Cannot remove {name}; extended by {", ".join(dependants)}.'
            )
        self.workspaces = [
            workspace for workspace in self.workspaces if workspace.name != name
        ]
        if self.active == name:
            self.active = ''

    def discover(self, folder: Path) -> list[RosWorkspace]:
        """Add immediate child directories that look like catkin workspaces."""
        root = Path(folder).expanduser().resolve()
        if not root.is_dir():
            raise ValueError(f'Master folder does not exist: {root}')
        self.master_folder = str(root)
        discovered: list[RosWorkspace] = []
        for child in sorted(root.iterdir()):
            if not child.is_dir() or not (child / 'src').is_dir():
                continue
            existing = self.get(child.name)
            if existing:
                discovered.append(existing)
                continue
            workspace = self.with_inferred_profile(
                RosWorkspace(name=child.name, path=str(child))
            )
            self.upsert(workspace)
            discovered.append(workspace)
        return discovered

    def create(
        self,
        name: str,
        *,
        folder: Path | None = None,
        extends: Iterable[str] = (),
    ) -> RosWorkspace:
        """Create a catkin workspace directory without fake build outputs."""
        root_value = folder or (Path(self.master_folder) if self.master_folder else None)
        if root_value is None:
            raise ValueError('Choose a master folder or workspace parent folder first.')
        root = Path(root_value).expanduser().resolve()
        root.mkdir(parents=True, exist_ok=True)
        workspace = self.with_inferred_profile(
            RosWorkspace(
                name=name,
                path=str(root / name),
                extends=list(extends),
            ).normalized()
        )
        workspace.directory.mkdir(parents=True, exist_ok=True)
        (workspace.directory / 'src').mkdir(parents=True, exist_ok=True)
        self.upsert(workspace)
        return workspace

    def runtime_environment(
        self,
        *,
        empty_mount_source: Path,
        workspace_name: str | None = None,
    ) -> dict[str, str]:
        """Return compose interpolation and container environment values."""
        workspace = (
            self.get(workspace_name)
            if workspace_name is not None
            else self.active_workspace()
        )
        if workspace is None:
            empty_source = str(Path(empty_mount_source).resolve())
            return {
                'MOBIPICK_WORKSPACE_ENABLED': '0',
                'MOBIPICK_WORKSPACE_NAME': '',
                'MOBIPICK_WORKSPACE_PATH': '',
                'MOBIPICK_WORKSPACE_SETUP': '',
                'MOBIPICK_WORKSPACE_DEVEL_PATHS': '',
                'MOBIPICK_WORKSPACE_COMPAT_ROOTS': '',
                'MOBIPICK_WORKSPACE_BUILT': '0',
                'MOBIPICK_ROS_PACKAGE_PATH': '',
                'ROS_WORKSPACE': '',
                'MOBIPICK_WORKSPACE_MOUNT_SOURCE': empty_source,
                'MOBIPICK_WORKSPACE_MOUNT_TARGET': (
                    '/tmp/mobipick-empty-workspace'
                ),
            }

        mount_root = self._mount_root(workspace)
        build_order = self._workspace_build_order(workspace)
        container_root = self.container_workspace_root
        container_directories = {
            item.name: self._container_path(item.directory, mount_root)
            for item in build_order
        }
        directory = container_directories[workspace.name]
        source_paths: list[str] = []
        for index, item in enumerate(build_order):
            if index == 0 and self.is_runtime_built(item):
                continue
            item_paths = self._runtime_source_paths(
                item,
                mount_root,
                container_directories[item.name],
            )
            source_paths[0:0] = item_paths
        compat_roots = [mount_root]
        legacy_root = self._legacy_mount_target(workspace, mount_root)
        if legacy_root:
            compat_roots.append(legacy_root)
        environment = {
            'MOBIPICK_WORKSPACE_ENABLED': '1',
            'MOBIPICK_WORKSPACE_NAME': workspace.name,
            'MOBIPICK_WORKSPACE_PATH': str(directory),
            'MOBIPICK_WORKSPACE_SETUP': str(directory / 'devel' / 'setup.bash'),
            'MOBIPICK_WORKSPACE_DEVEL_PATHS': ':'.join(
                str(container_directories[item.name] / 'devel')
                for item in build_order
            ),
            'MOBIPICK_WORKSPACE_COMPAT_ROOTS': ':'.join(
                str(root)
                for root in dict.fromkeys(compat_roots)
                if root != container_root
            ),
            'MOBIPICK_WORKSPACE_BUILT': (
                '1' if self.is_runtime_built(workspace) else '0'
            ),
            'MOBIPICK_ROS_PACKAGE_PATH': ':'.join(source_paths),
            'ROS_WORKSPACE': str(directory / 'src'),
            'MOBIPICK_WORKSPACE_MOUNT_SOURCE': str(mount_root),
            'MOBIPICK_WORKSPACE_MOUNT_TARGET': str(container_root),
        }
        return environment

    def is_runtime_built(self, workspace: RosWorkspace) -> bool:
        """Return whether the workspace contains a usable catkin setup."""
        if workspace.is_built:
            return True
        candidates = (
            workspace.directory / 'devel' / 'setup.sh',
            workspace.directory
            / 'devel'
            / '.private'
            / 'catkin_tools_prebuild'
            / 'setup.bash',
            workspace.directory
            / 'devel'
            / '.private'
            / 'catkin_tools_prebuild'
            / 'setup.sh',
        )
        for setup in candidates:
            try:
                if setup.is_file() and setup.stat().st_size > 0:
                    return True
            except OSError:
                continue
        return False

    def _container_path(self, path: Path, mount_root: Path) -> Path:
        """Map a host workspace path under the canonical container root."""
        return self.container_workspace_root / path.relative_to(mount_root)

    def _runtime_source_paths(
        self,
        workspace: RosWorkspace,
        mount_root: Path,
        container_directory: Path,
    ) -> list[str]:
        """Return non-recursive source entries for a workspace at runtime."""
        if not self.is_runtime_built(workspace):
            return [str(container_directory / 'src')]

        catkin_marker = workspace.directory / 'devel' / '.catkin'
        try:
            entries = catkin_marker.read_text(encoding='utf-8').split(';')
        except OSError:
            return [str(container_directory / 'src')]

        source_roots = [mount_root, self.container_workspace_root]
        legacy_root = self._legacy_mount_target(workspace, mount_root)
        if legacy_root:
            source_roots.append(legacy_root)
        result: list[str] = []
        for entry in entries:
            raw_path = entry.strip()
            if not raw_path:
                continue
            path = Path(raw_path)
            mapped = None
            for source_root in source_roots:
                try:
                    relative = path.relative_to(source_root)
                except ValueError:
                    continue
                mapped = self.container_workspace_root / relative
                break
            if mapped is None:
                continue
            value = str(mapped)
            if value not in result:
                result.append(value)
        return result or [str(container_directory / 'src')]

    def _mount_root(self, workspace: RosWorkspace) -> Path:
        master = (
            Path(self.master_folder).expanduser().resolve()
            if self.master_folder
            else None
        )
        if master:
            try:
                for path in self._workspace_tree_directories(workspace):
                    path.relative_to(master)
            except ValueError:
                master = None
        return master or self._common_workspace_root(workspace)

    def _legacy_mount_target(
        self,
        workspace: RosWorkspace,
        mount_root: Path,
    ) -> Path | None:
        """Infer the original master path embedded in catkin symlinks."""
        legacy_roots: set[Path] = set()
        for item in self._workspace_tree(workspace):
            setup = item.setup_file
            if not setup.is_symlink():
                continue
            raw_target = Path(os.readlink(setup))
            if not raw_target.is_absolute():
                continue
            try:
                workspace_index = raw_target.parts.index(item.name)
            except ValueError:
                continue
            legacy_root = Path(*raw_target.parts[:workspace_index])
            suffix = Path(*raw_target.parts[workspace_index + 1:])
            mapped_target = item.directory / suffix
            try:
                target_exists = (
                    mapped_target.is_file()
                    and mapped_target.stat().st_size > 0
                )
            except OSError:
                target_exists = False
            if target_exists:
                legacy_roots.add(legacy_root)
        if len(legacy_roots) != 1:
            return None
        legacy_root = legacy_roots.pop()
        return legacy_root if legacy_root != mount_root else None

    def _common_workspace_root(self, workspace: RosWorkspace) -> Path:
        directories = self._workspace_tree_directories(workspace)
        try:
            common = Path(os.path.commonpath([str(path) for path in directories]))
        except ValueError:
            return workspace.directory.parent
        if common == workspace.directory:
            return workspace.directory.parent
        return common

    def _workspace_tree_directories(
        self,
        workspace: RosWorkspace,
    ) -> list[Path]:
        return [item.directory for item in self._workspace_tree(workspace)]

    def _workspace_tree(
        self,
        workspace: RosWorkspace,
    ) -> list[RosWorkspace]:
        workspaces = [workspace]
        pending = list(workspace.extends)
        seen: set[str] = set()
        while pending:
            parent_name = pending.pop()
            if parent_name in seen:
                continue
            seen.add(parent_name)
            parent = self.get(parent_name)
            if not parent:
                continue
            workspaces.append(parent)
            pending.extend(parent.extends)
        return workspaces

    def build_command(self, workspace: RosWorkspace) -> str:
        """Return a Docker shell command that builds missing underlays first."""
        commands = ['source /opt/ros/noetic/setup.bash']
        build_order = self._workspace_build_order(workspace)
        mount_root = self._mount_root(workspace)
        for item in build_order:
            container_directory = self._container_path(
                item.directory,
                mount_root,
            )
            directory = self._shell_quote(str(container_directory))
            setup = self._shell_quote(
                str(container_directory / 'devel' / 'setup.bash')
            )
            if item.name == workspace.name:
                commands.append(f'cd {directory}')
                commands.append('catkin build')
            else:
                commands.append(
                    f'if [ ! -s {setup} ]; then cd {directory} && catkin build; fi'
                )
            commands.append(
                f'if [ -s {setup} ]; then source {setup}; '
                f'else echo "Build did not create {item.setup_file}" >&2; exit 1; fi'
            )
        return '; '.join(commands)

    def _workspace_build_order(
        self,
        workspace: RosWorkspace,
    ) -> list[RosWorkspace]:
        ordered: list[RosWorkspace] = []
        visited: set[str] = set()

        def add(item: RosWorkspace) -> None:
            if item.name in visited:
                return
            for parent_name in item.extends:
                parent = self.get(parent_name)
                if parent:
                    add(parent)
            visited.add(item.name)
            ordered.append(item)

        add(workspace)
        return ordered

    def graph_dot(self) -> str:
        """Return a Graphviz DOT representation of workspace inheritance."""
        lines = [
            'digraph ros_workspaces {',
            '  rankdir=LR;',
            '  node [shape=box, style="rounded,filled", fillcolor="#e8eef7"];',
        ]
        for workspace in self.workspaces:
            label = f'{workspace.name}\\n{workspace.directory}'
            attrs = [f'label={json.dumps(label)}']
            if workspace.name == self.active:
                attrs.extend(['fillcolor="#b7e4c7"', 'penwidth=2'])
            lines.append(
                f'  {json.dumps(workspace.name)} [{", ".join(attrs)}];'
            )
        for workspace in self.workspaces:
            for parent in workspace.extends:
                lines.append(
                    f'  {json.dumps(parent)} -> {json.dumps(workspace.name)};'
                )
        lines.append('}')
        return '\n'.join(lines) + '\n'

    def with_inferred_profile(self, workspace: RosWorkspace) -> RosWorkspace:
        """Populate profile paths from matching legacy resource filenames."""
        if not workspace.image:
            workspace.image = LEGACY_WORKSPACE_IMAGES.get(workspace.name, '')
        if not self.resources_root:
            return workspace
        buttons = (
            self.resources_root
            / 'private'
            / 'button_configs'
            / f'button_commands_{workspace.name}.yaml'
        )
        launches = (
            self.resources_root
            / 'private'
            / 'experiments'
            / f'{workspace.name}_experiments.yaml'
        )
        if buttons.is_file():
            workspace.button_config = str(buttons)
        if launches.is_file():
            workspace.launch_config = str(launches)
        workspace.sim_command = self._legacy_sim_commands().get(
            workspace.name,
            '',
        )
        return workspace

    def _legacy_sim_commands(self) -> dict[str, str]:
        if not self.resources_root:
            return {}
        path = self.resources_root / 'private' / 'config.yml'
        try:
            with path.open('r', encoding='utf-8') as handle:
                data = yaml.safe_load(handle) or {}
        except (OSError, yaml.YAMLError):
            return {}
        commands = data.get('commands') if isinstance(data, dict) else {}
        if not isinstance(commands, dict):
            return {}
        result: dict[str, str] = {}
        for name, command in commands.items():
            if isinstance(command, list):
                result[str(name)] = ' '.join(str(part) for part in command)
            elif isinstance(command, str):
                result[str(name)] = command
        return result

    def _migrate_legacy(self) -> None:
        selector = self.resources_root / 'private' / 'select_ros1_ws.sh'
        try:
            text = selector.read_text(encoding='utf-8')
        except OSError:
            return
        desired_match = re.search(
            r'^\s*DESIRED_NUMBER\s*=\s*(\d+)',
            text,
            re.MULTILINE,
        )
        desired = int(desired_match.group(1)) if desired_match else -1
        block_match = re.search(
            r'^\s*rws_list\s*=\s*\((.*?)^\s*\)',
            text,
            re.MULTILINE | re.DOTALL,
        )
        if not block_match:
            return

        parsed: list[tuple[str, str]] = []
        for raw_line in block_match.group(1).splitlines():
            line = raw_line.strip()
            if not line or line.startswith('#'):
                continue
            expression, _, comment = line.partition('#')
            match = re.search(r'(?P<name>[^/\s]+)/src\s*$', expression.strip())
            if match:
                parsed.append((match.group('name'), comment.strip()))

        home = Path.home()
        likely_roots = [home / 'ros1_ws', home / 'ros_ws']
        selected_root = next((root for root in likely_roots if root.is_dir()), None)
        if selected_root:
            self.master_folder = str(selected_root.resolve())

        for index, (name, comment) in enumerate(parsed):
            candidates = [
                root / name for root in likely_roots
            ]
            directory = next(
                (candidate for candidate in candidates if candidate.is_dir()),
                candidates[0],
            )
            parent_indexes = [
                int(value)
                for value in re.findall(r'extends\s+#?(\d+)', comment)
            ]
            parents = [
                parsed[parent_index][0]
                for parent_index in parent_indexes
                if 0 <= parent_index < len(parsed)
            ]
            workspace = self.with_inferred_profile(
                RosWorkspace(
                    name=name,
                    path=str(directory),
                    extends=parents,
                )
            )
            self.workspaces.append(workspace.normalized())
            if index == desired:
                self.active = name
        self._validate()

    @staticmethod
    def _shell_quote(value: str) -> str:
        return "'" + value.replace("'", "'\"'\"'") + "'"


__all__ = [
    'RosWorkspace',
    'WorkspaceRegistry',
    'default_registry_path',
]
