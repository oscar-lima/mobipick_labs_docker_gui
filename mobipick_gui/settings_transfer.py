"""Portable import and export of per-user GUI settings."""
from __future__ import annotations

from dataclasses import asdict
from pathlib import Path

import yaml

from .config import USER_CONFIG_FILE, default_user_config_dir
from .workspaces import RosWorkspace, WorkspaceRegistry

SETTINGS_FORMAT = 'mobipick-labs-docker-gui-settings'
SETTINGS_VERSION = 1
PROFILE_FIELDS = ('button_config', 'launch_config')


def export_settings(
    destination: Path,
    registry: WorkspaceRegistry,
    *,
    user_config_path: Path = USER_CONFIG_FILE,
) -> None:
    """Write the registry, user overrides, and profile files to one YAML file."""
    master = (
        Path(registry.master_folder).expanduser().resolve()
        if registry.master_folder
        else None
    )
    workspaces = []
    profiles: dict[str, dict[str, dict[str, str]]] = {}
    for workspace in registry.workspaces:
        item = asdict(workspace)
        item.pop('path', None)
        try:
            relative = workspace.directory.relative_to(master) if master else None
        except ValueError:
            relative = None
        item['relative_path'] = str(relative or Path(workspace.name))
        for field_name in PROFILE_FIELDS:
            raw_path = getattr(workspace, field_name)
            profile = _read_profile(raw_path)
            if profile:
                profiles.setdefault(workspace.name, {})[field_name] = profile
            item[field_name] = ''
        workspaces.append(item)

    data = {
        'format': SETTINGS_FORMAT,
        'version': SETTINGS_VERSION,
        'workspace_registry': {
            'active': registry.active or None,
            'workspaces': workspaces,
        },
        'profiles': profiles,
        'gui_settings': _read_yaml_mapping(user_config_path),
    }
    _write_yaml_atomic(Path(destination), data)


def import_settings(
    source: Path,
    registry: WorkspaceRegistry,
    *,
    master_folder: Path,
    user_config_path: Path = USER_CONFIG_FILE,
    profiles_dir: Path | None = None,
) -> None:
    """Import a portable settings file under the selected workspace root."""
    with Path(source).open('r', encoding='utf-8') as handle:
        data = yaml.safe_load(handle) or {}
    if not isinstance(data, dict) or data.get('format') != SETTINGS_FORMAT:
        raise ValueError('This is not a Mobipick Labs Docker GUI settings file.')
    if data.get('version') != SETTINGS_VERSION:
        raise ValueError(
            f'Unsupported settings version: {data.get("version")!r}.'
        )

    raw_registry = data.get('workspace_registry') or {}
    raw_workspaces = raw_registry.get('workspaces') or []
    if not isinstance(raw_registry, dict) or not isinstance(raw_workspaces, list):
        raise ValueError('The exported workspace registry is invalid.')

    target_master = Path(master_folder).expanduser().resolve()
    profile_root = Path(
        profiles_dir or default_user_config_dir() / 'profiles'
    )
    imported: list[RosWorkspace] = []
    pending_profiles: list[tuple[Path, str]] = []
    raw_profiles = data.get('profiles') or {}
    for item in raw_workspaces:
        if not isinstance(item, dict):
            raise ValueError('Each exported workspace must be a mapping.')
        name = str(item.get('name') or '').strip()
        relative = _safe_relative_path(item.get('relative_path'), name)
        workspace_profiles = (
            raw_profiles.get(name, {}) if isinstance(raw_profiles, dict) else {}
        )
        profile_paths = {}
        for field_name in PROFILE_FIELDS:
            profile = (
                workspace_profiles.get(field_name)
                if isinstance(workspace_profiles, dict)
                else None
            )
            if not isinstance(profile, dict):
                profile_paths[field_name] = ''
                continue
            filename = _safe_filename(
                profile.get('filename'),
                f'{field_name}.yaml',
            )
            profile_path = profile_root / name / f'{field_name}-{filename}'
            profile_paths[field_name] = str(profile_path)
            pending_profiles.append(
                (profile_path, str(profile.get('content') or ''))
            )
        imported.append(
            RosWorkspace(
                name=name,
                path=str(target_master / relative),
                image=str(item.get('image') or ''),
                extends=WorkspaceRegistry._normalize_extends(
                    item.get('extends')
                ),
                button_config=profile_paths['button_config'],
                launch_config=profile_paths['launch_config'],
                sim_command=str(item.get('sim_command') or ''),
            ).normalized()
        )

    candidate = WorkspaceRegistry(
        registry.path,
        resources_root=registry.resources_root,
        container_workspace_root=registry.container_workspace_root,
    )
    candidate.master_folder = str(target_master)
    candidate.active = str(raw_registry.get('active') or '')
    candidate.workspaces = imported
    candidate._validate()

    for path, content in pending_profiles:
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(content, encoding='utf-8')
    gui_settings = data.get('gui_settings')
    if isinstance(gui_settings, dict):
        _write_yaml_atomic(Path(user_config_path), gui_settings)

    registry.master_folder = candidate.master_folder
    registry.active = candidate.active
    registry.workspaces = candidate.workspaces
    registry.save()


def _read_profile(raw_path: str) -> dict[str, str] | None:
    if not raw_path:
        return None
    path = Path(raw_path).expanduser()
    try:
        content = path.read_text(encoding='utf-8')
    except OSError:
        return None
    return {'filename': path.name, 'content': content}


def _read_yaml_mapping(path: Path) -> dict:
    try:
        with Path(path).open('r', encoding='utf-8') as handle:
            data = yaml.safe_load(handle) or {}
    except OSError:
        return {}
    return data if isinstance(data, dict) else {}


def _safe_relative_path(value, fallback: str) -> Path:
    path = Path(str(value or fallback))
    if path.is_absolute() or '..' in path.parts:
        return Path(fallback)
    return path


def _safe_filename(value, fallback: str) -> str:
    name = Path(str(value or fallback)).name
    return name if name not in {'', '.', '..'} else fallback


def _write_yaml_atomic(path: Path, data: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + '.tmp')
    with temporary.open('w', encoding='utf-8') as handle:
        yaml.safe_dump(data, handle, sort_keys=False)
    temporary.replace(path)


__all__ = ['export_settings', 'import_settings']
