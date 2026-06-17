"""Configuration helpers for the Mobipick Labs GUI."""
from __future__ import annotations

import atexit
import copy
import os
import sys
from contextlib import ExitStack
from pathlib import Path
from typing import Dict

try:
    import grp
except ImportError:  # pragma: no cover - Windows safety
    grp = None  # type: ignore[assignment]

try:
    import pwd
except ImportError:  # pragma: no cover - Windows safety
    pwd = None  # type: ignore[assignment]

import yaml

try:  # Python 3.9+
    from importlib import resources as importlib_resources
except ImportError:  # pragma: no cover - fallback for Python 3.8
    import importlib_resources  # type: ignore


def _resolve_project_root() -> Path:
    """Return the directory that stores bundled assets."""

    env_root = os.environ.get('MOBIPICK_GUI_DATA_ROOT')
    if env_root:
        candidate = Path(env_root).expanduser()
        if candidate.is_dir():
            return candidate

    package_dir = Path(__file__).resolve().parent
    resources_dir = package_dir / 'resources'
    if resources_dir.is_dir():
        return resources_dir

    try:
        data_pkg = importlib_resources.files('mobipick_gui').joinpath('resources')
    except (AttributeError, ModuleNotFoundError):  # pragma: no cover - safety
        return package_dir

    _ASSET_STACK = _resolve_project_root._asset_stack  # type: ignore[attr-defined]
    resolved = _ASSET_STACK.enter_context(importlib_resources.as_file(data_pkg))
    return Path(resolved)


_resolve_project_root._asset_stack = ExitStack()  # type: ignore[attr-defined]
atexit.register(_resolve_project_root._asset_stack.close)  # type: ignore[attr-defined]

PROJECT_ROOT = _resolve_project_root()
DOCKER_COMPOSE_FILE = PROJECT_ROOT / 'docker-compose.yml'
CONFIG_FILE = PROJECT_ROOT / 'config' / 'gui_settings.yaml'
DOCKER_CP_CONFIG_FILE = PROJECT_ROOT / 'config' / 'docker_cp_image_tag.yaml'
SCRIPT_CLEAN = str(PROJECT_ROOT / 'clean.bash')
DEFAULT_YAML_PATH = str(PROJECT_ROOT / 'config' / 'worlds.yaml')
BUTTON_CONFIG_FILE = PROJECT_ROOT / 'config' / 'button_commands_labs.yaml'


def default_user_config_dir() -> Path:
    """Return the writable per-user configuration directory."""
    config_home = os.environ.get('XDG_CONFIG_HOME')
    base = Path(config_home).expanduser() if config_home else Path.home() / '.config'
    return base / 'mobipick-labs-docker-gui'


def default_user_data_dir() -> Path:
    """Return the writable per-user data directory."""
    data_home = os.environ.get('XDG_DATA_HOME')
    base = (
        Path(data_home).expanduser()
        if data_home
        else Path.home() / '.local' / 'share'
    )
    return base / 'mobipick-labs-docker-gui'


def default_user_config_path() -> Path:
    """Return the optional per-user GUI settings override path."""
    override = os.environ.get('MOBIPICK_GUI_CONFIG')
    if override:
        return Path(override).expanduser()
    return default_user_config_dir() / 'gui_settings.yaml'


USER_CONFIG_FILE = default_user_config_path()
USER_DATA_DIR = default_user_data_dir()
LAUNCH_SEQUENCE_DIR = default_user_config_dir() / 'launch_sequences'
WINDOW_LAYOUT_FILE = default_user_config_dir() / 'window_layout.yaml'


def _detect_numeric_id(getter_name: str, env_candidates: tuple[str, ...], fallback: str) -> str:
    getter = getattr(os, getter_name, None)
    if callable(getter):
        try:
            value = getter()
        except OSError:
            value = None
        else:
            if value is not None:
                return str(value)
    for env_name in env_candidates:
        raw = os.environ.get(env_name)
        if raw and raw.isdigit():
            return raw
    return fallback


HOST_UID = _detect_numeric_id('getuid', ('SUDO_UID', 'UID'), '0')
HOST_GID = _detect_numeric_id('getgid', ('SUDO_GID', 'GID'), HOST_UID)


def _detect_host_user() -> str:
    for env_name in ('SUDO_USER', 'USER'):
        value = os.environ.get(env_name)
        if value and value != 'root':
            return value
    if pwd is not None:
        try:
            record = pwd.getpwuid(int(HOST_UID))
        except (KeyError, ValueError):
            pass
        else:
            if record.pw_name and record.pw_name != 'root':
                return record.pw_name
    return f'host{HOST_UID}'


def _detect_host_group() -> str:
    if grp is not None:
        try:
            record = grp.getgrgid(int(HOST_GID))
        except (KeyError, ValueError):
            pass
        else:
            if record.gr_name:
                return record.gr_name
    return f'hostgrp{HOST_GID}'


def _detect_host_home() -> str:
    for env_name in ('SUDO_HOME', 'HOME'):
        value = os.environ.get(env_name)
        if value:
            expanded = Path(value).expanduser()
            if expanded.is_dir():
                return str(expanded)
    if pwd is not None:
        try:
            record = pwd.getpwuid(int(HOST_UID))
        except (KeyError, ValueError):
            pass
        else:
            if record.pw_dir:
                return record.pw_dir
    return '/root'


HOST_USER = _detect_host_user()
HOST_GROUP = _detect_host_group()
HOST_HOME = _detect_host_home()

CONFIG_DEFAULTS: Dict[str, Dict] = {
    'log': {
        'max_block_count': 20000,
        'flush_interval_ms': 30,
        'background_color': '#000000',
        'text_color': '#ffffff',
        'gui_log_color': '#ff00ff',
        'command_log_color': '#4da3ff',
        'font_family': 'monospace',
        'scroll_tolerance_min': 2,
    },
    'window': {
        'geometry': [100, 100, 1100, 780],
        'title': 'Mobipick Labs Control',
    },
    'ros': {
        'remote_master_uri': 'http://mobipick-os-sensor:11311',
        'remote_service': 'mobipick_remote_cmd',
        'remote_enabled_by_default': False,
    },
    'window_layout': {
        'state_file': str(WINDOW_LAYOUT_FILE),
        'auto_apply': True,
        'apply_delay_ms': 'auto',
    },
    'recording': {
        'enabled_by_default': False,
        'output_dir': str(USER_DATA_DIR / 'recordings'),
        'workspace_name': 'workspace',
        'resolution': '3440x1440',
        'presets': ['1920x1080', '2560x1440', '3440x1440'],
        'include_detected_resolution': True,
    },
    'timers': {
        'poll_ms': 1200,
        'sigint_check_ms': 100,
        'custom_tab_sigint_delay_ms': 1000,
        'sim_shutdown_delay_ms': 2500,
        'roscore_start_delay_ms': 1000,
    },
    'buttons': {
        'sim_toggle': {
            'padding_px': 6,
            'disabled_opacity': 0.85,
            'states': {
                'green': {'bg': '#28a745', 'fg': 'white'},
                'red': {'bg': '#dc3545', 'fg': 'white'},
                'yellow': {'bg': '#ffc107', 'fg': 'black'},
                'grey': {'bg': '#6c757d', 'fg': 'white'},
            },
        },
        'close': {
            'text': '✕',
            'tooltip': 'Close tab',
            'size': 18,
            'stylesheet': 'QPushButton { border: none; padding: 0px; }',
        },
        'config_file': str(BUTTON_CONFIG_FILE),
    },
    'process': {
        'container_scripts_dir': '/scripts_430ofkjl04fsw',
        'qprocess_env': {
            'COMPOSE_IGNORE_ORPHANS': '1',
            'COMPOSE_FILE': str(DOCKER_COMPOSE_FILE),
            'COMPOSE_PROJECT_NAME': 'mobipick',
            'MOBIPICK_UID': HOST_UID,
            'MOBIPICK_GID': HOST_GID,
            'MOBIPICK_HOST_USER': HOST_USER,
            'MOBIPICK_HOST_GROUP': HOST_GROUP,
            'MOBIPICK_HOST_HOME': HOST_HOME,
        },
        'compose_run_env': {
            'PYTHONUNBUFFERED': '1',
            'PYTHONIOENCODING': 'UTF-8',
            'MOBIPICK_ROS_USE_IP': '1',
            'MOBIPICK_UID': HOST_UID,
            'MOBIPICK_GID': HOST_GID,
            'MOBIPICK_HOST_USER': HOST_USER,
            'MOBIPICK_HOST_GROUP': HOST_GROUP,
            'MOBIPICK_HOST_HOME': HOST_HOME,
        },
    },
    'exit': {
        'dialog_title': 'Shutting Down',
        'dialog_message': 'Shutting down simulation and cleaning up. Please wait...',
        'log_start_message': 'Shutting down containers before exit...',
        'log_done_message': 'Shutdown complete. Exiting...',
        'docker_stop_timeout': 3,
    },
    'images': {
        'default': 'ozkrelo/mobipick_labs:noetic',
        'discovery_filters': ['mobipick'],
        'include_none_tag': False,
        'related_container_keywords': ['mobipick', 'mobipick_cmd', 'mobipick-run', 'rqt', 'rviz'],
        'related_image_keywords': ['mobipick_labs'],
        'manage_dialog_detail': 'simple',
        'default_user': 'root',
        'default_supports_host_workspaces': False,
        'profiles': [
            {
                'ref': 'ozkrelo/mobipick_labs:noetic',
                'user': 'root',
                'supports_host_workspaces': False,
                'compatible_workspaces': ['Docker image default'],
                'workdir': '/root/catkin_ws',
                'description': (
                    'Public image with a root-owned workspace baked into '
                    'the image.'
                ),
            },
            {
                'ref': 'ozkrelo/x_mobipick_labs:noetic-v1.1',
                'user': 'root',
                'supports_host_workspaces': False,
                'compatible_workspaces': ['Docker image default'],
                'workdir': '/root/catkin_ws',
                'description': (
                    'Public X11 image with a root-owned workspace baked '
                    'into the image.'
                ),
            },
            {
                'ref': 'ozkrelo/x_mobipick_labs:noetic-v1.2',
                'user': 'root',
                'supports_host_workspaces': False,
                'compatible_workspaces': ['Docker image default'],
                'workdir': '/root/catkin_ws',
                'description': (
                    'Public X11 image with a root-owned workspace baked '
                    'into the image.'
                ),
            },
            {
                'match': '*_user*',
                'user': 'host',
                'supports_host_workspaces': True,
                'compatible_workspaces': [],
                'description': (
                    'Fallback for local development images with a user '
                    'matching the host.'
                ),
            },
        ],
    },
    'worlds': {
        'default': 'moelk_tables',
    },
    'terminal': {
        'launcher': 'gnome-terminal --title "{title}" -- bash -lc "{command}"',
        'title': 'Mobipick Terminal',
        'container_prefix': 'mobipick-terminal',
        'drop_to_host_user': True,
    },
    'setup_wizard': {
        'show_on_first_run': True,
        'public_images': [
            'ozkrelo/mobipick_labs:noetic',
            'ozkrelo/x_mobipick_labs:noetic-v1.1',
            'ozkrelo/x_mobipick_labs:noetic-v1.2',
        ],
        'development_base_image': 'ozkrelo/x_mobipick_labs:noetic-v1.2',
        'development_image_repository': 'ozkrelo/x_mobipick_labs',
        'development_image_tag_template': '{user}_user_from_1.2',
    },
    'launch_sequence': {
        'config_file': 'auto',
        'label': 'Auto Launch',
        'start_text': 'Start Auto Launch',
        'stop_text': 'Stop Auto Launch',
        'tooltip': 'Launch the configured startup sequence',
        'retry_delay_ms': 750,
        'max_retry_attempts': 6,
    },
}

BUTTON_CONFIG_DEFAULTS = [
    {
        'key': 'sim',
        'label': 'Sim',
        'kind': 'builtin',
        'action': 'sim',
        'tooltip': 'Start or stop the simulator',
        'world_config_required': False,
        'world_arg_name': 'world_config',
        'setup': None,
        'host': False,
        'stop_command': None,
        'log_command': None,
        'pass_ros_master_uri': False,
    },
    {
        'key': 'tables',
        'label': 'Tables Demo',
        'kind': 'builtin',
        'action': 'tables_demo',
        'tooltip': 'Launch the tables demo planning node',
        'world_config_required': False,
        'world_arg_name': 'world_config',
        'setup': None,
        'host': False,
        'stop_command': None,
        'log_command': None,
        'pass_ros_master_uri': False,
    },
    {
        'key': 'rviz',
        'label': 'RViz',
        'kind': 'builtin',
        'action': 'rviz',
        'tooltip': 'Open RViz with the pick and place config',
        'world_config_required': False,
        'world_arg_name': 'world_config',
        'setup': None,
        'host': False,
        'stop_command': None,
        'log_command': None,
        'pass_ros_master_uri': False,
    },
    {
        'key': 'rqt',
        'label': 'RQt Tables',
        'kind': 'builtin',
        'action': 'rqt_tables',
        'tooltip': 'Open the RQt tables demo UI',
        'world_config_required': False,
        'world_arg_name': 'world_config',
        'setup': None,
        'host': False,
        'stop_command': None,
        'log_command': None,
        'pass_ros_master_uri': False,
    },
]


def _deep_update(base: Dict, updates: Dict) -> Dict:
    for key, value in updates.items():
        if isinstance(value, dict) and isinstance(base.get(key), dict):
            base[key] = _deep_update(base[key], value)
        else:
            base[key] = value
    return base


def _load_config() -> Dict:
    config = copy.deepcopy(CONFIG_DEFAULTS)
    for path in (CONFIG_FILE, USER_CONFIG_FILE):
        try:
            if path.is_file():
                with open(path, 'r', encoding='utf-8') as handle:
                    data = yaml.safe_load(handle) or {}
                if isinstance(data, dict):
                    _deep_update(config, data)
        except Exception as exc:
            print(
                f'Warning: failed to load configuration from {path}: {exc}',
                file=sys.stderr,
            )
    return config


CONFIG = _load_config()


def load_user_config_overrides() -> Dict:
    """Return the writable per-user GUI settings mapping."""
    try:
        if USER_CONFIG_FILE.is_file():
            with open(USER_CONFIG_FILE, 'r', encoding='utf-8') as handle:
                data = yaml.safe_load(handle) or {}
            return data if isinstance(data, dict) else {}
    except Exception as exc:
        print(
            f'Warning: failed to load configuration from {USER_CONFIG_FILE}: {exc}',
            file=sys.stderr,
        )
    return {}


def save_user_config_update(updates: Dict) -> Dict:
    """Merge updates into the per-user config and live CONFIG object."""
    user_config = load_user_config_overrides()
    _deep_update(user_config, copy.deepcopy(updates))
    USER_CONFIG_FILE.parent.mkdir(parents=True, exist_ok=True)
    temporary = USER_CONFIG_FILE.with_suffix(USER_CONFIG_FILE.suffix + '.tmp')
    with temporary.open('w', encoding='utf-8') as handle:
        yaml.safe_dump(user_config, handle, sort_keys=False)
    temporary.replace(USER_CONFIG_FILE)
    _deep_update(CONFIG, copy.deepcopy(updates))
    return user_config


def load_button_layout(config_path: str | Path | None = None) -> list[dict]:
    """Load configurable command buttons."""
    entries = copy.deepcopy(BUTTON_CONFIG_DEFAULTS)
    raw_path = (
        config_path
        or CONFIG.get('buttons', {}).get('config_file')
        or str(BUTTON_CONFIG_FILE)
    )
    path = Path(raw_path)
    if not path.is_absolute():
        path = PROJECT_ROOT / raw_path
    try:
        if path.is_file():
            with open(path, 'r', encoding='utf-8') as handle:
                data = yaml.safe_load(handle) or {}
            raw = data.get('buttons') if isinstance(data, dict) else None
            if raw is None and isinstance(data, list):
                raw = data
            normalized: list[dict] = []
            if isinstance(raw, list):
                for item in raw:
                    if not isinstance(item, dict):
                        continue
                    key = str(item.get('key', '')).strip()
                    if not key:
                        continue
                    normalized.append(
                        {
                            'key': key,
                            'label': item.get('label') or item.get('text') or key,
                            'kind': item.get('kind') or item.get('type') or 'builtin',
                            'action': item.get('action'),
                            'command': item.get('command'),
                            'tooltip': item.get('tooltip'),
                            'requires_roscore': item.get('requires_roscore', True),
                            'reuse_tab': item.get('reuse_tab', False),
                            'world_config_required': item.get('world_config_required', False),
                            'world_arg_name': item.get('world_arg_name', 'world_config'),
                            'setup': item.get('setup') or item.get('pre_command') or item.get('prologue'),
                            'host': bool(item.get('host', False)),
                            'stop_command': item.get('stop_command'),
                            'log_command': item.get('log_command'),
                            'pass_ros_master_uri': item.get('pass_ros_master_uri', False),
                        }
                    )
            if normalized:
                entries = normalized
    except Exception as exc:
        print(
            f'Warning: failed to load button configuration from {path}: {exc}',
            file=sys.stderr,
        )
    return entries


def load_launch_sequence_plan(
    button_config_path: str | Path | None = None,
    launch_config_path: str | Path | None = None,
) -> Dict:
    """Load auto-launch timeline and button text."""

    def _normalize_timeline(entries) -> list[dict]:
        if not isinstance(entries, list):
            return []
        normalized: list[dict] = []
        for item in entries:
            if not isinstance(item, dict):
                continue
            key = str(item.get('button') or item.get('key') or '').strip()
            if not key:
                continue
            at_value = item.get('at_seconds', item.get('at', item.get('time_seconds')))
            try:
                at_seconds = max(0.0, float(at_value))
            except (TypeError, ValueError):
                continue
            normalized.append({'button': key, 'at_seconds': at_seconds})
        normalized.sort(key=lambda e: (e['at_seconds'], e['button']))
        return normalized

    cfg = CONFIG.get('launch_sequence', {})
    button_defaults = {
        'label': cfg.get('label', 'Auto Launch') or 'Auto Launch',
        'start_text': cfg.get('start_text', 'Start Auto Launch') or 'Start Auto Launch',
        'stop_text': cfg.get('stop_text', 'Stop Auto Launch') or 'Stop Auto Launch',
        'tooltip': cfg.get('tooltip', '') or '',
    }
    retry_delay_ms = int(cfg.get('retry_delay_ms', 750) or 0)
    max_retry_attempts = int(cfg.get('max_retry_attempts', 6) or 0)

    raw_path = str(cfg.get('config_file') or '').strip()
    raw_button_cfg = button_config_path or CONFIG.get('buttons', {}).get('config_file')
    button_path = Path(raw_button_cfg) if raw_button_cfg else None
    if button_path and not button_path.is_absolute():
        button_path = PROJECT_ROOT / button_path

    def _candidate_paths() -> list[Path]:
        candidates: list[Path] = []
        if launch_config_path:
            explicit = Path(launch_config_path)
            if not explicit.is_absolute():
                explicit = PROJECT_ROOT / explicit
            candidates.append(explicit)
        if raw_path and raw_path.lower() not in {'auto', 'default'}:
            custom = Path(raw_path)
            if not custom.is_absolute():
                custom = PROJECT_ROOT / custom
            candidates.append(custom)
        stem_candidates: list[str] = []
        if button_path:
            stem_candidates.append(button_path.stem)
            if button_path.stem.startswith('button_commands_'):
                stem_candidates.append(button_path.stem.replace('button_commands_', '', 1))
            if button_path.stem.endswith('_ws'):
                stem_candidates.append(button_path.stem[: -len('_ws')])
        stem_candidates.append('launch_sequence')
        filenames: list[str] = []
        for stem in stem_candidates:
            filenames.extend(
                [
                    f'{stem}_launch.yaml',
                    f'{stem}_sequence.yaml',
                    f'{stem}_autolaunch.yaml',
                    f'{stem}_experiments.yaml',
                    f'{stem}.yaml',
                ]
            )
        search_dirs = [
            LAUNCH_SEQUENCE_DIR,
        ]
        for directory in search_dirs:
            for name in filenames:
                candidates.append(directory / name)
        return candidates

    path: Path | None = None
    for candidate in _candidate_paths():
        if candidate.is_file():
            path = candidate
            break
        if path is None:
            path = candidate
    if path is None:
        path = LAUNCH_SEQUENCE_DIR / 'launch_sequence.yaml'

    timeline: list[dict] = []
    shutdown_order: list[str] = []
    button_cfg = dict(button_defaults)
    shutdown_skip: list[str] = []
    try:
        if path.is_file():
            with open(path, 'r', encoding='utf-8') as handle:
                data = yaml.safe_load(handle) or {}
            if isinstance(data, dict):
                timeline = _normalize_timeline(data.get('timeline', []))
                shutdown_section = data.get('shutdown') or {}
                raw_order = []
                if isinstance(shutdown_section, dict):
                    raw_order = shutdown_section.get('order') or shutdown_section.get('buttons') or []
                    raw_skip = shutdown_section.get('skip') or shutdown_section.get('ignore') or []
                    if isinstance(raw_skip, list):
                        shutdown_skip = [str(entry).strip() for entry in raw_skip if str(entry).strip()]
                elif isinstance(data.get('shutdown_order'), list):
                    raw_order = data.get('shutdown_order') or []
                if isinstance(raw_order, list):
                    shutdown_order = [str(entry).strip() for entry in raw_order if str(entry).strip()]
                if isinstance(data.get('button'), dict):
                    merged = dict(button_cfg)
                    merged.update({k: v for k, v in data['button'].items() if v is not None})
                    button_cfg = merged
    except Exception as exc:
        print(f'Warning: failed to load auto launch configuration from {path}: {exc}', file=sys.stderr)

    if not shutdown_order:
        shutdown_order = list(dict.fromkeys(entry['button'] for entry in reversed(timeline)))

    return {
        'source': str(path),
        'timeline': timeline,
        'shutdown_order': shutdown_order,
        'shutdown_skip': shutdown_skip,
        'button': button_cfg,
        'retry_delay_ms': retry_delay_ms,
        'max_retry_attempts': max_retry_attempts,
    }


def load_docker_cp_config() -> Dict[str, Dict[str, list[dict]]]:
    """Load optional docker cp mappings keyed by image references."""

    def _normalize(entries) -> list[dict]:
        if not isinstance(entries, list):
            return []
        normalized: list[dict] = []
        for item in entries:
            if not isinstance(item, dict):
                continue
            host = item.get('host') or item.get('host_path')
            container = item.get('container') or item.get('container_path')
            if not isinstance(host, str) or not isinstance(container, str):
                continue
            normalized.append({'host': host, 'container': container})
        return normalized

    config: Dict[str, Dict[str, list[dict]]] = {}
    try:
        if DOCKER_CP_CONFIG_FILE.is_file():
            with open(DOCKER_CP_CONFIG_FILE, 'r', encoding='utf-8') as handle:
                data = yaml.safe_load(handle) or {}
            if isinstance(data, dict):
                for key, section in data.items():
                    if not isinstance(section, dict):
                        continue
                    host_to_container = _normalize(section.get('host_to_container'))
                    container_to_host = _normalize(section.get('container_to_host'))
                    if not host_to_container and not container_to_host:
                        continue
                    config[str(key)] = {
                        'host_to_container': host_to_container,
                        'container_to_host': container_to_host,
                    }
    except Exception as exc:  # pragma: no cover - defensive logging
        print(
            f'Warning: failed to load docker cp configuration from {DOCKER_CP_CONFIG_FILE}: {exc}',
            file=sys.stderr,
        )
    return config

__all__ = [
    'CONFIG',
    'CONFIG_DEFAULTS',
    'CONFIG_FILE',
    'USER_CONFIG_FILE',
    'USER_DATA_DIR',
    'default_user_config_dir',
    'default_user_config_path',
    'default_user_data_dir',
    'DOCKER_CP_CONFIG_FILE',
    'DEFAULT_YAML_PATH',
    'load_docker_cp_config',
    'load_user_config_overrides',
    'PROJECT_ROOT',
    'SCRIPT_CLEAN',
    'save_user_config_update',
    'BUTTON_CONFIG_FILE',
    'BUTTON_CONFIG_DEFAULTS',
    'load_button_layout',
    'DOCKER_COMPOSE_FILE',
    'LAUNCH_SEQUENCE_DIR',
    'WINDOW_LAYOUT_FILE',
    'load_launch_sequence_plan',
]
