"""Configuration helpers for the Mobipick Labs GUI."""
from __future__ import annotations

import atexit
import copy
import os
import shlex
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
BUTTON_PROFILE_DIR = default_user_config_dir() / 'button_profiles'
DOCKER_CP_PROFILE_DIR = default_user_config_dir() / 'docker_cp_profiles'
WINDOW_LAYOUT_FILE = default_user_config_dir() / 'window_layout.yaml'
USER_DOCKER_CP_CONFIG_FILE = default_user_config_dir() / 'docker_cp_image_tag.yaml'


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
        'maximized': False,
        'title': 'Mobipick Labs Control',
    },
    'ros': {
        'remote_master_uri': 'http://mobipick-os-sensor:11311',
        'remote_service': 'mobipick_remote_cmd',
        'remote_enabled_by_default': False,
    },
    'window_layout': {
        'state_file': str(
            default_user_config_dir() / 'window_layouts' / '{workspace}.yaml'
        ),
        'auto_apply': True,
        'apply_delay_ms': 'auto',
    },
    'recording': {
        'enabled_by_default': False,
        'output_dir': str(USER_DATA_DIR / 'recordings'),
        'remember_output_dir': False,
        'display': '',
        'show_control_window': True,
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
        'default': 'ozkrelo/x_mobipick_labs:noetic-v1.1',
        'discovery_filters': ['mobipick'],
        'blacklist': [],
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
            'ozkrelo/x_mobipick_labs:noetic-v1.1',
            'ozkrelo/x_mobipick_labs:noetic-v1.2',
        ],
        'development_base_image': 'ozkrelo/x_mobipick_labs:noetic-v1.2',
        'development_image_repository': 'ozkrelo/x_mobipick_labs',
        'development_image_tag_template': '{user}_user_from_1.2',
        'source_repository': 'https://github.com/DFKI-NI/mobipick_labs.git',
        'source_branch': 'noetic',
        'source_workspace_name': 'clean_mobipick_labs_ws',
        'source_image': '',
        'source_install_by_default': False,
    },
    'workspace_mismatch_warning': {
        'silenced_exceptions': [],
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

REQUIRED_BUTTON_KEYS = ('sim', 'rviz')


def _deep_update(base: Dict, updates: Dict) -> Dict:
    for key, value in updates.items():
        if isinstance(value, dict) and isinstance(base.get(key), dict):
            base[key] = _deep_update(base[key], value)
        else:
            base[key] = value
    return base


def _legacy_host_user_replacements() -> dict[str, str]:
    """Return old private host-user image strings mapped to this host user."""
    legacy_user = 'osc' + 'ar'
    return {
        legacy_user: HOST_USER,
        f'{legacy_user}_user_from_1.2': f'{HOST_USER}_user_from_1.2',
        f'rae_ws_from_{legacy_user}_user': f'rae_ws_from_{HOST_USER}_user',
        f'gpt_ws_from_{legacy_user}_user': f'gpt_ws_from_{HOST_USER}_user',
    }


def _migrate_legacy_host_user_values(value):
    """Replace legacy hardcoded host-user strings in config-like data."""
    if isinstance(value, dict):
        return {
            _migrate_legacy_host_user_values(key): (
                _migrate_legacy_host_user_values(item)
            )
            for key, item in value.items()
        }
    if isinstance(value, list):
        return [_migrate_legacy_host_user_values(item) for item in value]
    if isinstance(value, str):
        migrated = value
        for old, new in _legacy_host_user_replacements().items():
            migrated = migrated.replace(old, new)
        return migrated
    return value


def _load_config() -> Dict:
    config = copy.deepcopy(CONFIG_DEFAULTS)
    for path in (CONFIG_FILE, USER_CONFIG_FILE):
        try:
            if path.is_file():
                with open(path, 'r', encoding='utf-8') as handle:
                    data = yaml.safe_load(handle) or {}
                if isinstance(data, dict):
                    data = _migrate_legacy_host_user_values(data)
                    _deep_update(config, data)
        except Exception as exc:
            print(
                f'Warning: failed to load configuration from {path}: {exc}',
                file=sys.stderr,
            )
    return config


CONFIG = _load_config()


DEFAULT_BUTTON_COMMANDS = {
    'sim': (
        'roslaunch tables_demo_bringup demo_sim.launch '
        'world_config:=${MOBIPICK_WORLD:-moelk_tables}'
    ),
    'tables': 'rosrun tables_demo_planning tables_demo_node.py',
    'rviz': (
        'rosrun rviz rviz -d '
        '$(rospack find tables_demo_bringup)/config/pick_n_place.rviz '
        '__ns:=mobipick'
    ),
    'rqt': (
        'roslaunch rqt_tables_demo rqt_tables_demo.launch '
        'namespace:=mobipick world_config:=${MOBIPICK_WORLD:-moelk_tables}'
    ),
}


def load_user_config_overrides() -> Dict:
    """Return the writable per-user GUI settings mapping."""
    try:
        if USER_CONFIG_FILE.is_file():
            with open(USER_CONFIG_FILE, 'r', encoding='utf-8') as handle:
                data = yaml.safe_load(handle) or {}
            if isinstance(data, dict):
                return _migrate_legacy_host_user_values(data)
    except Exception as exc:
        print(
            f'Warning: failed to load configuration from {USER_CONFIG_FILE}: {exc}',
            file=sys.stderr,
        )
    return {}


def save_user_config_update(updates: Dict) -> Dict:
    """Merge updates into the per-user config and live CONFIG object."""
    user_config = load_user_config_overrides()
    updates = _migrate_legacy_host_user_values(copy.deepcopy(updates))
    _deep_update(user_config, updates)
    USER_CONFIG_FILE.parent.mkdir(parents=True, exist_ok=True)
    temporary = USER_CONFIG_FILE.with_suffix(USER_CONFIG_FILE.suffix + '.tmp')
    with temporary.open('w', encoding='utf-8') as handle:
        yaml.safe_dump(user_config, handle, sort_keys=False)
    temporary.replace(USER_CONFIG_FILE)
    _deep_update(CONFIG, copy.deepcopy(updates))
    return user_config


def _button_default(key: str) -> dict:
    for entry in BUTTON_CONFIG_DEFAULTS:
        if entry.get('key') == key:
            default = copy.deepcopy(entry)
            command = _default_button_command(key, default.get('action'))
            if command:
                default['command'] = command
                default['command_is_default'] = True
            return default
    default = {'key': key, 'label': key, 'kind': 'builtin', 'action': key}
    command = _default_button_command(key)
    if command:
        default['command'] = command
        default['command_is_default'] = True
    return default


def _default_button_command(key: str, action: str | None = None) -> str:
    key_text = str(key or '').strip()
    action_text = str(action or '').strip()
    for candidate in (key_text, action_text):
        if candidate in DEFAULT_BUTTON_COMMANDS:
            return DEFAULT_BUTTON_COMMANDS[candidate]
    return ''


def _normalize_button_entry(item: dict) -> dict | None:
    key = str(item.get('key', '')).strip()
    if not key:
        return None
    action = item.get('action')
    raw_command = item.get('command')
    command_is_default = raw_command in (None, '')
    command = raw_command
    if command_is_default:
        command = _default_button_command(key, action)
    return {
        'key': key,
        'label': item.get('label') or item.get('text') or key,
        'kind': item.get('kind') or item.get('type') or 'builtin',
        'action': action,
        'command': command,
        'command_is_default': command_is_default and bool(command),
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
        'service': item.get('service') or '',
    }


def ensure_required_button_layout(entries: list[dict]) -> list[dict]:
    """Return button entries with non-removable profile buttons present."""
    result: list[dict] = []
    seen: set[str] = set()
    for entry in entries:
        if not isinstance(entry, dict):
            continue
        normalized = _normalize_button_entry(entry) or {}
        key = str(normalized.get('key') or '').strip()
        if not key or key in {'roscore', 'terminal'} or key in seen:
            continue
        result.append(normalized)
        seen.add(key)

    if 'sim' not in seen:
        result.insert(0, _button_default('sim'))
        seen.add('sim')
    if 'rviz' not in seen:
        result.append(_button_default('rviz'))
    return result


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
                    entry = _normalize_button_entry(item)
                    if entry:
                        normalized.append(entry)
            if normalized:
                entries = normalized
    except Exception as exc:
        print(
            f'Warning: failed to load button configuration from {path}: {exc}',
            file=sys.stderr,
        )
    return ensure_required_button_layout(entries)


def writable_button_config_path(source: str | Path | None) -> Path:
    """Return a writable path for saving a button profile."""
    raw_path = Path(source).expanduser() if source else BUTTON_CONFIG_FILE
    if not raw_path.is_absolute():
        raw_path = PROJECT_ROOT / raw_path

    try:
        raw_path.relative_to(PROJECT_ROOT)
    except ValueError:
        return raw_path
    return BUTTON_PROFILE_DIR / raw_path.name


def _safe_button_profile_part(value: str) -> str:
    cleaned = ''.join(
        char if char.isalnum() or char in '._-' else '_'
        for char in str(value or '').strip()
    ).strip('._-')
    return cleaned or 'workspace'


def writable_workspace_button_config_path(
    source: str | Path | None,
    workspace_name: str,
) -> Path:
    """Return a workspace-specific writable button profile path."""
    raw_path = Path(source).expanduser() if source else BUTTON_CONFIG_FILE
    name = raw_path.name or BUTTON_CONFIG_FILE.name
    workspace = _safe_button_profile_part(workspace_name)
    if (
        raw_path.parent == BUTTON_PROFILE_DIR
        and raw_path.stem.startswith(f'{workspace}_')
    ):
        return raw_path
    return BUTTON_PROFILE_DIR / f'{workspace}_{name}'


def _button_entry_for_save(entry: dict) -> dict:
    saved: dict = {}
    for key in ('key', 'label', 'kind', 'action', 'command', 'tooltip'):
        value = entry.get(key)
        if value not in (None, ''):
            saved[key] = value

    if bool(entry.get('world_config_required')):
        saved['world_config_required'] = True
        world_arg = entry.get('world_arg_name')
        if world_arg and world_arg != 'world_config':
            saved['world_arg_name'] = world_arg
    for key in ('setup', 'stop_command', 'log_command', 'service'):
        value = entry.get(key)
        if value not in (None, ''):
            saved[key] = value
    if bool(entry.get('host')):
        saved['host'] = True
    if bool(entry.get('pass_ros_master_uri')):
        saved['pass_ros_master_uri'] = True
    return saved


def save_button_layout(path: str | Path, entries: list[dict]) -> Path:
    """Persist a normalized button profile."""
    destination = Path(path).expanduser()
    destination.parent.mkdir(parents=True, exist_ok=True)
    data = {
        'buttons': [
            _button_entry_for_save(entry)
            for entry in ensure_required_button_layout(entries)
        ]
    }
    temporary = destination.with_suffix(destination.suffix + '.tmp')
    with temporary.open('w', encoding='utf-8') as handle:
        yaml.safe_dump(data, handle, sort_keys=False)
    temporary.replace(destination)
    return destination


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
    recording_start_delay_seconds = 0.0

    raw_path = str(cfg.get('config_file') or '').strip()
    raw_button_cfg = button_config_path or CONFIG.get('buttons', {}).get('config_file')
    button_path = Path(raw_button_cfg) if raw_button_cfg else None
    if button_path and not button_path.is_absolute():
        button_path = PROJECT_ROOT / button_path

    def _candidate_paths() -> list[Path]:
        candidates: list[Path] = []

        def _add_with_user_override(raw: str | Path) -> None:
            raw_candidate = Path(raw)
            candidate = raw_candidate
            if not candidate.is_absolute():
                candidate = PROJECT_ROOT / candidate
                candidates.append(LAUNCH_SEQUENCE_DIR / raw_candidate.name)
            else:
                try:
                    candidate.relative_to(PROJECT_ROOT)
                except ValueError:
                    pass
                else:
                    candidates.append(LAUNCH_SEQUENCE_DIR / candidate.name)
            candidates.append(candidate)

        if launch_config_path:
            _add_with_user_override(launch_config_path)
        if raw_path and raw_path.lower() not in {'auto', 'default'}:
            _add_with_user_override(raw_path)
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
                raw_recording = data.get('recording') or {}
                raw_recording_delay = None
                if isinstance(raw_recording, dict):
                    raw_recording_delay = raw_recording.get(
                        'start_delay_seconds',
                        raw_recording.get('extra_start_delay_seconds'),
                    )
                raw_recording_delay = data.get(
                    'recording_start_delay_seconds',
                    raw_recording_delay,
                )
                try:
                    recording_start_delay_seconds = max(
                        0.0,
                        float(raw_recording_delay or 0),
                    )
                except (TypeError, ValueError):
                    recording_start_delay_seconds = 0.0
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
        'recording_start_delay_seconds': recording_start_delay_seconds,
    }


def writable_launch_sequence_path(source: str | Path | None) -> Path:
    """Return a writable path for saving an auto-launch sequence."""
    raw_path = Path(source).expanduser() if source else LAUNCH_SEQUENCE_DIR / 'launch_sequence.yaml'
    if not raw_path.is_absolute():
        raw_path = LAUNCH_SEQUENCE_DIR / raw_path.name

    try:
        raw_path.relative_to(PROJECT_ROOT)
    except ValueError:
        return raw_path
    return LAUNCH_SEQUENCE_DIR / raw_path.name


def save_launch_sequence_plan(
    path: str | Path,
    timeline: list[dict],
    shutdown_order: list[str],
    button: dict | None = None,
    recording_start_delay_seconds: float = 0.0,
) -> Path:
    """Persist an auto-launch sequence to a user-writable YAML file."""
    destination = Path(path).expanduser()
    destination.parent.mkdir(parents=True, exist_ok=True)
    data: dict = {
        'timeline': timeline,
        'shutdown': {
            'order': shutdown_order,
        },
    }
    if button:
        data['button'] = {
            key: value
            for key, value in button.items()
            if value not in (None, '')
        }
    try:
        recording_delay = max(0.0, float(recording_start_delay_seconds or 0))
    except (TypeError, ValueError):
        recording_delay = 0.0
    data['recording'] = {
        'start_delay_seconds': recording_delay,
    }
    temporary = destination.with_suffix(destination.suffix + '.tmp')
    with open(temporary, 'w', encoding='utf-8') as handle:
        yaml.safe_dump(data, handle, sort_keys=False)
    temporary.replace(destination)
    return destination


def _normalize_docker_cp_entries(entries) -> list[dict]:
    """Return valid docker cp mapping rows."""
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
        host = host.strip()
        container = container.strip()
        if not host or not container:
            continue
        normalized.append({'host': host, 'container': container})
    return normalized


def _load_docker_cp_config_file(
    path: Path,
    *,
    include_empty: bool = False,
) -> Dict[str, Dict[str, list[dict]]]:
    """Load one docker cp config file."""
    config: Dict[str, Dict[str, list[dict]]] = {}
    if not path.is_file():
        return config
    with open(path, 'r', encoding='utf-8') as handle:
        data = yaml.safe_load(handle) or {}
    if not isinstance(data, dict):
        return config
    for key, section in data.items():
        if not isinstance(section, dict):
            continue
        host_to_container = _normalize_docker_cp_entries(
            section.get('host_to_container')
        )
        container_to_host = _normalize_docker_cp_entries(
            section.get('container_to_host')
        )
        if not include_empty and not host_to_container and not container_to_host:
            continue
        config[str(key)] = {
            'host_to_container': host_to_container,
            'container_to_host': container_to_host,
        }
    return config


def load_docker_cp_config(
    user_config_path: str | Path | None = None,
) -> Dict[str, Dict[str, list[dict]]]:
    """Load bundled docker cp mappings plus per-user overrides."""
    config: Dict[str, Dict[str, list[dict]]] = {}
    user_path = (
        Path(user_config_path).expanduser()
        if user_config_path
        else USER_DOCKER_CP_CONFIG_FILE
    )
    for path, include_empty in (
        (DOCKER_CP_CONFIG_FILE, False),
        (user_path, True),
    ):
        try:
            config.update(
                _load_docker_cp_config_file(path, include_empty=include_empty)
            )
        except Exception as exc:  # pragma: no cover - defensive logging
            print(
                f'Warning: failed to load docker cp configuration from {path}: {exc}',
                file=sys.stderr,
            )
    return config


def load_docker_cp_user_config(
    path: str | Path | None = None,
) -> Dict[str, Dict[str, list[dict]]]:
    """Load only the persistent per-user docker cp mappings."""
    config_path = Path(path).expanduser() if path else USER_DOCKER_CP_CONFIG_FILE
    try:
        return _load_docker_cp_config_file(
            config_path,
            include_empty=True,
        )
    except Exception as exc:  # pragma: no cover - defensive logging
        print(
            'Warning: failed to load docker cp configuration from '
            f'{config_path}: {exc}',
            file=sys.stderr,
        )
    return {}


def save_docker_cp_config(
    config: Dict[str, Dict[str, list[dict]]],
    path: str | Path | None = None,
) -> Path:
    """Persist docker cp mappings to the per-user configuration file."""
    normalized: Dict[str, Dict[str, list[dict]]] = {}
    for key, section in (config or {}).items():
        if not isinstance(section, dict):
            continue
        key_text = str(key).strip()
        if not key_text:
            continue
        host_to_container = _normalize_docker_cp_entries(
            section.get('host_to_container')
        )
        container_to_host = _normalize_docker_cp_entries(
            section.get('container_to_host')
        )
        normalized[key_text] = {
            'host_to_container': host_to_container,
            'container_to_host': container_to_host,
        }

    destination = Path(path).expanduser() if path else USER_DOCKER_CP_CONFIG_FILE
    destination.parent.mkdir(parents=True, exist_ok=True)
    temporary = destination.with_suffix(destination.suffix + '.tmp')
    with temporary.open('w', encoding='utf-8') as handle:
        yaml.safe_dump(normalized, handle, sort_keys=False)
    temporary.replace(destination)
    return destination


def writable_docker_cp_config_path() -> Path:
    """Return the per-user docker cp configuration path."""
    return USER_DOCKER_CP_CONFIG_FILE


def writable_workspace_docker_cp_config_path(workspace_name: str) -> Path:
    """Return a workspace-specific writable docker cp profile path."""
    workspace = _safe_button_profile_part(workspace_name)
    return DOCKER_CP_PROFILE_DIR / f'{workspace}_docker_cp_image_tag.yaml'


def user_configuration_paths(
    *,
    workspace_registry_path: Path | None = None,
    window_layout_template: str | Path | None = None,
) -> list[tuple[str, Path]]:
    """Return writable config/data paths managed by the GUI."""
    config_dir = default_user_config_dir()
    data_dir = default_user_data_dir()
    window_layout_path = (
        Path(window_layout_template).expanduser()
        if window_layout_template
        else config_dir / 'window_layouts' / '{workspace}.yaml'
    )
    registry_path = (
        Path(workspace_registry_path).expanduser()
        if workspace_registry_path
        else config_dir / 'workspaces.yaml'
    )
    return [
        ('GUI settings', USER_CONFIG_FILE),
        ('Workspace registry', registry_path),
        ('Window layouts', window_layout_path),
        ('Docker cp paths', USER_DOCKER_CP_CONFIG_FILE),
        ('Workspace Docker cp profiles', DOCKER_CP_PROFILE_DIR),
        ('Button profiles', BUTTON_PROFILE_DIR),
        ('Auto-launch profiles', LAUNCH_SEQUENCE_DIR),
        ('Imported settings profiles', config_dir / 'profiles'),
        ('Recordings', data_dir / 'recordings'),
        ('Custom image build contexts', data_dir / 'image_builds'),
    ]


def user_state_reset_paths() -> list[Path]:
    """Return top-level per-user state roots for a full GUI reset."""
    return [
        default_user_config_dir(),
        default_user_data_dir(),
    ]


def user_state_reset_command() -> str:
    """Return an opt-in shell command that deletes all per-user GUI state."""
    paths = [str(path.expanduser()) for path in user_state_reset_paths()]
    quoted_paths = ' '.join(shlex.quote(path) for path in paths)
    path_lines = ' '.join(shlex.quote(f'  - {path}') for path in paths)
    return (
        "printf '%s\\n' "
        "'DANGER: this will permanently delete Mobipick Labs Docker GUI "
        "per-user config and data:' "
        f"{path_lines} "
        "'This cannot be undone by the GUI.'; "
        "read -r -p 'Type DELETE_MOBIPICK_GUI_CONFIG to continue: ' reply; "
        "if [ \"$reply\" = DELETE_MOBIPICK_GUI_CONFIG ]; then "
        f"rm -rf -- {quoted_paths}; "
        "printf '%s\\n' 'Deleted Mobipick Labs Docker GUI per-user state.'; "
        "else printf '%s\\n' 'Aborted. No files were deleted.'; fi"
    )

__all__ = [
    'CONFIG',
    'CONFIG_DEFAULTS',
    'CONFIG_FILE',
    'DEFAULT_BUTTON_COMMANDS',
    'BUTTON_PROFILE_DIR',
    'DOCKER_CP_PROFILE_DIR',
    'USER_CONFIG_FILE',
    'USER_DOCKER_CP_CONFIG_FILE',
    'USER_DATA_DIR',
    'default_user_config_dir',
    'default_user_config_path',
    'default_user_data_dir',
    'user_state_reset_command',
    'user_state_reset_paths',
    'user_configuration_paths',
    'DOCKER_CP_CONFIG_FILE',
    'DEFAULT_YAML_PATH',
    'load_docker_cp_config',
    'load_docker_cp_user_config',
    'load_user_config_overrides',
    'PROJECT_ROOT',
    'SCRIPT_CLEAN',
    'save_launch_sequence_plan',
    'save_button_layout',
    'save_docker_cp_config',
    'save_user_config_update',
    'BUTTON_CONFIG_FILE',
    'BUTTON_CONFIG_DEFAULTS',
    'REQUIRED_BUTTON_KEYS',
    'ensure_required_button_layout',
    'load_button_layout',
    'DOCKER_COMPOSE_FILE',
    'LAUNCH_SEQUENCE_DIR',
    'WINDOW_LAYOUT_FILE',
    'load_launch_sequence_plan',
    'writable_docker_cp_config_path',
    'writable_workspace_docker_cp_config_path',
    'writable_button_config_path',
    'writable_workspace_button_config_path',
    'writable_launch_sequence_path',
]
