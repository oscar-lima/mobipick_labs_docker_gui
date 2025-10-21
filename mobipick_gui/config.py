"""Configuration helpers for the Mobipick Labs GUI."""
from __future__ import annotations

import atexit
import copy
import os
import sys
from contextlib import ExitStack
from pathlib import Path
from typing import Dict

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
    },
    'process': {
        'qprocess_env': {
            'COMPOSE_IGNORE_ORPHANS': '1',
            'COMPOSE_FILE': str(DOCKER_COMPOSE_FILE),
            'COMPOSE_PROJECT_NAME': 'mobipick',
        },
        'compose_run_env': {
            'PYTHONUNBUFFERED': '1',
            'PYTHONIOENCODING': 'UTF-8',
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
    },
    'worlds': {
        'default': 'moelk_tables',
    },
    'terminal': {
        'launcher': 'gnome-terminal --title "{title}" -- bash -lc "{command}"',
        'title': 'Mobipick Terminal',
        'container_prefix': 'mobipick-terminal',
    },
}


def _deep_update(base: Dict, updates: Dict) -> Dict:
    for key, value in updates.items():
        if isinstance(value, dict) and isinstance(base.get(key), dict):
            base[key] = _deep_update(base[key], value)
        else:
            base[key] = value
    return base


def _load_config() -> Dict:
    config = copy.deepcopy(CONFIG_DEFAULTS)
    try:
        if CONFIG_FILE.is_file():
            with open(CONFIG_FILE, 'r', encoding='utf-8') as f:
                data = yaml.safe_load(f) or {}
            if isinstance(data, dict):
                _deep_update(config, data)
    except Exception as exc:
        print(f'Warning: failed to load configuration from {CONFIG_FILE}: {exc}', file=sys.stderr)
    return config


CONFIG = _load_config()


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
    'DOCKER_CP_CONFIG_FILE',
    'DEFAULT_YAML_PATH',
    'load_docker_cp_config',
    'PROJECT_ROOT',
    'SCRIPT_CLEAN',
    'DOCKER_COMPOSE_FILE',
]
