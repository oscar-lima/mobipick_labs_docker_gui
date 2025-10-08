"""Configuration helpers for the Mobipick Labs GUI."""
from __future__ import annotations

import copy
import sys
from pathlib import Path
from typing import Dict

import yaml

PROJECT_ROOT = Path(__file__).resolve().parent.parent
CONFIG_FILE = PROJECT_ROOT / 'config' / 'gui_settings.yaml'
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
    },
    'images': {
        'default': 'brean/mobipick_labs:noetic',
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

__all__ = [
    'CONFIG',
    'CONFIG_DEFAULTS',
    'CONFIG_FILE',
    'DEFAULT_YAML_PATH',
    'PROJECT_ROOT',
    'SCRIPT_CLEAN',
]
