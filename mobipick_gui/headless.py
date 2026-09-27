"""Headless mode: launch without opening desktop windows.

When the main window's Headless switch is on, Auto Launch skips the buttons
that open a window (RViz, RQt, viewers) and every button press appends the
button's ``headless_args`` (for example ``gui:=false`` for the simulator).
The helpers here are pure so they can be tested without Qt.
"""

from __future__ import annotations

# Builtin actions that exist only to show a window.
WINDOW_ACTIONS = frozenset({'rviz', 'rqt_tables'})


def button_opens_window(config: dict) -> bool:
    """Return whether a button's process exists to show a desktop window.

    ``opens_window`` in the button profile decides; unset, the builtin RViz
    and RQt buttons count as windows and everything else does not.
    """
    explicit = config.get('opens_window')
    if explicit is not None:
        return bool(explicit)
    return str(config.get('action') or '') in WINDOW_ACTIONS


def normalize_headless_args(raw: object) -> dict[str, str]:
    """Return ``{name: value}`` from a profile's ``headless_args`` mapping."""
    if not isinstance(raw, dict):
        return {}
    result: dict[str, str] = {}
    for name, value in raw.items():
        name_text = str(name or '').strip()
        if not name_text or value is None:
            continue
        # YAML reads an unquoted false as a boolean; roslaunch wants "false".
        value_text = str(value).lower() if isinstance(value, bool) else str(value).strip()
        if value_text:
            result[name_text] = value_text
    return result


def headless_launch_entries(
    entries: list[dict],
    window_keys: set[str],
) -> tuple[list[dict], list[str]]:
    """Drop window buttons from an Auto Launch plan.

    Returns the remaining entries and the skipped button keys. A remaining
    process that depended on a skipped one inherits that one's dependency,
    so a chain such as ``sim -> rviz -> tool`` becomes ``sim -> tool``.
    """
    by_key = {
        str(entry.get('button')): entry
        for entry in entries
        if isinstance(entry, dict) and entry.get('button')
    }
    skipped = [key for key in by_key if key in window_keys]

    def _resolve(dependency: str) -> str:
        seen: set[str] = set()
        while dependency in window_keys and dependency not in seen:
            seen.add(dependency)
            dependency = str((by_key.get(dependency) or {}).get('depends_on') or '')
        return '' if dependency in window_keys else dependency

    kept: list[dict] = []
    for entry in entries:
        if not isinstance(entry, dict):
            continue
        key = str(entry.get('button') or '')
        if key in window_keys:
            continue
        dependency = str(entry.get('depends_on') or '')
        if dependency in window_keys:
            entry = dict(entry)
            entry['depends_on'] = _resolve(dependency)
        kept.append(entry)
    return kept, skipped
