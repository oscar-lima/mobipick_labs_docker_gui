"""Group boxes of the Advanced Launch Options dialog: options that depend on each other sit together.

A workspace keeps a groups file beside its button profile, named after the profile
(``<profile stem>_option_groups.yaml``)::

    version: 1
    groups:
    - title: Voice agent
      options: [voice_agent, voice_languages, voice_ack]

The dialog shows one titled box per group, in the file's order, with the options of that group; an option no group
names lands in a last box, "Other". A group lists argument names only: slots, saved values, scopes and the remote
API are untouched, and without the file the dialog keeps its plain list. Problems are collected, not raised.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path

import yaml

GROUPS_FILE_SUFFIX = '_option_groups.yaml'
OTHER_TITLE = 'Other'


@dataclass
class OptionGroups:
    groups: list[tuple[str, tuple[str, ...]]] = field(default_factory=list)   # (title, option names), in order
    path: Path | None = None
    errors: list[str] = field(default_factory=list)

    def __bool__(self) -> bool:
        return bool(self.groups)

    def title_of(self, option: str) -> str:
        """The title of the group that names ``option``; ``OTHER_TITLE`` for an option no group names."""
        for title, names in self.groups:
            if option in names:
                return title
        return OTHER_TITLE

    def order(self) -> list[str]:
        """Titles in display order, ``OTHER_TITLE`` last."""
        return [title for title, _names in self.groups] + [OTHER_TITLE]


def option_groups_path(button_config: str | Path | None) -> Path | None:
    if not button_config:
        return None
    profile = Path(button_config).expanduser()
    candidate = profile.with_name(f'{profile.stem}{GROUPS_FILE_SUFFIX}')
    return candidate if candidate.is_file() else None


def parse_option_groups(data, path: Path | None = None) -> OptionGroups:
    """Build the groups from parsed YAML, collecting problems instead of raising."""
    loaded = OptionGroups(path=path)
    raw = data.get('groups') if isinstance(data, dict) else None
    if not isinstance(raw, list):
        if data is not None:
            loaded.errors.append('the file needs a top-level "groups" list')
        return loaded
    seen: dict[str, str] = {}
    titles: set[str] = set()
    for index, item in enumerate(raw, start=1):
        title = str(item.get('title') or '').strip() if isinstance(item, dict) else ''
        options = item.get('options') if isinstance(item, dict) else None
        if not title or not isinstance(options, list) or not options:
            loaded.errors.append(f'group {index}: needs a title and a list of options')
            continue
        if title in titles or title == OTHER_TITLE:
            loaded.errors.append(f'group {title}: the title is taken')
            continue
        titles.add(title)
        names = []
        for option in options:
            name = str(option).strip()
            if name in seen:
                loaded.errors.append(f'option {name}: in {seen[name]} and in {title}; kept in {seen[name]}')
                continue
            seen[name] = title
            names.append(name)
        loaded.groups.append((title, tuple(names)))
    return loaded


def load_option_groups(button_config: str | Path | None) -> OptionGroups:
    path = option_groups_path(button_config)
    if path is None:
        return OptionGroups()
    try:
        data = yaml.safe_load(path.read_text(encoding='utf-8'))
    except (OSError, yaml.YAMLError) as error:
        return OptionGroups(path=path, errors=[f'cannot read {path.name}: {error}'])
    return parse_option_groups(data, path)
