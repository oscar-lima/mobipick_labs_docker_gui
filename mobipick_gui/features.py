"""Features and presets of a button profile: named sets of launch argument values.

A workspace can keep a features file beside its button profile, named after the profile
(``<profile stem>_features.yaml``). It groups the profile's generic arguments into features, each a tick box that
sets its ``on`` or ``off`` values, and into mode groups, radio choices of which exactly one mode is on::

    version: 1
    groups:
    - name: Cable handling
      key: cable_handling
      kind: modes
      description: one switch for the pick, place and insert
      modes:
      - {key: hack, label: Untangle poses, default: true, class: safe, sets: {cable_detour_mode: hack}}
      - {key: cable_aware, label: Cable-aware, sim_default: true, class: careful, sets: {cable_detour_mode: cable_aware}}
    - name: Open-set picks
      key: open_set_picks
      kind: features
      features:
      - key: jaw_along_the_box
        label: Jaw along the box
        class: safe              # careful features stay off on a remote ROS master unless ticked there
        description: turns a diagonal grasp onto the box faces
        parent: ''               # key of a feature this one needs on
        on: {open_set_align_jaw_to_box: 'true'}
        off: {open_set_align_jaw_to_box: 'false'}

``default`` marks the mode a group falls back to on the real robot; ``sim_default`` (optional, at most one mode) marks
the sim's default where it differs, e.g. a careful mode the sim always tests.

A preset names one mode per group and the features that are on; every other feature is off, and ``options`` sets
plain arguments that are no feature. Presets live in ``<profile stem>_presets/<name>.yaml`` beside the profile and,
saved from the GUI, in the per-user presets directory (a user preset of the same name wins)::

    name: sim-full
    description: the simulation with every feature on
    target: sim                  # sim, or real for a remote ROS master
    modes: {cable_handling: cable_aware}
    features_on: [jaw_along_the_box]
    options: {cable_monitor: 'true'}

A preset never switches a careful feature or mode on on the real robot: there it stays off until the user ticks it in
the Features dialog (e-stop in hand). A ``careful_on_real`` list in an older file is ignored and reported.

Options a feature names but the profile lacks are reported as missing and left alone, so a features file can
list options that are still being added.
"""

from __future__ import annotations

import re
from dataclasses import dataclass, field
from pathlib import Path

import yaml

FEATURES_FILE_SUFFIX = '_features.yaml'
PRESETS_DIR_SUFFIX = '_presets'
CAREFUL = 'careful'
TARGETS = ('sim', 'real')


@dataclass(frozen=True)
class Feature:
    key: str
    label: str
    group: str
    cls: str = 'safe'
    description: str = ''
    parent: str = ''
    flags: tuple[str, ...] = ()
    issues: tuple[str, ...] = ()
    on: dict[str, str] = field(default_factory=dict)
    off: dict[str, str] = field(default_factory=dict)

    @property
    def careful(self) -> bool:
        return self.cls == CAREFUL


@dataclass(frozen=True)
class Mode:
    key: str
    label: str
    group: str
    default: bool = False
    sim_default: bool = False      # the sim's default where it differs from ``default`` (the real robot's)
    cls: str = 'safe'
    description: str = ''
    flags: tuple[str, ...] = ()
    issues: tuple[str, ...] = ()
    sets: dict[str, str] = field(default_factory=dict)
    pending: dict[str, str] = field(default_factory=dict)   # what the mode will set once ``pending_on`` exists
    pending_on: str = ''

    @property
    def careful(self) -> bool:
        return self.cls == CAREFUL


@dataclass(frozen=True)
class Group:
    key: str
    name: str
    kind: str                      # 'features' or 'modes'
    description: str = ''
    members: tuple[str, ...] = ()  # feature or mode keys, in order


@dataclass
class Features:
    """The features file of a profile, plus problems found while loading."""

    groups: list[Group] = field(default_factory=list)
    features: dict[str, Feature] = field(default_factory=dict)
    modes: dict[str, dict[str, Mode]] = field(default_factory=dict)   # group key -> mode key -> mode
    path: Path | None = None
    errors: list[str] = field(default_factory=list)

    def __bool__(self) -> bool:
        return bool(self.groups)

    def options(self) -> set[str]:
        """Every argument name a feature or mode sets."""
        names = {name for feature in self.features.values() for name in feature.on}
        names.update(name for modes in self.modes.values() for mode in modes.values() for name in mode.sets)
        return names

    def default_mode(self, group: str, sim: bool = False) -> str:
        modes = self.modes[group]
        if sim:
            key = next((key for key, mode in modes.items() if mode.sim_default), '')
            if key:
                return key
        return next((key for key, mode in modes.items() if mode.default), next(iter(modes)))


@dataclass
class Preset:
    name: str
    description: str = ''
    target: str = 'sim'
    modes: dict[str, str] = field(default_factory=dict)
    features_on: tuple[str, ...] = ()
    options: dict[str, str] = field(default_factory=dict)
    path: Path | None = None
    user: bool = False

    def to_yaml(self) -> dict:
        data = {'name': self.name, 'description': self.description, 'target': self.target,
                'modes': dict(self.modes), 'features_on': list(self.features_on)}
        if self.options:
            data['options'] = dict(self.options)
        return data


@dataclass
class Selection:
    """Which mode of each group and which features are on."""

    modes: dict[str, str] = field(default_factory=dict)
    features_on: set[str] = field(default_factory=set)


def _text(value) -> str:
    if isinstance(value, bool):
        return 'true' if value else 'false'
    return '' if value is None else str(value).strip()


def _strings(raw) -> dict[str, str]:
    return {str(name).strip(): _text(value) for name, value in (raw or {}).items()} if isinstance(raw, dict) else {}


def features_path(button_config: str | Path | None) -> Path | None:
    if not button_config:
        return None
    profile = Path(button_config).expanduser()
    candidate = profile.with_name(f'{profile.stem}{FEATURES_FILE_SUFFIX}')
    return candidate if candidate.is_file() else None


def presets_dir(button_config: str | Path | None) -> Path | None:
    if not button_config:
        return None
    profile = Path(button_config).expanduser()
    return profile.with_name(f'{profile.stem}{PRESETS_DIR_SUFFIX}')


def parse_features(data, path: Path | None = None) -> Features:
    """Build the features from parsed YAML, collecting problems instead of raising."""
    loaded = Features(path=path)
    raw_groups = data.get('groups') if isinstance(data, dict) else None
    if not isinstance(raw_groups, list):
        if data is not None:
            loaded.errors.append('the file needs a top-level "groups" list')
        return loaded
    for index, raw in enumerate(raw_groups, start=1):
        if not isinstance(raw, dict) or not raw.get('name'):
            loaded.errors.append(f'group {index}: needs a name')
            continue
        name = str(raw['name']).strip()
        key = str(raw.get('key') or name).strip()
        kind = str(raw.get('kind') or 'features').strip()
        if kind == 'modes':
            modes: dict[str, Mode] = {}
            for item in raw.get('modes') or []:
                if not isinstance(item, dict) or not item.get('key'):
                    loaded.errors.append(f'group {name}: a mode needs a key')
                    continue
                mode = Mode(key=str(item['key']), label=str(item.get('label') or item['key']), group=key,
                            default=bool(item.get('default')), sim_default=bool(item.get('sim_default')),
                            cls=_text(item.get('class') or 'safe'),
                            description=_text(item.get('description')), flags=tuple(item.get('flags') or ()),
                            issues=tuple(item.get('issues') or ()), sets=_strings(item.get('sets')),
                            pending=_strings(item.get('pending')), pending_on=_text(item.get('pending_on')))
                modes[mode.key] = mode
            if not modes:
                loaded.errors.append(f'group {name}: no modes')
                continue
            if sum(mode.default for mode in modes.values()) != 1:
                loaded.errors.append(f'group {name}: exactly one mode needs "default: true"')
            if sum(mode.sim_default for mode in modes.values()) > 1:
                loaded.errors.append(f'group {name}: at most one mode may have "sim_default: true"')
            loaded.modes[key] = modes
            loaded.groups.append(Group(key, name, kind, _text(raw.get('description')), tuple(modes)))
            continue
        members = []
        for item in raw.get('features') or []:
            if not isinstance(item, dict) or not item.get('key'):
                loaded.errors.append(f'group {name}: a feature needs a key')
                continue
            on, off = _strings(item.get('on')), _strings(item.get('off'))
            if not on or set(on) != set(off):
                loaded.errors.append(f'feature {item["key"]}: "on" and "off" must set the same options')
                continue
            feature = Feature(key=str(item['key']), label=str(item.get('label') or item['key']), group=key,
                              cls=_text(item.get('class') or 'safe'), description=_text(item.get('description')),
                              parent=_text(item.get('parent')), flags=tuple(item.get('flags') or ()),
                              issues=tuple(item.get('issues') or ()), on=on, off=off)
            if feature.key in loaded.features:
                loaded.errors.append(f'feature {feature.key}: defined twice')
                continue
            loaded.features[feature.key] = feature
            members.append(feature.key)
        loaded.groups.append(Group(key, name, kind, _text(raw.get('description')), tuple(members)))
    for feature in loaded.features.values():
        if feature.parent and feature.parent not in loaded.features:
            loaded.errors.append(f'feature {feature.key}: unknown parent {feature.parent}')
    return loaded


def load_features(button_config: str | Path | None) -> Features:
    path = features_path(button_config)
    if path is None:
        return Features()
    try:
        data = yaml.safe_load(path.read_text(encoding='utf-8'))
    except (OSError, yaml.YAMLError) as exc:
        return Features(path=path, errors=[f'{path}: {exc}'])
    loaded = parse_features(data, path)
    loaded.errors = [f'{path}: {error}' for error in loaded.errors]
    return loaded


def parse_preset(data, path: Path | None = None, user: bool = False) -> Preset:
    if not isinstance(data, dict):
        raise ValueError('a preset must be a mapping')
    name = _text(data.get('name')) or (path.stem if path else '')
    if not name:
        raise ValueError('a preset needs a name')
    target = _text(data.get('target') or 'sim')
    if target not in TARGETS:
        raise ValueError(f'preset {name}: target must be sim or real, not {target!r}')
    return Preset(name=name, description=_text(data.get('description')), target=target,
                  modes=_strings(data.get('modes')), features_on=tuple(_text(k) for k in data.get('features_on') or ()),
                  options=_strings(data.get('options')), path=path, user=user)


def load_presets(button_config: str | Path | None, user_dir: Path | None) -> tuple[dict[str, Preset], list[str]]:
    """Presets beside the profile and the user's own, by name (a user preset wins), and problems found."""
    presets: dict[str, Preset] = {}
    errors: list[str] = []
    for folder, user in ((presets_dir(button_config), False), (user_dir, True)):
        if folder is None or not folder.is_dir():
            continue
        for path in sorted(folder.glob('*.yaml')):
            try:
                data = yaml.safe_load(path.read_text(encoding='utf-8'))
                preset = parse_preset(data, path, user)
            except (OSError, yaml.YAMLError, ValueError) as exc:
                errors.append(f'{path}: {exc}')
                continue
            presets[preset.name] = preset
            if 'careful_on_real' in data:
                errors.append(f'{path}: careful_on_real is ignored: a careful feature goes on on the real robot '
                              'only by a tick in the Features dialog there')
    return presets, errors


def preset_file_name(name: str) -> str:
    stem = re.sub(r'[^A-Za-z0-9._-]+', '-', name.strip()).strip('-.')
    if not stem:
        raise ValueError('a preset name needs a letter or digit')
    return f'{stem}.yaml'


def save_preset(preset: Preset, user_dir: Path) -> Path:
    user_dir.mkdir(parents=True, exist_ok=True)
    path = user_dir / preset_file_name(preset.name)
    tmp = path.with_suffix('.yaml.tmp')
    tmp.write_text(yaml.safe_dump(preset.to_yaml(), sort_keys=False, allow_unicode=True), encoding='utf-8')
    tmp.replace(path)
    preset.path, preset.user = path, True
    return path


def preset_problems(features: Features, preset: Preset) -> list[str]:
    """Names in a preset that the features file does not know."""
    problems = [f'unknown feature {key}' for key in preset.features_on if key not in features.features]
    for group, mode in preset.modes.items():
        if group not in features.modes:
            problems.append(f'unknown mode group {group}')
        elif mode not in features.modes[group]:
            problems.append(f'unknown mode {group}: {mode}')
    return problems


def selection_of(features: Features, preset: Preset) -> Selection:
    sim = preset.target != 'real'
    modes = {group: preset.modes.get(group) or features.default_mode(group, sim) for group in features.modes}
    modes = {group: mode if mode in features.modes[group] else features.default_mode(group, sim)
             for group, mode in modes.items()}
    return Selection(modes=modes, features_on={key for key in preset.features_on if key in features.features})


def selection_values(features: Features, selection: Selection) -> dict[str, str]:
    """The argument values that put the selection in place: every mode group and every feature, on or off. A
    feature whose parent is off is off too."""
    values: dict[str, str] = {}
    for group, mode in selection.modes.items():
        values.update(features.modes[group][mode].sets)
    for key, feature in features.features.items():
        on = key in selection.features_on and (not feature.parent or feature.parent in selection.features_on)
        values.update(feature.on if on else feature.off)
    return values


def preset_values(features: Features, preset: Preset) -> dict[str, str]:
    values = selection_values(features, selection_of(features, preset))
    values.update(preset.options)
    return values


def current_selection(features: Features, values: dict[str, str],
                      sim: bool = False) -> tuple[Selection, dict[str, str]]:
    """The selection the current argument values show, and per feature or mode group 'mixed' where they match
    neither its on nor its off values. Options missing from ``values`` (not in the profile) are ignored; a mode group
    they do not decide shows its default (the sim's with ``sim``)."""
    selection = Selection()
    mixed: dict[str, str] = {}

    def matches(wanted: dict[str, str]) -> bool | None:
        present = {name: value for name, value in wanted.items() if name in values}
        if not present:
            return None
        return all(values[name] == value for name, value in present.items())

    for group, modes in features.modes.items():
        hit = [key for key, mode in modes.items() if matches(mode.sets)]
        if len(hit) == 1:
            selection.modes[group] = hit[0]
        else:
            selection.modes[group] = features.default_mode(group, sim)
            if any(matches(mode.sets) is not None for mode in modes.values()):
                mixed[group] = 'the options match no single mode'
    for key, feature in features.features.items():
        on, off = matches(feature.on), matches(feature.off)
        if on:
            selection.features_on.add(key)
        elif on is not None and not off:
            mixed[key] = 'some of its options are on, some off'
    return selection, mixed


def missing_options(features: Features, profile_options: set[str]) -> dict[str, list[str]]:
    """feature or mode key -> the options it sets that the profile does not have"""
    result: dict[str, list[str]] = {}
    for key, feature in features.features.items():
        missing = [name for name in feature.on if name not in profile_options]
        if missing:
            result[key] = missing
    for modes in features.modes.values():
        for key, mode in modes.items():
            missing = [name for name in mode.sets if name not in profile_options]
            if missing:
                result[key] = missing
    return result


def careful_turned_on(features: Features, selection: Selection) -> list[str]:
    """Keys of careful features and careful modes the selection switches on."""
    keys = [key for key in selection.features_on if features.features[key].careful]
    keys += [mode for group, mode in selection.modes.items() if features.modes[group][mode].careful]
    return sorted(keys)


def without_careful(features: Features, selection: Selection, allowed: set[str]) -> tuple[Selection, list[str]]:
    """The selection with every careful feature off and every careful mode back to its group's default, except the
    keys in ``allowed``, and the keys it switched off (the real robot keeps careful features off until ticked)."""
    dropped = [key for key in careful_turned_on(features, selection) if key not in allowed]
    modes = {group: (features.default_mode(group) if mode in dropped else mode)
             for group, mode in selection.modes.items()}
    return Selection(modes=modes, features_on=selection.features_on - set(dropped)), dropped


def careful_option_values(features: Features, allowed: set[str]) -> dict[tuple[str, str], str]:
    """(argument name, value) -> key of the careful feature or mode that sets it, for every careful feature and mode
    not in ``allowed``: the values plain options must not set on the real robot either."""
    owners: dict[tuple[str, str], str] = {}
    for key, feature in features.features.items():
        if feature.careful and key not in allowed:
            owners.update({(name, str(value)): key for name, value in feature.on.items()})
    for modes in features.modes.values():
        for key, mode in modes.items():
            if mode.careful and key not in allowed:
                owners.update({(name, str(value)): key for name, value in mode.sets.items()})
    return owners


def careful_off_values(features: Features, values: dict[str, str]) -> dict[str, tuple[str, str]]:
    """name -> (off value, owner key) for every current value (``values``) that a careful feature or mode sets: the
    feature's off value, for a careful mode the real robot's default mode's value. Options without an off value of
    their own are left out."""
    result: dict[str, tuple[str, str]] = {}
    for key, feature in features.features.items():
        if feature.careful:
            for name, value in feature.on.items():
                if values.get(name) == str(value) and name in feature.off:
                    result.setdefault(name, (str(feature.off[name]), key))
    for group, modes in features.modes.items():
        default = modes.get(features.default_mode(group))
        for key, mode in modes.items():
            if mode.careful and default is not None and not default.careful:
                for name, value in mode.sets.items():
                    if values.get(name) == str(value) and name in default.sets:
                        result.setdefault(name, (str(default.sets[name]), key))
    return result


def gate_careful_options(features: Features, values: dict[str, str], gated: dict[str, str],
                         allowed: set[str]) -> tuple[dict[str, str], dict[str, dict[str, str]]]:
    """The final argument values with the careful gate applied to plain options too: a value a careful feature or mode
    outside ``allowed`` sets, which the gated selection (``gated``, its values) does not give, goes back to the gated
    selection's value (or is left out); returns (values, refused name -> {value, reason})."""
    owners = careful_option_values(features, allowed)
    result: dict[str, str] = {}
    refused: dict[str, dict[str, str]] = {}
    for name, value in values.items():
        owner = owners.get((name, value))
        if owner is not None and gated.get(name) != value:
            refused[name] = {'value': value, 'reason': f'careful ({owner}): stays off on the real robot unless '
                                                       f'ticked in the Features dialog'}
            if name in gated:
                result[name] = gated[name]
            continue
        result[name] = value
    return result, refused
