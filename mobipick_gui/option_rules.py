"""Declarative rules that make toolbar dropdown options invalid.

A workspace can keep a rules file beside its button profile, named after the
profile (``<profile stem>_rules.yaml``), or ``option_rules.yaml`` in the same
directory. Each rule has a ``when`` condition over the GUI state and lists the
options it forbids (``invalid``) or the only options it allows (``only``)::

    rules:
    - when:
        remote_master: true
      only:
        world: [cic_tables]
      reason: The real robot only runs the cic_tables environment.

State names are ``remote_master`` (true when the GUI uses a remote ROS
master), ``world`` (the world_config dropdown), and the name of every
generic toolbar argument such as ``model_profile``. A condition value may be
a list, which matches any of its entries; every condition of a rule must
match. Rules only name dropdowns and states, so the GUI stays free of
workspace-specific logic.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path

import yaml

RULES_FILE_SUFFIX = '_rules.yaml'
SHARED_RULES_FILE = 'option_rules.yaml'


@dataclass(frozen=True)
class OptionRule:
    """One condition and the options it forbids or exclusively allows."""

    when: dict[str, tuple[str, ...]]
    invalid: dict[str, tuple[str, ...]] = field(default_factory=dict)
    only: dict[str, tuple[str, ...]] = field(default_factory=dict)
    reason: str = ''

    def matches(self, state: dict[str, str]) -> bool:
        return all(
            name in state and state[name] in values
            for name, values in self.when.items()
        )

    def describe(self) -> str:
        if self.reason:
            return self.reason
        condition = ', '.join(
            f'{name}={"|".join(values)}' for name, values in self.when.items()
        )
        return f'not allowed when {condition}' if condition else 'not allowed'


@dataclass
class OptionRules:
    """Rules loaded from one file, plus problems found while loading."""

    rules: list[OptionRule] = field(default_factory=list)
    path: Path | None = None
    errors: list[str] = field(default_factory=list)

    def invalid_options(
        self,
        state: dict,
        choices: dict[str, list[str]],
    ) -> dict[str, dict[str, str]]:
        """Return ``{choice: {option: reason}}`` for the current state."""
        normalized = {name: _text(value) for name, value in state.items()}
        result: dict[str, dict[str, str]] = {}
        for rule in self.rules:
            if not rule.matches(normalized):
                continue
            reason = rule.describe()
            for choice, options in choices.items():
                forbidden = set(rule.invalid.get(choice, ()))
                allowed = rule.only.get(choice)
                for option in options:
                    if option in forbidden or (
                        allowed is not None and option not in allowed
                    ):
                        result.setdefault(choice, {}).setdefault(
                            option, reason
                        )
        return result


def _text(value) -> str:
    if isinstance(value, bool):
        return 'true' if value else 'false'
    return str(value).strip()


def _values(raw) -> tuple[str, ...]:
    items = raw if isinstance(raw, (list, tuple)) else [raw]
    return tuple(_text(item) for item in items if item is not None)


def _choice_map(raw, label: str, errors: list[str], index: int) -> dict:
    if raw is None:
        return {}
    if not isinstance(raw, dict):
        errors.append(f'rule {index}: {label} must be a mapping')
        return {}
    return {str(name).strip(): _values(value) for name, value in raw.items()}


def parse_option_rules(data, path: Path | None = None) -> OptionRules:
    """Build rules from parsed YAML, collecting problems instead of raising."""
    loaded = OptionRules(path=path)
    if data is None:
        return loaded
    raw_rules = data.get('rules') if isinstance(data, dict) else None
    if not isinstance(raw_rules, list):
        loaded.errors.append('the file needs a top-level "rules" list')
        return loaded
    for index, item in enumerate(raw_rules, start=1):
        if not isinstance(item, dict):
            loaded.errors.append(f'rule {index}: must be a mapping')
            continue
        errors_before = len(loaded.errors)
        when = _choice_map(item.get('when'), 'when', loaded.errors, index)
        invalid = _choice_map(
            item.get('invalid'), 'invalid', loaded.errors, index
        )
        only = _choice_map(item.get('only'), 'only', loaded.errors, index)
        if len(loaded.errors) > errors_before:
            continue
        if not invalid and not only:
            loaded.errors.append(
                f'rule {index}: needs "invalid" or "only" options'
            )
            continue
        loaded.rules.append(
            OptionRule(
                when=when,
                invalid=invalid,
                only=only,
                reason=str(item.get('reason') or '').strip(),
            )
        )
    return loaded


def option_rules_path(button_config: str | Path | None) -> Path | None:
    """Return the rules file that belongs to a button profile, if any."""
    if not button_config:
        return None
    profile = Path(button_config).expanduser()
    for candidate in (
        profile.with_name(f'{profile.stem}{RULES_FILE_SUFFIX}'),
        profile.with_name(SHARED_RULES_FILE),
    ):
        if candidate.is_file():
            return candidate
    return None


def load_option_rules(button_config: str | Path | None) -> OptionRules:
    """Load the rules file next to ``button_config``; empty when absent."""
    path = option_rules_path(button_config)
    if path is None:
        return OptionRules()
    try:
        with path.open('r', encoding='utf-8') as handle:
            data = yaml.safe_load(handle)
    except (OSError, yaml.YAMLError) as exc:
        return OptionRules(path=path, errors=[str(exc)])
    return parse_option_rules(data, path)
