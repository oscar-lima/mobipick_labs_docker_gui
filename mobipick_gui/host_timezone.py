"""Give containers the host's time zone.

Containers start in UTC unless told otherwise, so every local time a node
prints, speaks or writes into a file name (log stamps, "a person told me at
18:40", archived files) is off by one or two hours from the clock on the
wall.  The GUI passes the host's zone name as ``TZ``; images that ship
tzdata then show local time.  Only the name is passed, never a path, because
a path such as ``:/etc/localtime`` would resolve to the container's own UTC
file.
"""
from __future__ import annotations

import os
from pathlib import Path
from typing import Mapping

ETC_TIMEZONE = Path('/etc/timezone')
ETC_LOCALTIME = Path('/etc/localtime')
_ZONEINFO_MARKER = 'zoneinfo/'


def _zone_name(value: str) -> str:
    """Return ``value`` when it is a zone name such as ``Europe/Berlin``."""
    name = value.strip()
    if not name or name.startswith((':', '/')) or ' ' in name:
        return ''
    return name


def host_timezone_name(
    environ: Mapping[str, str] | None = None,
    timezone_file: Path = ETC_TIMEZONE,
    localtime_file: Path = ETC_LOCALTIME,
) -> str:
    """Return the host's time zone name, or ``''`` when it is unknown.

    Order: a ``TZ`` zone name in the GUI's own environment, the zone the
    ``/etc/localtime`` link points to, then ``/etc/timezone``.
    """
    environ = os.environ if environ is None else environ
    name = _zone_name(environ.get('TZ', ''))
    if name:
        return name
    try:
        target = os.readlink(localtime_file)
    except OSError:
        target = ''
    if _ZONEINFO_MARKER in target:
        name = _zone_name(target.split(_ZONEINFO_MARKER, 1)[1])
        if name:
            return name
    try:
        lines = timezone_file.read_text(encoding='utf-8').splitlines()
    except (OSError, UnicodeDecodeError):
        return ''
    return _zone_name(lines[0]) if lines else ''


def host_timezone_environment(**kwargs) -> dict[str, str]:
    """Return ``{'TZ': <host zone>}``, or ``{}`` when the zone is unknown."""
    name = host_timezone_name(**kwargs)
    return {'TZ': name} if name else {}


__all__ = ['host_timezone_environment', 'host_timezone_name']
