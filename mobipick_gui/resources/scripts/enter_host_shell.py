#!/usr/bin/env python3
"""Drop privileges inside the mobipick_cmd container to match the host user."""
from __future__ import annotations

import grp
import os
import pwd
import re
import stat
import sys
from pathlib import Path


def _parse_int(value: str | None, default: int) -> int:
    if not value:
        return default
    try:
        return int(value)
    except ValueError:
        return default


def _sanitize_name(raw: str | None, *, prefix: str, fallback_id: int) -> str:
    if raw:
        candidate = re.sub(r"[^A-Za-z0-9_.-]", "-", raw.strip())
    else:
        candidate = ""
    if not candidate:
        candidate = f"{prefix}{fallback_id}"
    if candidate.lower() == "root":
        candidate = f"{prefix}{fallback_id}"
    if candidate[0].isdigit():
        candidate = f"{prefix}-{candidate}"
    return candidate[:32]


def _ensure_group(gid: int, name: str) -> str:
    try:
        group = grp.getgrgid(gid)
    except KeyError:
        pass
    else:
        return group.gr_name

    entry = f"{name}:x:{gid}:\n"
    with open("/etc/group", "a", encoding="utf-8") as handle:
        handle.write(entry)
    return name


def _ensure_user(uid: int, gid: int, name: str, home: Path) -> str:
    try:
        user = pwd.getpwuid(uid)
    except KeyError:
        user = None
    if user:
        return user.pw_name

    home.mkdir(parents=True, exist_ok=True)
    entry = f"{name}:x:{uid}:{gid}::{home}:{os.environ.get('SHELL', '/bin/bash')}\n"
    with open("/etc/passwd", "a", encoding="utf-8") as handle:
        handle.write(entry)
    return name


def _relax_permissions(path: Path) -> None:
    """Ensure the host user can traverse ``path`` if it lives under /root."""

    try:
        stats = path.stat()
    except FileNotFoundError:
        return

    if not str(path).startswith("/root"):
        return

    # If the directory already grants group/other read & execute, leave it alone.
    mode = stat.S_IMODE(stats.st_mode)
    required_bits = stat.S_IRGRP | stat.S_IXGRP | stat.S_IROTH | stat.S_IXOTH
    if mode & required_bits == required_bits:
        return

    new_mode = (stats.st_mode & ~0o777) | (mode | required_bits)
    os.chmod(path, new_mode)


def _select_home(home_hint: str | None) -> Path:
    """Return the home directory that should be used for the dropped user."""

    default_home = Path("/root")
    if not home_hint:
        return default_home

    candidate = Path(home_hint).expanduser()

    try:
        candidate_exists = candidate.exists()
    except OSError:
        candidate_exists = False

    if not candidate_exists:
        return default_home

    if candidate == default_home:
        return candidate

    candidate_rc = candidate / ".bashrc"
    default_rc = default_home / ".bashrc"

    if candidate_rc.exists():
        return candidate

    if default_rc.exists():
        return default_home

    return candidate


def main(argv: list[str]) -> "None":
    uid = _parse_int(os.environ.get("MOBIPICK_UID"), 0)
    gid = _parse_int(os.environ.get("MOBIPICK_GID"), uid)

    command = argv[1:] or ["bash"]

    if uid == 0:
        os.execvp(command[0], command)
        raise RuntimeError("execvp returned")

    requested_user = _sanitize_name(os.environ.get("MOBIPICK_HOST_USER"), prefix="host", fallback_id=uid)
    requested_group = _sanitize_name(os.environ.get("MOBIPICK_HOST_GROUP"), prefix="hostgrp", fallback_id=gid)
    home_path = _select_home(os.environ.get("MOBIPICK_HOST_HOME"))

    _ensure_group(gid, requested_group)
    user_name = _ensure_user(uid, gid, requested_user, home_path)

    cwd = Path.cwd().resolve()
    for candidate in (cwd, *cwd.parents):
        _relax_permissions(candidate)
        if candidate == Path("/"):
            break

    script_dir = Path(__file__).resolve().parent
    for candidate in (script_dir, *script_dir.parents):
        _relax_permissions(candidate)
        if candidate == Path("/"):
            break

    os.setgroups([gid])
    os.setgid(gid)
    os.setuid(uid)

    os.environ["HOME"] = str(home_path)
    os.environ["USER"] = user_name
    os.environ.setdefault("LOGNAME", user_name)

    os.execvp(command[0], command)
    raise RuntimeError("execvp returned")


if __name__ == "__main__":
    main(sys.argv)
