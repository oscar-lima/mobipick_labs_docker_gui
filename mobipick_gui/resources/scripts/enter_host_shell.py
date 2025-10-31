#!/usr/bin/env python3
"""Drop privileges inside the mobipick_cmd container to match the host user."""
from __future__ import annotations

import grp
import os
import pwd
import re
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


def main(argv: list[str]) -> "None":
    uid = _parse_int(os.environ.get("MOBIPICK_UID"), 0)
    gid = _parse_int(os.environ.get("MOBIPICK_GID"), uid)

    command = argv[1:] or ["bash"]

    if uid == 0:
        os.execvp(command[0], command)
        raise RuntimeError("execvp returned")

    requested_user = _sanitize_name(os.environ.get("MOBIPICK_HOST_USER"), prefix="host", fallback_id=uid)
    requested_group = _sanitize_name(os.environ.get("MOBIPICK_HOST_GROUP"), prefix="hostgrp", fallback_id=gid)
    home_hint = os.environ.get("MOBIPICK_HOST_HOME", "")
    home_path = Path(home_hint).expanduser() if home_hint else Path("/root")

    _ensure_group(gid, requested_group)
    user_name = _ensure_user(uid, gid, requested_user, home_path)

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
