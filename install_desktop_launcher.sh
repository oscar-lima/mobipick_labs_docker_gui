#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
exec "${MOBIPICK_GUI_PYTHON:-python3}" \
    "$script_dir/gui.py" --install-desktop-launcher
