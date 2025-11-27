#!/usr/bin/env python3

# First action: ensure a clean environment for selection and rendering
# by unsetting any inherited ROS_WORKSPACE
import os
os.environ.pop("ROS_WORKSPACE", None)
try:
    os.unsetenv("ROS_WORKSPACE")  # no error if already absent
except AttributeError:
    print('os.unsetenv not available! could not unset ROS_WORKSPACE')
    pass  # os.unsetenv may not exist on some platforms

import re
import sys
import subprocess
from pathlib import Path
from typing import List, Tuple

class SelectRos1Ws:
    """Parse select_ros1_ws.sh, prompt for a workspace index, update DESIRED_NUMBER, then run the renderer."""

    def __init__(self, bash_path: Path, renderer_path: Path):
        self.bash_path = bash_path
        self.renderer_path = renderer_path

    def _read_bash(self) -> str:
        if not self.bash_path.exists():
            raise FileNotFoundError(f"Not found: {self.bash_path}")
        return self.bash_path.read_text(encoding="utf-8")

    @staticmethod
    def _extract_rws_list(sh_text: str) -> List[Tuple[int, str, str]]:
        """
        Robustly extract entries between a line with 'rws_list=(' and the first line that is only ')'.
        Returns list of tuples: (index, path_expression, trailing_comment_without_hash).
        """
        lines = sh_text.splitlines()
        start_idx = None
        for i, line in enumerate(lines):
            # allow variants like "rws_list = (", possibly with trailing spaces
            if re.search(r"^\s*rws_list\s*=\s*\(\s*$", line):
                start_idx = i + 1
                break
            # also support inline form: rws_list=( item1 ...
            m_inline = re.search(r"^\s*rws_list\s*=\s*\(\s*(.*)$", line)
            if m_inline:
                start_idx = i + 1
                # if there is content on same line, treat it as the first entry line
                remainder = m_inline.group(1).rstrip()
                if remainder:
                    lines.insert(i + 1, remainder)
                break
        if start_idx is None:
            raise ValueError("Could not find 'rws_list=(' in select_ros1_ws.sh")

        # collect until a line that is just ')'
        body_lines: List[str] = []
        for j in range(start_idx, len(lines)):
            if re.match(r"^\s*\)\s*$", lines[j]):
                break
            body_lines.append(lines[j])

        if not body_lines:
            raise ValueError("rws_list appears empty")

        entries: List[Tuple[int, str, str]] = []
        idx = 0
        for raw in body_lines:
            line = raw.strip()
            if not line:
                continue
            # skip full-line comments
            if line.startswith("#"):
                continue
            # split on first '#' only to keep description
            path_expr, comment = SelectRos1Ws._split_comment(line)
            if not path_expr:
                continue
            entries.append((idx, path_expr.strip(), comment))
            idx += 1

        if not entries:
            raise ValueError("No usable entries found in rws_list")
        return entries

    @staticmethod
    def _split_comment(line: str) -> Tuple[str, str]:
        """Split line into path expression and comment string (without leading '#')."""
        parts = line.split("#", 1)
        path_expr = parts[0].strip()
        comment = parts[1].strip() if len(parts) > 1 else ""
        return path_expr, comment

    @staticmethod
    def _replace_desired_number(sh_text: str, new_idx: int) -> str:
        # Replace first occurrence of DESIRED_NUMBER=number preserving rest of the line
        pattern = r"^(?P<prefix>\s*DESIRED_NUMBER\s*=\s*)(?P<num>\d+)(?P<suffix>.*)$"
        repl = r"\g<prefix>" + str(new_idx) + r"\g<suffix>"
        new_text, n = re.subn(pattern, repl, sh_text, count=1, flags=re.MULTILINE)
        if n == 0:
            raise ValueError("Could not find DESIRED_NUMBER assignment to replace")
        return new_text

    def _prompt_choice(self, entries: List[Tuple[int, str, str]]) -> int:
        print("Select ROS1 workspace by number:")
        for idx, path_expr, comment in entries:
            label = f"    # {comment}" if comment else ""
            print(f"  {idx}: {path_expr}{label}")

        while True:
            raw = input("Enter number: ").strip()
            if not raw.isdigit():
                print("Invalid input. Enter a non negative integer.")
                continue
            choice = int(raw)
            if 0 <= choice < len(entries):
                return choice
            print(f"Out of range. Valid: 0..{len(entries) - 1}")

    def run(self) -> None:
        # always re-read the actual file to reflect any user changes
        sh_text = self._read_bash()
        entries = self._extract_rws_list(sh_text)
        choice = self._prompt_choice(entries)

        updated = self._replace_desired_number(sh_text, choice)
        self.bash_path.write_text(updated, encoding="utf-8")
        print(f"Updated {self.bash_path.name}: DESIRED_NUMBER={choice}")

        # Run the renderer with the same interpreter
        print(f"Running {self.renderer_path.name} ...")
        result = subprocess.run(
            [sys.executable, str(self.renderer_path)],
            cwd=str(self.renderer_path.parent),
        )
        if result.returncode != 0:
            raise SystemExit(result.returncode)
        print("Done.")

def main() -> None:
    here = Path(__file__).resolve().parent
    bash_path = here / "select_ros1_ws.sh"
    renderer_path = here / "ros_ws_template_renderer.py"
    SelectRos1Ws(bash_path, renderer_path).run()

if __name__ == "__main__":
    main()
