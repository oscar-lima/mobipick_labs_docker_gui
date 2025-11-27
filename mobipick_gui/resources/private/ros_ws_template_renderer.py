#!/usr/bin/env python3
# Purpose: source ./select_ros1_ws.sh, capture the path segment between $HOME and src,
# then render every file in ./jinja_templates with Jinja using:
#   ros1_workspace -> the captured segment (e.g., "ros_ws/mobipick_labs_ws")
#   command        -> a list of strings chosen from a config file based on the workspace
# Output files are written one directory up from the current working directory
# while preserving relative names and file modes.

import os
import stat
import subprocess
from pathlib import Path
from typing import Dict, List, Any

try:
    from jinja2 import Environment, FileSystemLoader, StrictUndefined
except ModuleNotFoundError as e:
    raise SystemExit("Missing dependency: jinja2. Install with: pip install jinja2") from e

try:
    import yaml  # PyYAML
except ModuleNotFoundError as e:
    raise SystemExit("Missing dependency: PyYAML. Install with: pip install pyyaml") from e


class RosWorkspaceFetcher:
    """Source a local bash script that sets ROS_WORKSPACE and return the segment between $HOME and src."""
    def __init__(self, script_path: Path):
        self.script_path = script_path

    def _capture_ros_workspace(self) -> str:
        if not self.script_path.exists():
            raise FileNotFoundError(f"Bash script not found: {self.script_path}")
        if not self.script_path.is_file():
            raise IsADirectoryError(f"Path is not a file: {self.script_path}")

        cmd = f'set -a; source "{self.script_path}"; printf "%s" "$ROS_WORKSPACE"'
        result = subprocess.run(
            ["/bin/bash", "-c", cmd],
            capture_output=True,
            text=True,
            check=True,
        )
        value = result.stdout.strip()
        if not value:
            raise RuntimeError("ROS_WORKSPACE is empty or not set by the script")
        return value

    @staticmethod
    def _segment_between_home_and_src(ros_workspace: str) -> str:
        expanded = os.path.expanduser(os.path.expandvars(ros_workspace))
        p = Path(expanded).resolve()
        home = Path(os.path.expanduser("~")).resolve()

        try:
            rel = p.relative_to(home)
        except ValueError:
            parts = p.parts
            if "src" in parts:
                idx = parts.index("src")
                seg_parts = parts[:idx]
                if not seg_parts:
                    raise RuntimeError("Cannot compute workspace segment")
                # remove leading slash
                return Path(*seg_parts).as_posix().lstrip("/")
            return p.name

        parts = rel.parts
        if "src" in parts:
            idx = parts.index("src")
            seg_parts = parts[:idx]
        else:
            seg_parts = parts

        if not seg_parts:
            raise RuntimeError("Workspace segment between $HOME and src is empty")

        return Path(*seg_parts).as_posix()

    def fetch_workspace_segment(self) -> str:
        ros_ws = self._capture_ros_workspace()
        return self._segment_between_home_and_src(ros_ws)


class CommandSelector:
    """Load a YAML config and choose the command list based on the workspace name."""
    def __init__(self, config_path: Path):
        self.config_path = config_path
        if not self.config_path.exists():
            raise FileNotFoundError(f"Config file not found: {self.config_path}")
        if not self.config_path.is_file():
            raise IsADirectoryError(f"Path is not a file: {self.config_path}")
        self.config = self._load_yaml()

    def _load_yaml(self) -> Dict[str, Any]:
        with self.config_path.open("r", encoding="utf-8") as f:
            data = yaml.safe_load(f) or {}
        if not isinstance(data, dict):
            raise ValueError("Config root must be a mapping")
        return data

    def select(self, workspace_segment: str) -> List[str]:
        """
        workspace_segment example values:
          - "catkin_ws"
          - "ros_ws/mobipick_labs_ws"
          - "ros_ws/rae_upom_mobipick_ws"
        Selection uses the last path part by default, with fallbacks:
          commands.<last_part> -> commands.<workspace_segment> -> commands.default
        """
        commands_map = self.config.get("commands", {})
        if not isinstance(commands_map, dict):
            raise ValueError("Config must contain a mapping 'commands'")

        last_part = Path(workspace_segment).name
        candidates = [
            commands_map.get(last_part),
            commands_map.get(workspace_segment),
            commands_map.get("default"),
        ]
        for cmd in candidates:
            if cmd is not None:
                if not isinstance(cmd, list) or not all(isinstance(x, str) for x in cmd):
                    raise ValueError("Selected command must be a list of strings")
                return cmd

        raise KeyError(
            "No command found for workspace and no default provided. "
            "Add either commands.default or a specific key in the config."
        )


class TemplateRenderer:
    """Render all files under templates_dir with Jinja and write them under output_base preserving names and modes."""
    def __init__(self, templates_dir: Path, output_base: Path):
        self.templates_dir = templates_dir
        self.output_base = output_base
        if not self.templates_dir.is_dir():
            raise NotADirectoryError(f"Templates directory not found: {self.templates_dir}")

        self.env = Environment(
            loader=FileSystemLoader(str(self.templates_dir)),
            undefined=StrictUndefined,
            autoescape=False,
            keep_trailing_newline=True,
            lstrip_blocks=False,
            trim_blocks=False,
        )

    def render_all(self, context: dict) -> None:
        for src in self.templates_dir.rglob("*"):
            if not src.is_file():
                continue
            rel = src.relative_to(self.templates_dir)
            dst = self.output_base / rel
            dst.parent.mkdir(parents=True, exist_ok=True)

            template = self.env.get_template(str(rel))
            rendered = template.render(**context)
            dst.write_text(rendered, encoding="utf-8")

            # Preserve executable bit from source
            src_mode = src.stat().st_mode
            if src_mode & stat.S_IXUSR:
                dst.chmod(dst.stat().st_mode | stat.S_IXUSR | stat.S_IXGRP | stat.S_IXOTH)


def main() -> None:
    here = Path(__file__).resolve().parent
    script_path = here / "select_ros1_ws.sh"     # bash script next to this python file
    templates_dir = here / "jinja_templates"     # templates directory next to this python file
    config_path = here / "config.yml"            # config file next to this python file
    output_base = Path.cwd().parent              # write one directory up from where the script is run

    # Capture workspace segment and derive selection key
    workspace_segment = RosWorkspaceFetcher(script_path).fetch_workspace_segment()

    # Choose command from config
    command_list = CommandSelector(config_path).select(workspace_segment)

    # Render templates with both variables
    context = {
        "ros1_workspace": workspace_segment,
        "command": command_list,              # usable with: command: {{ command }}
        "command_str": " ".join(command_list) # optional convenience if templates want a single string
    }
    TemplateRenderer(templates_dir, output_base).render_all(context)


if __name__ == "__main__":
    main()
