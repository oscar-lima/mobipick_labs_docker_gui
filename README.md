# Mobipick Labs Docker GUI

The Mobipick Labs Docker GUI is a PyQt5 desktop application that controls the
Docker-based Mobipick Labs ROS 1 simulation. It wraps Docker Compose commands,
sets the environment required by the selected Docker image and ROS workspace,
streams process output into GUI tabs, and performs cleanup when the application
closes.

This README is developer documentation for maintaining and extending the
application. User-facing button and menu documentation lives in the
[`gui_user_documentation.md`](mobipick_gui/resources/gui_user_documentation.md)
resource and is rendered in the application from **Help > Documentation**.

<img src="doc/mobipick_labs_docker_gui.png" alt="mobipick tables sim and real" width="420">

## Repository layout

```text
.
|-- gui.py                         # Legacy shim that forwards to mobipick_gui.cli
|-- mobipick_gui/
|   |-- cli.py                     # QApplication setup and CLI parsing
|   |-- main_window.py             # Main PyQt window and Docker/ROS orchestration
|   |-- process_tab.py             # QProcess plus log widget wrapper
|   |-- log_widget.py              # Buffered QTextEdit for high-volume logs
|   |-- documentation_dialog.py    # Rendered user documentation and search
|   |-- bug_report.py              # Diagnostic report builder
|   |-- setup_wizard.py            # Image setup and custom image choices
|   |-- workspace_dialog.py        # Workspace manager dialog
|   |-- workspaces.py              # Workspace registry and runtime env model
|   |-- settings_transfer.py       # Portable import/export of GUI settings
|   |-- window_layout.py           # wmctrl/xprop capture and replay helper
|   |-- config.py                  # Bundled/user config loading and defaults
|   `-- resources/
|       |-- docker-compose.yml
|       |-- custom_entrypoint.sh
|       |-- clean.bash
|       |-- gui_user_documentation.md
|       |-- config/
|       |   |-- gui_settings.yaml
|       |   |-- button_commands_labs.yaml
|       |   |-- worlds.yaml
|       |   `-- docker_cp_image_tag.yaml
|       `-- scripts/
|           |-- enter_host_shell.py
|           |-- terminal.bashrc
|           `-- ros_workspace_setup.bash
|-- tests/mobipick_gui/            # pytest regression tests
|-- pyproject.toml                 # Packaging metadata and package data
|-- MANIFEST.in                    # Source distribution manifest
`-- doc/                           # Project imagery
```

Private experiments and templates under `mobipick_gui/resources/private/` are
excluded from package data and source distribution output.

## Runtime architecture

`mobipick_gui.cli.main()` creates a `QApplication`, instantiates
`MainWindow`, and forwards unknown arguments to Qt. `MainWindow` owns the GUI
state, menu actions, process tabs, workspace/image selection, recording state,
and shutdown sequence.

Runtime commands are executed in two ways:

- Long-running tasks use `QProcess` through `ProcessTab`. Output is merged,
  sanitized for terminal escape sequences, converted from ANSI color to HTML
  when needed, and flushed into `LogTextEdit`.
- Short helper commands use `subprocess.run()` through `MainWindow._sp_run()`,
  which injects the same runtime environment and logs the command to the GUI.

Docker Compose is always invoked by the GUI with the bundled compose file and a
fixed project name. The compose file is not intended to be run directly during
normal GUI use, because the GUI also tracks process state, xhost access, tabs,
recording, and cleanup.

## Prerequisites

The application targets Linux desktops with X11.

- Python 3.8 or newer.
- PyQt5 5.15 or newer.
- Docker Engine and the Docker Compose plugin available to the current user.
- An X11 desktop session for Gazebo, RViz, RQt, and recording.
- Optional but recommended: NVIDIA Container Toolkit for GPU-accelerated
  simulation.
- Optional tools for specific features:
  - `wmctrl` and `xprop` for window layout capture/replay.
  - `graphviz` for workspace graph rendering.
  - `ffmpeg` for Auto Launch screen recording.

Common Ubuntu setup:

```bash
sudo apt update
sudo apt install docker.io docker-compose-plugin wmctrl x11-utils graphviz ffmpeg
sudo systemctl enable docker
sudo systemctl start docker
sudo usermod -aG docker "$USER"
```

Log out and back in after changing Docker group membership, or start a shell
with `newgrp docker`.

Pull at least one Mobipick image before launching the GUI, or use the setup
wizard on first launch:

```bash
docker pull ozkrelo/x_mobipick_labs:noetic-v1.1
docker pull ozkrelo/x_mobipick_labs:noetic-v1.2
```

## Development setup

Create an editable install from the checkout:

```bash
python -m pip install -e .
```

Install test-only tools when working on regression tests:

```bash
python -m pip install pytest pytest-qt
```

Launch the GUI from the checkout:

```bash
mobipick-labs-docker-gui --verbose 2
```

Equivalent debug entry points:

```bash
python -m mobipick_gui --verbose 2
python gui.py --verbose 2
```

The verbosity option accepts levels 1 through 3. Unknown CLI arguments are
passed to Qt, for example `-platform offscreen` in headless checks.

## Tests

Run the full test suite:

```bash
pytest
```

Most GUI tests set `QT_QPA_PLATFORM=offscreen` and stub Docker discovery or
process methods. Keep new tests Docker-independent unless the test explicitly
targets Docker command construction.

Current coverage includes:

- configuration loading and writable user config paths;
- workspace registry, switching, import/export, and image/workspace matching;
- setup wizard and custom image profile persistence;
- remote ROS master behavior;
- Auto Launch and recording state transitions;
- menu-only status/image controls and menu tooltips;
- bug report formatting;
- documentation dialog rendering and keyword search.

## Packaging

Build source and wheel artifacts with:

```bash
python -m build
```

Package data is declared in both `pyproject.toml` and `MANIFEST.in`. When adding
new runtime assets under `mobipick_gui/resources/`, update both files so editable
installs, wheels, and source distributions all behave the same.

The console script installed by the package is:

```text
mobipick-labs-docker-gui = mobipick_gui.cli:main
```

## Configuration model

Bundled defaults live under `mobipick_gui/resources/config/`.
`mobipick_gui.config.CONFIG_DEFAULTS` supplies hard defaults, then
`config/gui_settings.yaml` is merged over them, then the per-user
`gui_settings.yaml` is merged last.

Important environment overrides:

- `MOBIPICK_GUI_DATA_ROOT` points the package at an alternate resources root.
- `MOBIPICK_GUI_CONFIG` points to an alternate per-user GUI settings file.
- `MOBIPICK_WORKSPACE_CONFIG` points to an alternate workspace registry.
- `XDG_CONFIG_HOME` and `XDG_DATA_HOME` control the default per-user roots.

Default per-user state locations:

```text
~/.config/mobipick-labs-docker-gui/gui_settings.yaml
~/.config/mobipick-labs-docker-gui/workspaces.yaml
~/.config/mobipick-labs-docker-gui/window_layout.yaml
~/.config/mobipick-labs-docker-gui/docker_cp_image_tag.yaml
~/.config/mobipick-labs-docker-gui/launch_sequences/
~/.config/mobipick-labs-docker-gui/profiles/
~/.local/share/mobipick-labs-docker-gui/recordings/
~/.local/share/mobipick-labs-docker-gui/image_builds/
```

Keep bundled resource files immutable at runtime. User edits should be written
to per-user config/data paths.

## Docker and ROS services

The bundled compose file defines three services:

- `mobipick` runs the simulator.
- `mobipick_cmd` runs local ROS tools, scripts, terminals, and custom commands.
- `mobipick_remote_cmd` runs tools with host networking for external ROS
  master mode.

The GUI injects these important values into Docker commands:

- `MOBIPICK_IMAGE` selected from the image combo box.
- `MOBIPICK_WORLD` selected from `worlds.yaml`.
- `MOBIPICK_CONTAINER_USER`, `MOBIPICK_CONTAINER_ENTRYPOINT`, and
  `MOBIPICK_CONTAINER_WORKDIR` derived from image profiles.
- `MOBIPICK_UID`, `MOBIPICK_GID`, `MOBIPICK_HOST_USER`,
  `MOBIPICK_HOST_GROUP`, and `MOBIPICK_HOST_HOME` derived from the host user.
- Workspace mount and ROS environment values from `WorkspaceRegistry`.
- `ROS_MASTER_URI` from local Roscore or remote ROS master mode.

The GUI creates or reuses the external Docker network named `mobipick` and
labels one-off containers with `mobipick.exec` and `mobipick.tab` so they can be
found and stopped reliably.

## Workspace model

`WorkspaceRegistry` stores host catkin workspaces, their inheritance, optional
workspace-specific Docker images, button profiles, auto-launch profiles, and
simulator command overrides.

For host workspace mode, the registry mounts the common workspace root once
inside Docker at the canonical container root `~/ros_ws`. It maps selected
workspaces and underlays into that root, exports `MOBIPICK_WORKSPACE_*`
variables, and provides fallback source paths when a workspace is not built.

`WorkspaceManagerDialog` is the UI for:

- choosing or creating a master folder;
- discovering child workspaces with `src/`;
- adding or creating standalone workspaces;
- editing inheritance and image/button/launch profiles;
- building the active workspace inside Docker;
- exporting/importing portable GUI settings;
- rendering the workspace graph.

Builds use `catkin build` inside the selected development image. Public root
images are configured as image-default only and do not mount host workspaces.

## Image profiles and setup wizard

Image behavior is controlled by `images` in `gui_settings.yaml`.

- `default` is the preferred Docker image.
- `discovery_filters` controls which local images appear in the combo box.
- `profiles` maps image refs or glob patterns to container user behavior,
  workspace support, compatible workspaces, working directory, entrypoint, and
  tooltip description.

The setup wizard can pull public images, choose a default image, and build a
host-user development image. The custom image builder writes a Docker build
context under the per-user data directory, copies `custom_entrypoint.sh`, adds a
host-matching user, installs passwordless sudo, and tags the result according
to the wizard fields.

## Button profiles

Default toolbar buttons are loaded from
`resources/config/button_commands_labs.yaml`. `load_button_layout()` supports
workspace-specific replacements through the workspace registry.

Button entries can be:

- `kind: builtin` with actions such as `sim`, `tables_demo`, `rviz`, and
  `rqt_tables`;
- `kind: command` with an arbitrary command executed either in Docker or on the
  host.

Command entries can declare:

- `requires_roscore`;
- `reuse_tab`;
- `world_config_required` and `world_arg_name`;
- `setup` or `pre_command`;
- `host`;
- `stop_command`;
- `log_command`;
- `pass_ros_master_uri`.

The GUI normalizes all entries and creates matching process tabs and
start/stop visual state.

## Auto Launch

`load_launch_sequence_plan()` resolves Auto Launch YAML from a workspace
profile, per-user launch sequence directory, or fallback filenames derived from
the active button profile.

The saved format stores:

- `timeline`: button key plus `at_seconds`;
- `shutdown.order`: reverse or custom stop order;
- optional `shutdown.skip`;
- optional button text/tooltip metadata;
- `recording.start_delay_seconds`.

`AutoLaunchWizard` edits the timeline and saves it to a writable per-user path
when the source is a packaged resource. Auto Launch can also coordinate window
layout replay and delayed recording startup.

## Recording and window layout

Recording captures X11 screen video with `ffmpeg -f x11grab`. It is armed by the
GUI checkbox and starts only after Auto Launch begins and the timeline/layout
delay has elapsed. Recording sessions create timestamped folders containing the
MP4, `ffmpeg.log`, and saved HTML logs when requested.

Window layout capture uses `WindowLayoutManager` plus `wmctrl` and `xprop`.
The manager records the baseline windows present when the GUI starts, excludes
the GUI/helper windows during capture, and applies saved positions to matching
new windows after the configured delay.

## Remote ROS master mode

Remote mode is controlled by the hidden Remote ROS Master view controls. When
enabled:

- local Roscore and simulation actions are disabled;
- tools, scripts, terminals, configured commands, and custom commands use
  `mobipick_remote_cmd`;
- `ROS_MASTER_URI` is normalized and passed into containers;
- host networking is used so ROS 1 callbacks can reach the nodes.

Changing remote mode or the URI is blocked while workspace processes are
running.

## Docker cp profiles

`docker_cp_image_tag.yaml` defines optional copy rules keyed by `default`,
`*`, `all`, exact image ref, repository, or tag.

- `host_to_container` entries run automatically after eligible containers
  appear.
- `container_to_host` entries run from **Tools > Docker > Execute Docker cp**
  for the current running tab.

User edits are saved to
`~/.config/mobipick-labs-docker-gui/docker_cp_image_tag.yaml` and override
bundled entries, including with empty profiles.

## Logging and reports

Every process tab uses `LogTextEdit`, which buffers updates to keep high-volume
process output responsive. The log widget keeps only the configured maximum
block count.

GUI-originated messages and executed commands are written to the **Log** tab.
Users can save the current tab, save all tabs, or load a saved HTML log into a
closable tab. The bottom search row searches only the current log tab.

The bug report dialog collects selected diagnostic sections, including GUI
version, selected workspace, selected image/workspace match, optional command
outputs, workspace graph, log tab text, and user notes. Keep new diagnostics
optional so report generation remains useful on machines without every tool
installed.

## User documentation dialog

The Help documentation window renders
`resources/gui_user_documentation.md` with `QTextBrowser`. It supports keyword
search from a line edit plus Find and Previous buttons. Matching keywords are
highlighted and the current match is selected and scrolled into view.

When editing user documentation, keep the text task-oriented and avoid
developer internals. Developer details belong in this README or code comments.

## Shutdown behavior

Closing the GUI starts a controlled shutdown:

1. cancel Auto Launch timers and pending recording start;
2. stop screen recording if active;
3. stop the external terminal container;
4. kill GUI-owned background `QProcess` instances;
5. stop simulator and related Mobipick containers;
6. run `clean.bash` when available;
7. revoke temporary X11 access;
8. quit the Qt application.

Avoid adding early returns in shutdown paths unless they still leave the GUI in
a recoverable state.

## Development guidelines

- Keep user-visible defaults in `resources/config/` and hard fallbacks in
  `config.py`.
- Keep per-user writes out of packaged resources.
- Prefer extending existing helper methods in `MainWindow` before adding a new
  orchestration path.
- Add tests under `tests/mobipick_gui/` for new behavior.
- Stub Docker and external tools in tests unless the test only validates command
  construction.
- For new package resources, update `pyproject.toml` and `MANIFEST.in`.
- For new user-visible controls, update
  `resources/gui_user_documentation.md`.
- For new developer-facing configuration, update this README.

## Troubleshooting for developers

- If no images appear, inspect `images.discovery_filters` and run
  `docker images`.
- If GUI tests create real dialogs unexpectedly, set
  `QT_QPA_PLATFORM=offscreen` and monkeypatch Docker discovery.
- If a workspace does not mount, check the selected image profile for
  `supports_host_workspaces`.
- If RViz or Gazebo windows do not open, verify X11, `DISPLAY`, Docker
  permissions, and temporary `xhost` access.
- If recordings produce no MP4, inspect the session `ffmpeg.log` and the
  configured display/resolution.
- If window layout replay does nothing, install `wmctrl` and `xprop` and save a
  layout after simulator windows are visible.
