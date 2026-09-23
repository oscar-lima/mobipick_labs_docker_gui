# Mobipick Labs Docker GUI

The Mobipick Labs Docker GUI is a PyQt5 desktop application that controls the
Docker-based Mobipick Labs ROS 1 simulation. It wraps Docker Compose commands,
sets the environment required by the selected Docker image and ROS workspace,
streams process output into GUI tabs, and performs cleanup when the application
closes.

Related project and distribution links:

- Mobipick Labs repository: <https://github.com/DFKI-NI/mobipick_labs>
- PyPI package: <https://pypi.org/project/mobipick-labs-docker-gui/>

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
|   |-- flow_layout.py             # Wrapping row layout used by the toolbar rows
|   |-- bug_report.py              # Diagnostic report builder
|   |-- setup_wizard.py            # Image setup and custom image choices
|   |-- workspace_dialog.py        # Workspace manager dialog
|   |-- workspaces.py              # Workspace registry and runtime env model
|   |-- settings_transfer.py       # Portable import/export of GUI settings
|   |-- window_control.py          # window backends: wmctrl/xprop (X11), GNOME Shell extension (Wayland)
|   |-- window_layout.py           # window layout capture and replay helper
|   |-- remote_control.py          # HTTP remote-control server, events, shell sessions
|   |-- remote_adapter.py          # MainWindow bridge used by the remote-control server
|   |-- remote_client.py           # mobipick-labs-docker-gui-remote CLI client
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
and shutdown sequence. Before creating the main window, startup acquires a
per-user runtime lock. If another Mobipick Labs Control GUI is already running,
the new process shows an error and exits instead of opening a second window.

At startup, the GUI refreshes its per-user desktop entry at
`~/.local/share/applications/mobipick-labs-docker-gui.desktop` (or below
`XDG_DATA_HOME`). Its desktop-file ID and `StartupWMClass` match the stable Qt
application identity, allowing X11 window managers and Wayland compositors to
associate console-launched windows with the bundled Mobipick icon.

That refresh rewrites the entry's `Exec` from the running invocation
(`desktop_launch_command`), so a pinned dock icon keeps following the way the
GUI is actually started. `python -m mobipick_gui` is the exception: it leaves
the package's `__main__.py` in `argv[0]`, and running that file as a script
fails with `attempted relative import with no known parent package`. The
launcher therefore records the `gui.py` shim next to the package, or
`python -m mobipick_gui` when no shim exists, so a debugging run never leaves
the dock icon pointing at a command that cannot start.

The same refresh installs hidden `mobipick-rviz.desktop`, `mobipick-rqt.desktop`,
and `mobipick-gazebo.desktop` entries next to it. GNOME Shell 45 and newer
ignore the `_NET_WM_ICON` that RViz, rqt, and Gazebo set on their own windows,
and containers cannot register desktop entries on the host, so without these
entries the dock and Alt-Tab switcher show a generic gear for every container
tool window. Each entry is `NoDisplay`, has an inert `Exec`, and matches
windows by identity: `StartupWMClass=rviz` and `StartupWMClass=gazebo` cover
RViz and the Gazebo client (both always X11/XWayland because of OGRE; gzclient
names its Qt application `gazebo`), while the RQt entry uses
`StartupWMClass=python3`, the `app_id` that Qt 5.12 reports for every
Python-based rqt tool on native Wayland. GNOME resolves a `StartupWMClass` hit
on either WM_CLASS part before it falls back to the desktop-file ID, so GUI
launches use the ID as a fallback identity: the RQt Tables button, custom
buttons and Custom Command entries that invoke an rqt tool
(`desktop_launcher.desktop_entry_for_command`), and the Sim button set
`RESOURCE_NAME=mobipick-rqt` in the container. Qt's XCB platform uses that as
the WM_CLASS instance, so rqt plugin windows (`rqt_tables_demo`, `rqt_graph`,
the sim's `rqt_robot_steering`, ...) resolve to the RQt entry while Gazebo and
RViz windows from the same launch keep their own icons. Windows launched from
terminal tabs keep their own WM_CLASS and therefore only match when it equals
one of the `StartupWMClass` values above.

Matching has a second precondition on X11 and XWayland: Mutter marks a window
whose `WM_CLIENT_MACHINE` differs from the compositor's hostname as remote, and
GNOME Shell never associates remote windows with desktop entries. Qt fills that
property from the container hostname, so the GUI exports
`MOBIPICK_CONTAINER_HOSTNAME=<host hostname>` to every `docker compose`
process; the `mobipick_cmd` service interpolates it as
`hostname: ${MOBIPICK_CONTAINER_HOSTNAME:-}` and the `mobipick` simulation
service as `hostname: ${MOBIPICK_CONTAINER_HOSTNAME:-mobipick}`. Manual Compose
invocations leave the variable unset and keep the previous hostnames. Because
`GAZEBO_MASTER_URI` and the fallback `ROS_MASTER_URI` point at `mobipick`, the
GUI starts the simulation with `docker compose run --use-aliases` so the
service's `mobipick` network alias resolves inside the sim container and from
the tool containers (Docker's embedded DNS otherwise resolves it through the
container hostname). The variable is only exported while `MOBIPICK_ROS_USE_IP`
is `1` (the default), because nodes that advertise hostnames instead of IPs
must keep a hostname their peers can resolve. As a visible side effect, prompts
inside container terminals show the host's hostname. The tool entries live in
`mobipick_gui.desktop_launcher.TOOL_DESKTOP_ENTRIES`; extend that table when
adding a new container tool launch. The bundled `rviz_icon.png`,
`rqt_icon.png`, and `gazebo_icon.svg` are copies of the upstream ROS Noetic
rviz, rqt_gui, and Gazebo 11 icons.

To also add the launcher to the Ubuntu/GNOME dock for one-click startup, run
the checkout helper:

```bash
./install_desktop_launcher.sh
```

Package installs provide the equivalent command:

```bash
mobipick-labs-docker-gui --install-desktop-launcher
```

The command preserves the existing dock favorites, is safe to run repeatedly,
and only changes files and settings for the current user. The Setup Wizard
offers the same action as a checked-by-default setup choice.

Runtime commands are executed in two ways:

- Long-running tasks use `QProcess` through `ProcessTab`. Output is merged,
  buffered to complete lines, sanitized for terminal escape sequences,
  converted from ANSI color to HTML when needed, and flushed into
  `LogTextEdit`. Uncolored ROS warning lines receive a yellow fallback.
- Short helper commands use `subprocess.run()` through `MainWindow._sp_run()`,
  which injects the same runtime environment and logs the command to the GUI.

Docker Compose is always invoked by the GUI with the bundled compose file and a
fixed project name. The compose file is not intended to be run directly during
normal GUI use, because the GUI also tracks process state, xhost access, tabs,
recording, and cleanup.

## Prerequisites

The application targets Linux desktops with X11 or Wayland.

- Python 3.8 or newer.
- PyQt5 5.15 or newer.
- Docker Engine and the Docker Compose plugin available to the current user.
- An X11, XWayland, or native Wayland desktop path for Gazebo, RViz, and RQt.
  Screen recording still requires X11/XWayland.
- Optional but recommended: NVIDIA Container Toolkit for GPU-accelerated
  simulation.
- Optional tools for specific features:
  - `wmctrl` and `xprop` for window layout capture/replay on X11 sessions.
  - On GNOME Wayland sessions, the bundled GNOME Shell extension instead
    (`mobipick-labs-docker-gui --install-gnome-window-extension`, then log
    out and back in), because `wmctrl` cannot see native Wayland windows.
  - `graphviz` for workspace graph rendering.
  - `ffmpeg` for Auto Launch screen recording.

Common Ubuntu setup:

Use **Tools > Setup Wizard** and copy the Host Dependencies script. The script
is interactive: before each major step it explains what will run, why it is
needed, prints the commands, and asks for `y/N` confirmation. The guarded steps
install apt prerequisites, replace the Docker apt repository key/source, verify
Docker package candidates, install Docker Engine and the Compose plugin,
install selected optional tools, restart Docker services, configure docker
group access, and test Docker with both `sudo` and the current user.
On GNOME Wayland, the same dependency step includes the one-time bundled
window-extension install command and reminds the user to log out and back in.

Log out and back in after changing Docker group membership, or start a shell
with `newgrp docker`.

When no matching Mobipick Labs image is installed, the GUI opens the setup
wizard so you can pull one on the host PC with streamed output. If you choose
the manual wizard option, run one of these commands and confirm in the wizard
when it finishes:

```bash
docker pull ozkrelo/x_mobipick_labs:noetic-v1.1
docker pull ozkrelo/x_mobipick_labs:noetic-v2.0
```

## Installation

Install the released package from PyPI:

```bash
python -m pip install mobipick-labs-docker-gui
```

If you installed an older released version, upgrade it in the same Python
environment:

```bash
python -m pip install --upgrade mobipick-labs-docker-gui
python -m pip show mobipick-labs-docker-gui
```

If the command was installed with `--user` and your shell cannot find
`mobipick-labs-docker-gui`, make sure Python's user script directory is on
`PATH`, for example `~/.local/bin` on many Linux systems.

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

The package version is declared in `pyproject.toml`. The fallback version used
when running directly from an unpackaged source tree lives in
`mobipick_gui/version.py`; keep both values in sync for releases.

Build source and wheel artifacts with:

```bash
python -m pip install --upgrade build
python -m build
```

Optionally validate the artifacts before uploading:

```bash
python -m pip install --upgrade twine
python -m twine check dist/*
```

Package data is declared in both `pyproject.toml` and `MANIFEST.in`. When adding
new runtime assets under `mobipick_gui/resources/`, update both files so editable
installs, wheels, and source distributions all behave the same.

The console scripts installed by the package are:

```text
mobipick-labs-docker-gui = mobipick_gui.cli:main
mobipick-labs-docker-gui-remote = mobipick_gui.remote_client:main
```

### PyPI release flow

Publishing is handled by `.github/workflows/python-publish.yml`. The workflow
builds the distributions and uploads them to PyPI when a GitHub Release is
published. Merging to `main` alone does not publish a new PyPI version.

Recommended release steps:

1. Run the tests and build locally.
2. Merge the release branch to `main`.
3. Create and publish a GitHub Release from `main` with `./release.sh`.
   The script suggests the next patch tag from existing releases/tags and
   updates, commits, and pushes the package version bump before tagging when
   needed. Explicit `--target` releases still require the package version to
   already match the tag.
4. Confirm the workflow succeeds, then verify the new release on
   <https://pypi.org/project/mobipick-labs-docker-gui/>.

## Configuration model

Bundled defaults live under `mobipick_gui/resources/config/`.
`mobipick_gui.config.CONFIG_DEFAULTS` supplies hard defaults, then
`config/gui_settings.yaml` is merged over them, then the per-user
`gui_settings.yaml` is merged last.

Important local configuration and environment overrides:

- `MOBIPICK_GUI_DATA_ROOT` points the package at an alternate resources root.
- `MOBIPICK_GUI_CONFIG` points to an alternate per-user GUI settings file.
- `launch_sequence.robot_race: true` in the per-user `gui_settings.yaml`
  replaces Auto Launch progress bars with full-resolution synchronized robot
  animations. `ROBOT_RACE=true` remains available as an environment override.
  The default is `false`; the setting can also be changed under **Tools >
  Automation > Use Robot Race Animations**.
- `MOBIPICK_WORKSPACE_CONFIG` points to an alternate workspace registry.
- `XDG_CONFIG_HOME` and `XDG_DATA_HOME` control the default per-user roots.

Default per-user state locations:

```text
~/.config/mobipick-labs-docker-gui/gui_settings.yaml
~/.config/mobipick-labs-docker-gui/workspaces.yaml
~/.config/mobipick-labs-docker-gui/window_layouts/{workspace}.yaml
~/.config/mobipick-labs-docker-gui/docker_cp_image_tag.yaml
~/.config/mobipick-labs-docker-gui/docker_cp_profiles/{workspace}_docker_cp_image_tag.yaml
~/.config/mobipick-labs-docker-gui/launch_sequences/
~/.config/mobipick-labs-docker-gui/profiles/
~/.local/share/mobipick-labs-docker-gui/recordings/
~/.local/share/mobipick-labs-docker-gui/image_builds/
```

Keep bundled resource files immutable at runtime. User edits should be written
to per-user config/data paths.

The top-level **Settings** menu exposes migration and troubleshooting actions:
**Export All Settings...** writes the workspace registry, per-user GUI settings,
and workspace profiles to one portable YAML file; **Import All Settings...**
restores that file under a chosen workspace master folder; **Show Configuration
Paths** displays the writable config/data paths that the GUI manages; **Copy
Full Reset Command...** shows a destructive warning and copies an opt-in
terminal command for deleting all per-user GUI config/data roots from the PC.
The reset command requires typing `DELETE_MOBIPICK_GUI_CONFIG` in the terminal
before it runs `rm -rf`; it does not remove Docker images, Docker containers,
ROS workspaces, the source checkout, or bundled package defaults.

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

In remote ROS master mode no local Roscore runs, so the GUI creates the
external `mobipick` network itself before starting any container (compose
otherwise fails with `network mobipick declared as external, but could not be
found`), and a button that pins `service: mobipick_cmd` is run as
`mobipick_remote_cmd` instead: the bridge addresses of `mobipick_cmd` are
unreachable from the robot, while the remote service is the same tool on host
networking.

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
Interactive terminals pin `catkin build` to the selected workspace root, so
package-local operations such as `catkin build --this` are not redirected by
stale nested `.catkin_tools` metadata. An explicit `--workspace` option still
takes precedence.

`WorkspaceManagerDialog` is the UI for:

- choosing or creating a master folder;
- discovering child workspaces with `src/`;
- adding or creating standalone workspaces;
- editing inheritance and image/button/launch profiles;
- building the active workspace inside Docker;
- exporting/importing portable GUI settings;
- rendering the workspace graph.

The main window keeps the workspace manager alive as an independent non-modal
top-level window, allowing it and the main log window to be minimized or used
separately.

Builds use `catkin build` inside the selected development image. Public root
images are configured as image-default only and do not mount host workspaces.

## Image profiles and setup wizard

Image behavior is controlled by `images` in `gui_settings.yaml`.

- `default` is the preferred Docker image.
- `discovery_filters` controls which local images appear in the combo box.
  The **Configure Image Filters** dialog shows these filters next to the
  blacklist and previews which local Docker images will be used, ignored, or
  hidden.
- `blacklist` contains image refs or glob patterns ignored after discovery
  filtering.
- `profiles` maps image refs or glob patterns to container user behavior,
  workspace support, compatible workspaces, working directory, entrypoint, and
  tooltip description.

On startup, the setup wizard opens automatically only when no Docker image
matching `images.discovery_filters` is installed. A missing configured default
does not reopen the wizard: the GUI uses an installed workspace image, then a
compatible host-user image, then another discovered Mobipick Labs image for
that session without changing the saved default. Missing optional host tools
are reported in the GUI Log tab instead. Set
`MOBIPICK_GUI_SUPPRESS_OPTIONAL_DEPENDENCY_WARNINGS=1` to suppress those
warnings.

The setup wizard first explains operating system and hardware compatibility,
including the tested Ubuntu releases, the dedicated CUDA-capable NVIDIA GPU
requirement, and reference GPU memory usage. It then checks common Ubuntu host
dependencies and shows one checkbox per package so users can copy a selected
set of short, commented `apt` commands into a terminal, install the tools
themselves, and mark the step done. Docker setup uses explicit repository,
package installation, and user-group commands without wrapping them in an
interactive Bash script. The following
NVIDIA Container Toolkit page links to the official installation guide, can
copy the guide URL or the elevated terminal test command, and runs a
non-blocking current-user Docker GPU test. That test passes only when the
container exits successfully and reports NVIDIA-SMI, driver, CUDA, and GPU
table evidence. The wizard can then
pull public images on the host PC with streamed output, pause for a manual pull
confirmation, choose a default image, build a host-user development image, and
clone/build `DFKI-NI/mobipick_labs` from source in a host-mounted workspace.
Its setup choices also include installing the application launcher and adding
it to the Ubuntu/GNOME dock. This choice is enabled by default and can be
cleared on non-GNOME desktops or when launchers are managed separately.
Each optional wizard page has a skip button.
The summary page can launch the stock Docker Compose simulation as an isolated
display smoke test and capture its combined terminal output. It disables host
workspace mounting, runs as root against the workspace baked into the image,
and prefers the simplest locally available public image in this order:
`mobipick_labs`, then `x_mobipick_labs`. If neither family is available
locally, it uses the Docker image selected as the wizard default.
This keeps private workspace commands and host-user image settings out of the
basic display test. If Gazebo is not visible, **I Cannot See the Simulation**
stops the test and opens a
privacy-scrubbed bug report with the output and relevant host, GPU, image,
workspace, and GUI diagnostics selected for a GitHub issue.
It does not enable `docker cp` paths by default; configure any copy rules later
from **Tools > Docker > Configure Docker cp Paths**.
The custom image builder writes a Docker build context under the per-user data
directory, copies `custom_entrypoint.sh`, adds a host-matching user, installs
passwordless sudo, and tags the result according to the wizard fields.

The source install step creates
`<master folder>/clean_mobipick_labs_ws/src/mobipick_labs` by default, runs the
work inside Docker with the workspace mounted to the host, sources
`/opt/ros/noetic/setup.bash`, then executes `./install-deps.sh` and `./build.sh`.
Output streams into an **Install Source** tab. Existing git checkouts are
updated; existing non-git paths stop the step with an explicit error.

## Button profiles

Default toolbar buttons are loaded from
`resources/config/button_commands_labs.yaml`. `load_button_layout()` supports
workspace-specific replacements through the workspace registry.
Use **Tools > Configure Toolbar Buttons** to edit the active profile from the
GUI. Workspace edits are saved as writable per-user copies named for the active
workspace, and that workspace is updated to point at its copy. Packaged global
profiles are also copied before saving. The dialog shows the editable button
key, label, command, stop command, compose service, host execution, and
tooltip; other execution fields are preserved when saving. The stop-command
column is shown only when at least one button uses host execution. Its cells
are enabled only for host command buttons, and the command runs after the GUI
sends SIGINT to the original command process.
Toolbar profiles remain editable while workspace processes are running. Saved
changes are used by later launches without rebuilding process tabs. A running
command retains the launch and stop settings with which it was started; if its
button is removed from the profile, the button remains available until that
command stops.

Use **Import from Workspace...** to select another registered workspace and
choose the individual toolbar buttons to copy. Imported buttons replace active
buttons with the same key, while new keys are appended; buttons not selected
for import remain unchanged. Saving writes an independent profile for the
active workspace, so later edits do not change the source workspace. Use
**Load Profile** and **Export Profile** to move complete button configurations
as one YAML file. The automatic save location is the per-user XDG config
directory so an installed package is never modified at runtime; exporting is
the way to place a profile in a private repository or share it with another
setup.

Button entries can be:

- `kind: builtin` with actions such as `sim`, `tables_demo`, `rviz`, and
  `rqt_tables`;
- `kind: command` with an arbitrary command executed either in Docker or on the
  host.

Every editable top-row button has a `command`. Default commands for the bundled
`sim`, `tables`, `rviz`, and `rqt` buttons live in the button profile and are
used by their builtin start/stop wrappers. Legacy workspace `sim_command`
values remain supported and are written into the button profile when saved from
the GUI editor.

Check **Host** in the editor to run a command directly on the host and save
`host: true` in the button profile. Unchecked commands continue to run in the
configured Mobipick Docker service. Stopping Roscore shuts down the dependent
Docker stack but leaves running host commands alive. Host commands also start
directly without checking or automatically starting Roscore. When the GUI's
local Roscore is already running, a newly started host command receives
`ROS_MASTER_URI` pointing to that container and `ROS_IP` set to the host side
of the `mobipick` Docker bridge. In remote ROS master mode a host command
instead receives the configured remote `ROS_MASTER_URI` and the host address
that routes to it. These overrides apply only to host commands.

Command entries can declare:

- `requires_roscore`;
- `reuse_tab`;
- `world_config_required` and `world_arg_name`;
- `setup` or `pre_command`;
- `host`;
- `stop_command`;
- `log_command`;
- `pass_ros_master_uri`;
- `service`, for choosing the compose service used by Docker command buttons.
  Leave it empty for the normal tool service, or use `mobipick` for launch
  files that start Gazebo themselves and need the simulator service identity.

Profiles can also define as many as five generic ROS arguments on each button.
For slot 1, set `arg_1_name` to the argument name, provide the dropdown values
in `arg_1_options`, and set `arg_1_applies: true` on every button that should
receive it; slots 2 to 5 use the corresponding `arg_2_*` to `arg_5_*` fields.
A slot must use the same name and option list throughout one profile. Named
slots appear as combo boxes in the main GUI, and an enabled button command
receives the selected `name:=value`. If no slot has a name, no generic argument
controls are shown. Select a
button in **Configure Toolbar Buttons**, then use **Configure Arguments...** to
edit the names, comma-separated combo options, and per-button flags in a
separate dialog. Argument details remain out of the main profile table and
travel with loaded or exported profiles.

### Option rules

A profile can make dropdown options invalid under conditions without any
workspace-specific GUI code. `mobipick_gui/option_rules.py` loads
`<profile stem>_rules.yaml` beside the button profile, or `option_rules.yaml`
in the same directory, whenever the profile is loaded:

```yaml
rules:
- when:
    remote_master: true          # "Use remote ROS master" is on
  only:
    world: [cic_tables]          # every other world becomes invalid
  reason: the real robot only runs the cic_tables environment
- when:
    model_profile: [o3, o3-jev]  # a list matches any entry
  invalid:
    disc_mode: [cpu]
- when:
    remote_master: true
    running.tables_demo_bringup: false
  block_start: all               # or a list of button keys
  except: [tables_demo_bringup]
  reason: on the real robot start tables_demo_bringup first
- when:
    remote_master: true
  remind_start: [tables_demo_bringup]   # or all
  notice: launch rgbd_snapshot_server.py on the real robot
  clipboard: rgbd_snapshot_server       # optional
```

`when` conditions and the `invalid` / `only` lists name `world` or any generic
argument name; `when` can also read `remote_master` and
`running.<button key>`. Every condition of a rule must hold. Invalid options
are greyed out with the reason as tooltip. A selection that becomes invalid
switches to the first valid option; when the user's own change (a dropdown or
the remote master checkbox) caused it, a non-modal popup explains it.
`block_start` refuses starting toolbar buttons, Roscore and Terminal (never
stopping them) with a popup. Pressing Auto Launch checks every step first,
treating the run's own steps as running (so a rule waiting for a step of
the same run passes), and refuses the whole run with a popup when any step
stays blocked. A step still blocked when its turn comes is logged and never
marked ready. `POST /args` rejects invalid values and a button press returns
`accepted: false` with the rule's reason instead of opening a popup.
`remind_start` does not refuse anything: when a named toolbar button starts
(by click, remote control or Auto Launch) the GUI logs the `notice`, shows it
in an information popup and copies `clipboard`, if set, to the clipboard.
`running.*` conditions are read when a start or dropdown change is evaluated,
not polled. Malformed rules are skipped and reported in the GUI log. The
profile editor does not rewrite the rules file.

The GUI normalizes all entries and creates matching process tabs and
start/stop visual state.

`roscore` and `terminal` are fixed top-row buttons and are not stored in button
profiles. `sim` and `rviz` are required profile buttons: they cannot be
removed, but their `command` fields can override the default simulation and
RViz launch commands. Other profile buttons can be added, removed, reordered,
or changed.

## Auto Launch

`load_launch_sequence_plan()` resolves Auto Launch YAML from a workspace
profile, per-user launch sequence directory, or fallback filenames derived from
the active button profile.

The saved format stores:

- `mode`: `legacy` or `advanced`;
- `timeline`: legacy button key plus fixed `at_seconds`;
- `processes`: advanced button definitions with `duration_seconds`, optional
  `depends_on`, `dependency_type` (`hard` or `soft`), and
  `ready_percentage`;
- `process_settings`: settings for all advanced rows, including disabled rows,
  so temporarily unused dependency choices are retained;
- `shutdown.order`: reverse or custom stop order;
- optional `shutdown.skip`;
- optional button text/tooltip metadata;
- `recording.start_delay_seconds`.

`AutoLaunchWizard` keeps the fixed-delay editor in its Legacy tab and provides
a dependency-aware Advanced tab. Old YAML without `mode` or `processes` is
loaded as Legacy. In Advanced mode, hard dependents wait for the dependency's
full readiness duration, while soft dependents wait for the configured
percentage. Processes already running at the start are ready immediately.
Each Advanced row also has an interactive readiness measurement: **Measure**
launches the process at time zero, and **Ready** records the user's confirmation
into `duration_seconds`, rounded upward to one decimal place.
**Import...** and **Export...** move a plan between the wizard and an
arbitrary YAML file. Export writes the shown state with
`save_launch_sequence_plan()`; import parses the file with
`read_launch_sequence_file()` (the strict parser `load_launch_sequence_plan()`
also uses) and only refills the widgets, so the active plan changes when the
user presses Save. Imported button text overrides are kept for that save.
In remote ROS master mode the local Roscore and simulation entries are dropped
from the sequence and from the progress window, since that master already
provides them; advanced dependents of the simulation start at time zero, and a
shortened legacy timeline is shifted so its first remaining entry starts right
away.
Starting Auto Launch displays an always-on-top readiness progress window. Its
duration is the latest legacy timeline offset or the effective advanced
dependency schedule (including already-running process shortcuts). It reports
per-process progress, includes saved window-layout replay as a final milestone,
reports completion for one second, and then hides automatically. With
`window_layout.apply_delay_ms: auto`, layout replay runs one second after every
process is ready. Unmatched saved entries are retried for 30 seconds rather
than polled indefinitely; launching another managed window starts a fresh
retry window.
Profiles are saved to a writable per-user path when the source is a packaged
resource. Auto Launch can also coordinate window layout replay and delayed
recording startup.

## Responsive main-window layout

Every control row of the main window (workspace, ROS master, toolbar buttons,
Auto Launch/argument row, scripts, custom command, bottom controls, and search)
uses `FlowLayout` from `flow_layout.py` instead of `QHBoxLayout`. A
`QHBoxLayout` makes a window at least as wide as the sum of its children, so a
workspace with many button profiles pinned the main window to a minimum width
wider than a laptop panel: the window could neither be shrunk nor usefully
maximized. `FlowLayout` wraps its items onto additional lines, so the minimum
width is only that of the widest single item; items whose size policy expands
still share the slack of their own line, and a zero-width expanding spacer on a
line keeps that slack instead of stretching the widgets next to it. Nested
wrapping rows work because the layout clamps each item to the line width and
asks it for `heightForWidth`.

Two helpers in `main_window.py` support this: `_configure_shrinkable_combo`
gives combo boxes a minimum contents length (image references, workspace paths,
and script names are otherwise wider than the screen), and `_labeled_control`
groups a label with its control so a wrap never separates them.

`window_utils.fit_geometry_to_screen` clamps a restored geometry to the
available area of the screen the window will appear on. Geometry saved on a
large external monitor is both too big and off-screen on a laptop panel, which
leaves the window unreachable. On Wayland only the size is applied, as before.

When adding controls to the main window, add them to the existing wrapping
rows and verify at a narrow width (for example 800 px) that nothing is clipped.

## Recording and window layout

Recording captures X11 screen video with `ffmpeg -f x11grab`. It is armed by the
GUI checkbox and starts only after Auto Launch begins and the timeline/layout
delay has elapsed, or at once through `POST /recording/start` on the remote
API. Recording sessions create timestamped folders containing the MP4,
`ffmpeg.log`, and saved HTML logs when requested.

A recording is a list of segments (`segments/segment_NNN.mp4`): **Pause**
(Recording Control window or `POST /recording/pause`) ends the running ffmpeg
segment and **Resume** starts the next one, so idle time is never captured.
**Stop** concatenates the segments into `<name>.mp4` and renders
`<name>_<speedup>x.mp4` next to it (`recording.speedup`, default 4); the API
emits `recording_exported` with both paths when the export is done.

Window layout capture uses `WindowLayoutManager` on top of a backend from
`window_control.py`. On X11 sessions the backend shells out to `wmctrl` and
`xprop`. On Wayland sessions those tools only see XWayland windows, so the
backend talks over D-Bus to the GNOME Shell extension shipped in
`mobipick_gui/resources/gnome-shell-extension/` (installed with
`mobipick-labs-docker-gui --install-gnome-window-extension`; rerun this after
GUI upgrades because GNOME Shell only loads extension changes after a fresh
login). The extension exposes `ListWindows`, `MoveResize`, `SetWorkspace`,
`Activate`, `ClearAttention`, `Unmaximize`, and `SetAbove` on
`/org/gnome/Shell/Extensions/MobipickWinCtl`, and windows are addressed by the
Mutter window id. `MainWindow.keep_window_above` uses
`SetAbove` for the always-on-top helper windows, since Wayland ignores
`Qt.WindowStaysOnTopHint`. When the extension is not available on Wayland the manager
falls back to `wmctrl` for XWayland windows.
The manager records the baseline windows present when the GUI starts, excludes
the GUI/helper windows during capture, stores a separate layout for each active
workspace, and applies saved positions to matching new windows after the
configured delay. The `window_layout.state_file` setting may include
`{workspace}` or `{workspace_slug}`; paths without a placeholder are treated as
a base location and expanded into one YAML file per workspace.

## Remote control API

The GUI can expose a JSON-over-HTTP API so another machine, or an automation
agent such as Claude Code, can press toolbar buttons, wait until a launch has
settled, read log tabs, and run commands in persistent ROS 1 shells inside the
Mobipick containers. The server is off by default. Enable it with any of:

- `mobipick-labs-docker-gui --remote-control [--remote-host H] [--remote-port P] [--remote-token T]`
- `MOBIPICK_GUI_REMOTE_CONTROL=1` plus optional `MOBIPICK_GUI_REMOTE_HOST`,
  `MOBIPICK_GUI_REMOTE_PORT`, and `MOBIPICK_GUI_REMOTE_TOKEN`
- **Tools > Remote Control > Enable Remote Control API** at runtime

Remote control is opt-in for each launch. A persisted
`remote_control.enabled: true` setting is ignored by the GUI command unless
`--remote-control` or `MOBIPICK_GUI_REMOTE_CONTROL=1` enables the API.

The GUI normally runs on the host and the processes it launches are mostly
containers, though configured buttons can also run host processes. The
default bind address is `0.0.0.0:8765` so other machines on the network can
reach the API; use `127.0.0.1` for a local-only agent.
Anyone who can reach the port can run commands inside the containers, so set
`remote_control.token` on shared networks; clients then send
`Authorization: Bearer <token>` (or `?token=`).

Implementation lives in `mobipick_gui/remote_control.py` (server, event bus,
shell sessions), `mobipick_gui/remote_adapter.py` (the `MainWindow` bridge and
its thread-safe status snapshot), and `mobipick_gui/remote_client.py` (the
`mobipick-labs-docker-gui-remote` CLI, standard library only). Configuration
keys are documented in `config/gui_settings.yaml` under `remote_control`.

Docker, host subprocess, network discovery, and window-manager commands never
run on the Qt event thread. Their results return through Qt signals, so a slow
or unreachable Docker daemon leaves an operation pending without freezing the
window. `GET /status`, `GET /buttons`, and `GET /tabs` read the last published
snapshot directly on HTTP worker threads, as do presence and event requests.
Endpoints that must inspect or change widgets are marshalled to Qt and return
a clear 504 error after one second if Qt cannot service the action.

### Endpoints

`GET /` returns this list as JSON. Responses are `{"ok": true, ..., "seq": N}`
where `seq` is the newest event sequence number, so a client can chain a
click with a wait without missing events.

| Method and path | Purpose |
| --- | --- |
| `GET /status` | Workspace, image, world, cached roscore/sim state, buttons, tabs, shells, active dialog. |
| `GET /buttons` | Toolbar buttons with `state` (`red` stopped, `green` running, `yellow` busy, `grey` unavailable), `tooltip`, `runs_on` (`host` or `container`), the log `tab` key, the toolbar `args` the button receives and the resulting `full_command`, plus readiness from the auto-launch estimates: `startup_seconds` (the plan's `duration_seconds`, `null` when the button has none), `started_at`, `ready_at`, `ready_in_s` and `ready` (running and past the estimate). |
| `POST /buttons/{key}/click`, `/start`, `/stop` | Press a button. `start`/`stop` are idempotent. Body may contain `args` (`{"anygrasp_mode": "real"}`, selected before the press), `wait_for` (event names) and `timeout`. Keyed events (`button_state`, `button_ready`, `process_finished`) only match this button; waiting for `button_ready` on a button that is already running and past its estimate returns at once with `already_ready: true`. |
| `GET /args`, `POST /args` | The toolbar argument dropdowns (generic `arg_N` slots from the button profile) and the world selector: `name`, current `value`, `options` and the `buttons` each applies to. `POST` selects values by name (`{"anygrasp_mode": "real", "world": "moelk_tables"}`) exactly like choosing them in the toolbar, so no profile edit or reload is needed; unknown names or values are rejected. `invalid` maps options the profile's option rules currently forbid to the reason, and `POST` rejects them too. |
| `GET /presence`, `POST /presence`, `DELETE /presence` | Declare that a client is using the GUI (`{"name": "<agent>", "ttl_s": 600, "note": ""}`; the name is chosen by the client, so any agent can use its own, and several may be present at once) or withdraw it. The first `POST` for a name returns a `token` that the `DELETE` must carry (`{"name": ..., "token": ...}`); a refresh by a namesake gets no token and its bye is refused with 409, so two agents that picked the same name cannot withdraw and clean up after each other. While a client is present the window icon glows bright and the GUI log records who is working. Presence is kept alive by activity, not by heartbeats: every request the client sends refreshes it (with several clients present, name yourself with the `X-Client-Name` header or a `client` field; an anonymous request refreshes the client present the longest, the one that also owns new processes), and so does a shell command it started that is still running or an `follow=1` stream it keeps open. Only `ttl_s` (default 10 min, max 30 min) of complete idleness lapses it. The server remembers every button, custom command, and shell the client started; when the client withdraws (without `"keep": true`) or lapses, the GUI stops those and logs the cleanup, so a crashed agent cannot leave the simulator running. Clients should not run a separate presence-refresh process: one that outlives its owner keeps a dead agent "present" for hours. |
| `POST /reload` | Re-read `gui_settings.yaml` and the workspace button profile without restarting the GUI. Button commands, labels, tooltips and argument slots are picked up for the next press; running processes and their tabs are preserved. |
| `GET /recording`, `POST /recording/{start\|pause\|resume\|stop}` | Screen recording state (`active`, `paused`, `segments`, `recorded_s`, `video_path`, `video_speedup_path`) and its control without Auto Launch. A recording started by a client is stopped when that client leaves. |
| `POST /tabs/{key}/stop` | Stop the process behind a log tab: a button process (same as `/buttons/{key}/stop`), a `customN` command started with `/command`, or a remote shell. |
| `GET /events?since=N&names=a,b` | Event history; add `follow=1&timeout=s` to stream NDJSON. |
| `POST /wait` | Block until one of `events` arrives after `since` (default: now) or `timeout`; `key` restricts keyed events to one button or tab. |
| `GET /tabs`, `GET /tabs/{key}?tail=N&grep=RE` | Log tab list and plain-text tab contents. |
| `GET /dialogs`, `POST /dialogs/dismiss` | Inspect or close the active modal dialog (`{"button": "Continue"}`, `accept`, `reject`). |
| `POST /command` | Run text through the GUI custom command box. |
| `POST /shell` | Open a shell session (`{"name", "stream", "root", "robot"}`); blocks until ready. The ROS tool container is the default; `"robot": true` ssh-es onto the robot PC instead (remote ROS master mode only) to debug that machine. `GET /status` reports the choice under `shell`. |
| `POST /shell/{id}/exec` | Run a command: `{"command", "stream", "tail", "grep", "max_lines", "timeout", "wait"}`. |
| `GET /shell/{id}/output?since=N&command=ID&tail=N&grep=RE` | Buffered output; `follow=1` streams NDJSON until the command finishes. |
| `POST /shell/{id}/interrupt` | Send `INT` (default), `TERM`, `KILL`, or `HUP` to the foreground command. |
| `POST /shell/{id}/settings`, `DELETE /shell/{id}` | Change the session `stream` default; close the session and its container (a robot shell: end the ssh connection and kill what it left running). |
| `POST /quit` | Close the GUI with its normal container cleanup. |

Events: `button_state`, `button_ready` (the button has been running for its
configured `duration_seconds`; immediate for buttons without an estimate),
`process_finished`, `auto_launch_started`,
`auto_launch_ready`, `auto_launch_complete` (every process in the plan reached
its ready time), `window_layout_applied` (the saved layout was replayed, which
is the usual "everything is up" signal), `auto_launch_stopped`,
`shell_opened`, `shell_exited`, `shell_closed`, `client_connected`,
`client_disconnected` (with `expired: true` when a TTL ran out), and
`gui_closing`.

### Window icon glow

While the API is listening the main window icon carries a light-blue halo:
light blue while nobody is connected, **green** while a client has declared
presence with `POST /presence`, and pulsing while a request is being served.
The colour change (not just brightness) is what makes "an agent is driving
this GUI" readable at a glance on the dock. On GNOME
the dock icon is styled through the bundled shell extension
(`SetAppGlow`, protocol version 4); on other desktops `setWindowIcon` is used.

### Shell sessions and output streaming

A session is `docker compose run --rm -T ... <tool service> python3
enter_host_shell.py bash --noprofile --norc` with `terminal.bashrc` sourced on
start, so it has the same ROS environment and user as **Open Terminal**.

`{"robot": true}` opens the session on the robot PC instead, as `ssh -o
BatchMode=yes -o ConnectTimeout=10 <user>@<host> bash --noprofile --norc`. That
shell is for debugging the robot machine itself - its processes, drivers, logs,
services, disks - which a container cannot see. **ROS work belongs in the
container shell**, which carries the workspace chain and reaches the same
master over the network, so the container stays the default even in remote ROS
master mode; `ros.robot_shell_by_default: true` reverses that, and
`{"robot": false}` always forces the container. `{"robot": true}` outside
remote master mode is refused: there is no robot to reach.

Settings: `ros.robot_ssh_user`, `ros.robot_ssh_host` (empty means the host of
`ROS_MASTER_URI`), `ros.robot_ssh_options`, `ros.robot_shell_by_default`. The
ssh key must be in place: `BatchMode=yes` turns a missing one into a clear
startup error instead of a password prompt. The session's startup output names
the host, user, `ROS_DISTRO` and `ROS_MASTER_URI` it found; the robot's own
environment comes from `/etc/profile` and `~/.bashrc`, and `ROS_MASTER_URI` is
only filled in when the robot leaves it unset. `describe()` and `GET /shell`
report `runs_on` (`robot`/`container`) and the `target`, and `GET /status`
carries a `shell` block (`default`, `container_service`, `robot_available`,
`robot_target`, `hint`) so a client can see where a new shell would land. The
shell is stateful (`cd`, `source`, exported variables persist) and its output
is mirrored into a closable **Remote Shell N** tab. Each command is wrapped
with a base64 `eval` and a completion marker, so quoting and multi-line
commands are safe and the API knows the exit code. Standard input is
`/dev/null`; interactive prompts fail fast instead of hanging.

Output is buffered per session with sequence numbers. The `stream` flag
decides whether an `exec` response carries the lines at all: `stream: false`
returns only the exit code and line count, and the output stays retrievable
through `/output` with `tail`, `grep`, `since`, or `command`. The per-session
default can be changed with `/settings`. Long-running commands use
`wait: false` and are polled or followed; `interrupt` sends SIGINT to the
foreground process group child of the session shell through `docker exec`, or
through the same `ssh` target for a robot shell (ssh joins its remote arguments
with spaces, so that command is passed as one already-quoted word).

### Client and agent workflow

```bash
export MOBIPICK_GUI_REMOTE_URL=http://<gui-host>:8765   # and MOBIPICK_GUI_REMOTE_TOKEN
mobipick-labs-docker-gui-remote hello claude --note "tables demo"   # icon glows until bye; repeat every <10 min
mobipick-labs-docker-gui-remote status
mobipick-labs-docker-gui-remote click auto_launch --wait window_layout_applied,auto_launch_complete --timeout 240
mobipick-labs-docker-gui-remote --text tab sim --tail 40 --grep "ERROR|WARN"
mobipick-labs-docker-gui-remote shell open
mobipick-labs-docker-gui-remote --text shell exec 1 "rostopic list" --tail 20
mobipick-labs-docker-gui-remote shell exec 1 "rosrun tables_demo_planning tables_demo_node.py" --no-wait
mobipick-labs-docker-gui-remote stop-tab custom1                     # stop a /command launch
mobipick-labs-docker-gui-remote reload                               # after editing a button profile
mobipick-labs-docker-gui-remote bye claude                           # stops anything claude left running
mobipick-labs-docker-gui-remote --text shell output 1 --follow --grep "ERROR|Success" --timeout 120
mobipick-labs-docker-gui-remote shell interrupt 1
mobipick-labs-docker-gui-remote shell close 1
```

A Claude Code skill describing this workflow with plain `curl` is kept in
two identical copies: `.claude/skills/mobipick-gui-remote/SKILL.md` (loaded
automatically in this checkout) and
`mobipick_gui/resources/skills/mobipick-gui-remote/SKILL.md` (shipped in the
package). Port it to another machine or project with:

```bash
mobipick-labs-docker-gui-remote skill --install ~/.claude/skills        # user-wide
mobipick-labs-docker-gui-remote skill --install /path/to/repo/.claude/skills
mobipick-labs-docker-gui-remote skill            # print it
```

A test fails when the two copies drift apart.

`tests/mobipick_gui/test_remote_live.py` holds opt-in checks against a running
GUI: with `MOBIPICK_GUI_REMOTE_LIVE=1` (and `MOBIPICK_GUI_REMOTE_URL` when the
port differs) it opens a fresh remote shell, verifies that TCPROS
subscriptions receive `/clock` and joint states from the simulator, and closes
the shell. It starts and stops nothing.

If a request reports HTTP 504 with a `dialog` entry, a modal dialog (for
example the workspace mismatch warning) is waiting; answer it with
`dismiss <button text>` and retry. Requests are served even while a dialog is
open because Qt modal loops keep processing queued calls.

## Container display backends

`display.mode` in `gui_settings.yaml` accepts `auto`, `x11`, or `wayland`.
Automatic mode exposes every valid host display socket to one-off containers,
and selects the backend matching the host session: native Wayland on Wayland
and X11 on Xorg. If the native transport is unavailable, automatic mode falls
back to the other transport. Set `display.mode: x11` for an older image that
does not contain Qt's Wayland platform plugin.

Gazebo and RViz from ROS Noetic are exceptions. Their OGRE 1.9 renderer uses
GLX and requires an X11 parent window, so the GUI launches those applications
through XWayland on a Wayland desktop. On NVIDIA hosts it also enables PRIME
render offload and selects the NVIDIA GLX vendor. This avoids both the native
Wayland `Invalid parentWindowHandle` failure and the accelerated XWayland
viewport appearing black.

The GUI adds display mounts to each `docker compose run`; the compose file no
longer mounts all of `/run/user`. X11 authorization uses a mounted Xauthority
cookie when one is available and otherwise grants the selected container user
temporary access with `xhost`. Native Wayland requires the image to contain
Qt's Wayland platform plugin. Host-user images newly built by the setup wizard
install `qtwayland5`.

Focal-based NVIDIA images also need Wayland client 1.20 or newer. Current
NVIDIA Container Toolkit releases inject `libnvidia-egl-wayland2.so.1`, which
uses `wl_proxy_marshal_flags`; Focal's Wayland 1.18 does not export that
symbol. The Mobipick Noetic base-image hierarchy supplies the compatible
runtime. Rebuild the hierarchy after updating that base image.

The GUI also reads the owning groups of the host's `/dev/dri/renderD*` and
`/dev/dri/card*` devices and adds those numeric groups to every Compose
service. This lets the host-matching non-root container user open the GPU
devices even when the host and image assign different IDs to `render` and
`video`.

The container entrypoint creates a private `XDG_RUNTIME_DIR` with mode `0700`
for the effective container user. When an interactive terminal changes from
root to the host-matching user, its privilege-drop helper creates a new runtime
directory for that UID before starting the shell. Every child process inherits
the corrected environment, whether or not it uses Qt. For native Wayland, the
GUI mounts only the selected host socket and links it into the active user's
private directory. This prevents Qt runtime ownership warnings without
exposing the rest of the host user's runtime directory.

Container launches also receive an unreachable D-Bus session address. This
prevents applications opened by the GUI, including commands started in its
container terminals, from delivering desktop notifications on the host. The
GUI applies this isolation after command-specific environment overrides.
During GUI-managed launches, the window controller also clears attention from
new windows. This suppresses desktop-generated "application is ready" banners,
which do not travel over the application's D-Bus connection. Xorg uses
`wmctrl`; Wayland uses the bundled GNOME Shell extension.

Recreate already-running GUI containers and terminals after upgrading so they
start with the updated entrypoint and privilege-drop helper.

See [Wayland and RViz troubleshooting](doc/wayland-rviz-troubleshooting.md) for
display and OpenGL diagnostics, including the Mesa loader/code 139 failure.

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

`docker_cp_image_tag.yaml` defines optional copy rules keyed by `default` or
by ROS workspace name.

- `host_to_container` entries run automatically after eligible containers
  appear.
- `container_to_host` entries run from **Tools > Docker > Execute Docker cp**
  for the current running tab.

When the Docker image default workspace is active, user edits are saved to
`~/.config/mobipick-labs-docker-gui/docker_cp_image_tag.yaml`. When a ROS
workspace is active, edits are saved to
`~/.config/mobipick-labs-docker-gui/docker_cp_profiles/{workspace}_docker_cp_image_tag.yaml`.
No copy rules are enabled in the bundled defaults, so fresh installs do not copy
`pick_n_place.rviz` or any other host file unless the user adds rows here.
The editor shows workspaces rather than Docker image tags. Add Row opens a
path setup dialog; the host side uses a local file picker, and the container
side can use a selected running setup container or manual path entry. Empty
profiles in the active writable file override bundled entries.

## Logging and reports

Every process tab uses `LogTextEdit`, which buffers updates to keep high-volume
process output responsive. The log widget keeps only the configured maximum
block count. ROS messages use readable wall-clock time and identify their node,
for example `[WARN] [14:26:56] [/pose_selector]: Clearing planning scene`.

GUI-originated messages and executed commands are written to the **Log** tab.
Users can save the current tab, save all tabs, or load a saved HTML log into a
closable tab. Ctrl+W closes the current tab when it has an X close button. The
bottom search row searches only the current log tab.

The bug report dialog collects selected diagnostic sections, including GUI
version, selected workspace, selected image/workspace match, optional command
outputs, workspace graph, log tab text, and user notes. Keep new diagnostics
optional so report generation remains useful on machines without every tool
installed. Report rendering passes every section through the same anonymizer
before preview, copy, save, email, or GitHub issue creation. It removes local
user and computer names, filesystem paths, network identifiers, and common
secret assignments; numeric identifiers are replaced by stars so their digit
count remains available for diagnosis.

GitHub issue links are kept below 2,000 encoded characters for broad browser
compatibility. If a report is too large, the dialog opens GitHub with the
largest safe first part, explains the truncation, and changes **Copy** to
**Copy Remaining** so the user can paste the omitted tail into the issue.

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
4. collect stop commands only for active configurable toolbar processes;
5. kill GUI-owned background `QProcess` instances;
6. stop simulator and related Mobipick containers;
7. run `clean.bash` when available;
8. revoke temporary X11 access;
9. quit the Qt application.

Interactive container stops use the configured ROS shutdown grace by default.
The **Fast stop** checkbox selects a zero-second grace for the session. Local
roscore shutdown always uses zero grace because the master and its registration
database are going away together. When a remote master survives a fast stop,
the GUI enables **Clean stale ROS nodes**, which runs `rosnode cleanup` from the
remote ROS tool service after an explicit confirmation.

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

- If no images appear, open **Configure Image Filters** to inspect
  `images.discovery_filters`, `images.blacklist`, and the preview of local
  images from `docker images`.
- If GUI tests create real dialogs unexpectedly, set
  `QT_QPA_PLATFORM=offscreen` and monkeypatch Docker discovery.
- The CLI filters Qt 5's
  `QSocketNotifier: Can only be used with threads started with QThread`
  message only while constructing `QApplication`. A copy emitted later is not
  filtered and should be investigated as an application threading problem.
- On Wayland, the CLI also filters Qt 5's benign `QWindow::requestActivate()`
  platform warning. Other Qt Wayland warnings remain visible.
- If a workspace does not mount, check the selected image profile for
  `supports_host_workspaces`.
- If RViz or Gazebo windows do not open, inspect the detected display variables,
  Docker GPU access, and OpenGL renderer. See
  [Wayland and RViz troubleshooting](doc/wayland-rviz-troubleshooting.md).
- If recordings produce no MP4, inspect the session `ffmpeg.log` and the
  configured display/resolution.
- If window layout replay does nothing on X11, install `wmctrl` and `xprop` and
  save a layout after simulator windows are visible. On a Wayland session run
  `mobipick-labs-docker-gui --install-gnome-window-extension`, log out and back
  in, and check `gnome-extensions info winctl@mobipick-labs-docker-gui`.
