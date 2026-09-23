# Mobipick Labs Docker GUI User Documentation

This guide explains the controls in the Mobipick Labs Docker GUI from a user
perspective. It describes what each button and menu option does while you are
running the simulator, ROS tools, scripts, terminals, recordings, and logs.

Only one Mobipick Labs Control window can run at a time. If you try to launch
another one, it asks you to close the running GUI before opening a new one.

## Main workflow

1. Choose a ROS 1 workspace at the top of the window.
2. Choose a Docker image and world configuration.
3. Start Roscore.
4. Start the simulator or the tools you need.
5. Watch each tab for live output.
6. Stop running tools before switching workspace, image, or ROS master mode.

The GUI changes button colors while a process is running or changing state:

- Red means the process is stopped and can be started.
- Green means the process is running and the button will stop it.
- Yellow means the process is starting or stopping.
- Grey means the action is unavailable in the current mode.

The control rows at the top and bottom of the main window wrap onto extra
lines when the window is too narrow to show them side by side, so a workspace
with many buttons still fits a laptop screen. You can shrink, maximize, and
tile the window freely; the buttons, selectors, and search field rearrange
themselves, and long image references or workspace paths are shortened
instead of forcing the window to stay wide. Their full text stays visible in
the drop-down list. A window position saved on a large external monitor is
moved and resized to fit when the GUI is opened on a smaller screen.

The GUI remembers the size and maximized state of the main window and every
tool or configuration window. On Xorg it also restores each saved position;
on Wayland, the desktop compositor chooses window positions. Closing and
opening a window again restores its most recent state.

## ROS 1 Workspace selector

The ROS 1 workspace selector controls which workspace is mounted into Docker
when the selected image supports host workspaces.

- **Docker image default** uses the workspace already inside the Docker image.
- A listed workspace mounts that host workspace into the containers.
- The tooltip tells you whether the workspace is built and whether the selected
  image can mount it.
- The GUI asks for confirmation before switching, because switching clears the
  current tabs and log output.
- If the configured Docker image is not installed, **Open Workspace Settings**
  opens the workspace manager with that workspace selected so you can update
  its Docker image.

Use **Configure Workspaces** to add, create, import, export, build, and activate
workspaces.

## Configure Workspaces

The workspace manager is used to prepare ROS 1 workspaces for the GUI.
It opens as an independent, non-modal window, so it can be minimized without
minimizing the main GUI and does not block access to the main log tabs.

- **Choose Master Folder** selects the folder that contains your workspace
  folders.
- **Create Master Folder** creates a new parent folder for workspaces.
- **Discover Workspaces** finds child folders that contain a `src` directory.
- **Add Existing** registers an existing catkin workspace.
- **Create Workspace** creates a new workspace folder with a `src` directory.
- **Remove** removes a workspace from the GUI list. It does not delete files
  from disk.
- **Set Active** switches the GUI to the selected workspace.
- **Use Image Default** switches back to the workspace inside the Docker image.
- **Show Graph** shows workspace inheritance, using Graphviz when available.
- **Export Settings** writes a portable GUI settings file.
- **Import Settings** restores a portable settings file and maps workspaces
  under the selected master folder.
- **Build Selected** builds the selected workspace inside the Docker image.
  If workspace settings were edited, save or discard those changes before
  building. Starting a build marks the selected Docker image as a workspace
  match for that workspace.
- **Save Workspace Settings** saves the selected workspace path, selected built
  underlay workspace, Docker image, simulator command, button profile, and
  auto-launch profile.
  Docker image choices are locally discovered Mobipick Docker images. Button
  profile and auto-launch profile fields offer known choices while still
  accepting custom typed values.
  **Extends** offers `/opt/ros/noetic`, the Docker image baked workspace, and
  already-built host workspaces. Choose the baked workspace when the host
  workspace should be built after sourcing the workspace already inside the
  selected image. Unbuilt workspaces with no selected underlay show an empty
  **Extends** value until they are configured and built.

Stop running containers before switching workspaces or importing settings.

## Remote ROS Master

Remote ROS master controls are hidden by default. Enable them from **View >
Remote ROS Master**.

- **Use remote ROS master** makes RViz, RQt, scripts, terminals, configured ROS
  commands, custom commands, and Host commands connect to an external ROS
  master.
- **ROS_MASTER_URI** selects the external ROS 1 master, for example
  `http://mobipick-os-sensor:11311`.
- Local Roscore and the local simulation are disabled while remote mode is on.
  Buttons that run in a container still work: the GUI creates the Docker
  network they need on its own and runs them with host networking, so their
  ROS nodes and the robot can reach each other.
  Auto Launch leaves them out of its sequence and its progress window, assumes
  they already run on the robot, and starts everything that depends on them
  right away.

Stop running containers before changing remote mode or the ROS master URI.

## Roscore

**Start Roscore** starts the ROS master container. It is the normal first step
for local simulation work.

**Stop Roscore** stops Roscore and also shuts down related running Mobipick
containers and tools. It is the fastest way to return to a clean state before
closing the GUI or changing workspace setup.

Many actions start Roscore automatically if it is needed and local remote mode
is not enabled.

## Sim

**Start Sim** launches the Mobipick Gazebo simulation using the selected world
configuration and Docker image.

**Stop Sim** stops the simulator container and returns the button to its stopped
state. Stopping Roscore also stops the simulator.

If a workspace has its own simulation command configured, that command is used
instead of the default simulator launch command.

## Tables Demo

**Run Tables Demo** starts the tables demo planning node.

**Stop Tables Demo** stops the node. The output appears in the **Tables Demo**
tab.

## RViz

**Start RViz** opens RViz with the Mobipick pick and place configuration.

**Stop RViz** stops the RViz container. The output appears in the **RViz** tab.

In remote ROS master mode, RViz connects to the selected external ROS master.

### X11, XWayland, and Wayland

The GUI automatically forwards available X11/XWayland and native Wayland
sockets into containers. Automatic mode follows the native desktop session:
ordinary Qt tools use native Wayland on a Wayland desktop and X11 on Xorg. If
the native transport is unavailable, it falls back to the other transport.
ROS Noetic's Gazebo and RViz use X11/XWayland because their OGRE renderer
creates a GLX child window. On NVIDIA Wayland sessions, the GUI also selects
NVIDIA PRIME render offload for these applications to prevent a black 3D
viewport.

Advanced users can set `display.mode` to `x11` or `wayland` in their GUI
settings override to force a backend. Use `x11` as a compatibility option for
older images or applications. The Gazebo and RViz launch actions retain their
required GLX compatibility override when automatic mode is active. Native
Wayland requires the selected Docker image to include the Qt Wayland platform
plugin. Screen recording continues to require X11 or XWayland.

Focal-based NVIDIA images must also provide Wayland client 1.20 or newer for
compatibility with the EGL-Wayland v2 library injected by current NVIDIA
drivers. If several Qt applications exit with code 127 and report an undefined
`wl_proxy_marshal_flags` symbol in `libnvidia-egl-wayland2.so.1`, rebuild the
Mobipick Docker image hierarchy from its updated Noetic base image.

The GUI grants containers the host's numeric `render` and `video` device
groups automatically. If EGL reports `Permission denied` for a device below
`/dev/dri`, restart the GUI after updating it so newly launched containers
receive those supplemental groups.

Each container user receives a private runtime directory with the ownership
and permissions expected by Qt and other XDG-aware programs. Terminals update
that directory when changing from root to the host-matching user, and every
command launched from the terminal inherits it. Native Wayland links the
selected host Wayland socket into that directory; unrelated host session
sockets are not exposed.

Applications launched inside these containers cannot send desktop
notifications to the host. This also applies to commands run from terminals
opened by the GUI.
The GUI also suppresses GNOME's own "application is ready" banners while
container application windows are opening. Other desktop applications keep
their normal attention and notification behavior.

If RViz prints `MESA-LOADER: failed to retrieve device information` and exits
with code 139, the display connection was established and the likely problem
is OpenGL/GPU passthrough rather than a missing Wayland socket. Check
`nvidia-smi`, `/dev/dri`, and `glxinfo -B` inside a GUI-opened container.

## RQt Tables

**Start RQt Tables** opens the RQt tables demo interface for the selected world
configuration.

**Stop RQt Tables** stops the RQt container. The output appears in the **RQt
Tables** tab.

In remote ROS master mode, RQt connects to the selected external ROS master.

## Open Terminal

**Open Terminal** launches an external terminal backed by a Docker container.
The terminal uses the selected image, workspace, ROS environment, and current
ROS master mode.

For a selected catkin workspace, `catkin build` works from its root. From
inside one of its package directories, `catkin build --this` builds that
package using the selected workspace, even if a stale `.catkin_tools` directory
exists below the workspace root.

**Close Terminal** closes the terminal process and stops its container.

The **Run as root** checkbox controls how a new terminal starts:

- Unchecked starts the terminal as a host-matching user when possible. This is
  the normal choice for editing mounted workspaces without changing file
  ownership.
- Checked starts the terminal as root inside the container.

A terminal log stream is shown in a closable **Terminal** tab.

## Auto Launch

**Auto Launch** starts or stops a configured sequence of GUI buttons. Use it
when you repeatedly need the same startup flow.

Use **Tools > Automation > Configure Auto Launch** to choose which buttons are
part of the sequence and when each step starts.

If no sequence exists, pressing Auto Launch offers to open the configuration
dialog.

## Recording Controls

Recording controls are hidden by default. Enable them from **View > Recording
Controls**.

- **Record Auto Launch** arms recording for the next Auto Launch run. It does
  not start recording immediately.
- **REC armed: press Auto Launch** means the GUI is waiting for you to start
  Auto Launch.
- **REC** means screen recording is active.
- **Recording Options** chooses the output folder, whether the folder should be
  remembered, the capture resolution, and whether an always-on-top Stop
  Recording window appears.
- The resolution selector chooses the screen size passed to the recorder.

Recording starts after the Auto Launch plan and window layout delay finish.

The **Configure Auto Launch** window has two modes. **Legacy** preserves the
fixed delays used by older profiles. **Advanced** gives every enabled process a
readiness duration and an optional dependency. A hard dependency waits for the
dependency's full duration. A soft dependency starts after the selected
percentage of that duration (for example, 30%). If a dependency is already
running when Auto Launch begins, it is treated as ready immediately. Advanced
profiles are saved with `mode: advanced` and a `processes` list; existing YAML
files without those fields continue to load in Legacy mode.
Unchecking an Advanced process excludes it from Auto Launch without discarding
its readiness duration or dependency settings; those values return when the
configuration window is reopened.

**Export...** in the configuration window writes the settings currently shown
(both modes, the recording delay, and the button text) to a YAML file of your
choice, for example to keep a backup or share it with another machine.
**Import...** loads such a file into the window; nothing is saved until you
press **Save**. Processes in the file that are not toolbar buttons of the
active workspace are skipped and listed in a message.

To measure a readiness duration experimentally, stop the process first and
click **Measure** in its Advanced row. The GUI launches that process immediately
and enables **Ready**. Click **Ready** as soon as the process is usable; the
elapsed time is rounded upward to one decimal place and copied into **Ready
after**. For example, 5.65 seconds becomes 5.7 seconds. The measured process is
left running so you can verify it normally.

When you start Auto Launch, an always-on-top progress window opens in the center
of the main window and shows the time remaining until the demo is ready. By
default, it uses normal progress bars. Set `launch_sequence.robot_race: true`
in your per-user `gui_settings.yaml` to replace the overall and per-process
bars with full-resolution robot animations. `ROBOT_RACE=true` remains
available as an environment override for terminal launches. You can change
the same per-user preference from **Tools > Automation > Use Robot Race
Animations**; the choice applies to the next Auto Launch run.
Waiting robots remain at the start, launching robots advance with their
readiness time, and ready or already-running robots finish their lane. Legacy
profiles show their configured launch times as milestones. Advanced profiles
use the full dependency and
readiness schedule, while processes that were already running add no wait. At
completion the window shows **Demo ready** and disappears automatically after
one second. When automatic window layout replay is enabled and a saved layout
exists, an **Arrange windows** bar shows the remaining wait; the default
`apply_delay_ms: auto` rearranges the windows one second after all processes
are ready. If a saved window is missing, layout matching stops after 30 seconds
instead of continuing in the background indefinitely. Launching another
managed window starts a fresh retry window.
Stopping Auto Launch dismisses the progress window immediately.
It stops when you uncheck **Record Auto Launch**, press **Stop Recording**, stop
Auto Launch, stop Roscore, or exit the GUI.

Each recording creates a timestamped session folder containing the MP4, the
`ffmpeg.log`, and saved GUI logs when recording ends through an Auto Launch
stop.

## World and Image selectors

The GUI remembers the selected world, image, script, recording resolution,
and additional toolbar argument values when it closes normally. On the next
launch it restores each value when that option is still available; otherwise
it uses the applicable configured default or first available option. The ROS
workspace selection is likewise retained in the workspace registry.

The **world_config** selector chooses the Gazebo world setting passed to the
simulation and RQt tables launch.

A workspace can ship option rules beside its button profile that make some
choices invalid in certain situations, for example allowing only
`cic_tables` while **Use remote ROS master** is on because the real robot
supports no other world. Invalid options are greyed out and their tooltip
says why; when a selection becomes invalid the GUI switches to the first
valid option and a popup tells you. Rules can also require a button to run
first: with the real robot, for example, every other button refuses to start
with a popup until **tables_demo_bringup** is running. Stopping is never
refused.

A loaded toolbar-button profile can expose up to three additional argument
combo boxes beside **world_config**. Their labels and selectable options come
from the profile, and each button profile entry controls which values apply to
that button. Selected values are appended to applicable commands as ROS
arguments (`name:=value`).
When the profile defines no generic argument names, these fields are not shown.
Use **Tools > Configure Toolbar Buttons**, select a button, and click
**Configure Arguments...** to edit each slot's argument name, comma-separated
combo options, and per-button applicability checkbox in a separate dialog.
This keeps the normal button table focused on commonly edited settings.

The **image** selector chooses the Docker image used by containers. Image labels
may include:

- **workspace match** when the image profile explicitly matches the active
  workspace.
- **image default only** when the active workspace is selected but the image
  uses only its built-in workspace.

The image tooltip shows the container user and whether host workspace mounting
is enabled.

Image discovery first applies `images.discovery_filters`, then removes refs
matching `images.blacklist` patterns. **Configure Image Filters** lets you edit
both lists and previews which local Docker images will be used, ignored, or
hidden.

When switching ROS 1 workspaces, the GUI selects the first available Docker
image marked as a **workspace match** for that workspace. If no match is
available, it falls back to the workspace's saved image or the GUI default.

Use **View > Refresh Images** after pulling, building, or removing Docker
images.

## Script Controls

Script controls are hidden by default. Enable them from **View > Script
Controls**.

- **Scripts** lists Python scripts available to run in Docker.
- **Refresh Scripts** rescans the scripts folder.
- **Run Script** starts the selected script.
- **Stop Script** stops the running script.

Script output appears in a custom tab. Scripts use the selected image,
workspace, and ROS master mode.

## Command Controls

Command controls are hidden by default. Enable them from **View > Command
Controls**.

- Enter a command and press Enter or **Run Command** to run it in a Docker
  command container.
- **Stop Command** stops the running custom command tab.
- **Run in current custom tab** reuses an idle custom tab when possible.

Custom commands use the selected image, workspace, world, and ROS master mode.

## Log tabs

Each main action has a tab that shows live output.

ROS messages identify the severity, local wall-clock time, and node, for
example `[WARN] [14:26:56] [/pose_selector]: Clearing planning scene`.
Warnings are shown in yellow even when the ROS process does not emit terminal
color codes.

- **Roscore** shows ROS master output.
- **Sim** shows simulator output.
- **Tables Demo** shows planning node output.
- **RViz** shows RViz output.
- **RQt Tables** shows RQt output.
- **Log** shows GUI status messages and commands the GUI runs.
- **Custom** tabs show script and custom command output.
- **Terminal** tabs show terminal container logs.
- **Loaded log** tabs show logs opened from saved HTML files.

Only custom, terminal, build, and loaded log tabs can be closed.
Press **Ctrl+W** to close the current tab when it has an **X** close button.

## Bottom log controls

- **Clear Current Tab** clears the visible tab.
- **Clear All Tabs** clears all visible log tabs.
- **Search**, **Prev**, and **Next** search within the current tab.

The tab search is separate from the documentation window search.

## Workspace menu

- **Configure Workspaces** opens the workspace manager.
- **Configure Workspace Matches** edits which Docker images are marked as
  valid for each ROS 1 workspace.
- **Build Active Workspace** builds the currently selected workspace, if one is
  active and the selected image supports host workspaces.

## Settings menu

- **Export All Settings...** saves a portable YAML file containing GUI settings,
  the workspace registry, and embedded workspace button and auto-launch
  profiles. Use this before moving to a new PC.
- **Import All Settings...** loads a portable settings file, asks for the new
  workspace master folder, and remaps imported workspaces under that folder.
  Stop running workspace processes before importing.
- **Show Configuration Paths** opens a separate window listing config and data
  paths managed by the GUI. The window notes that manual editing is not
  recommended. Use the row **Copy** and **Show** buttons for one path, or
  **Copy All Paths** and **Show All Contents** for the complete list. **Show**
  opens another window with readable file contents or directory listings.
- **Copy Full Reset Command...** is a development-only nuclear option. It
  displays a destructive warning, lists the per-user GUI config/data roots that
  would be deleted, and copies a terminal command to the clipboard. Nothing is
  deleted by the GUI. The command only deletes files after you paste it into a
  terminal and type `DELETE_MOBIPICK_GUI_CONFIG`. It removes GUI settings,
  workspace registry data, profiles, layouts, recordings, and custom image
  build contexts, but not Docker images, containers, ROS workspaces, the source
  checkout, or bundled defaults.

## Tools menu

- **Configure Toolbar Buttons** edits the active workspace toolbar button
  profile in a separate window. You can add, remove, reorder, and edit button
  labels, commands, stop commands, compose services, host execution, and
  tooltips. The **Stop Command** column appears when at least one button has
  **Host** checked; only Host rows can edit it. The stop command runs when you
  stop the button, after the GUI signals the original process. Check
  **Host** to run that button's command directly on the host instead of inside
  a Mobipick Docker container; saving writes this choice to the button profile.
  You can edit and save the profile while workspace processes are running.
  Changes apply to later launches. A running command keeps the launch and stop
  settings with which it started; a removed button remains available until its
  running command stops.
  Host commands do not check or automatically start Roscore. If the GUI's
  local Roscore is already running, the GUI gives each newly started Host
  command a matching `ROS_MASTER_URI` and a host-reachable `ROS_IP` so ROS
  nodes on the host and in Mobipick Labs can communicate. In remote ROS master
  mode the Host command gets the remote `ROS_MASTER_URI` and the host address
  that routes to that master instead.
  **Stop Roscore** leaves running Host commands alive; stop them with their own
  toolbar buttons when needed.
  **Sim** and **RViz** cannot be removed, but their commands can be changed.
  **Roscore** and **Terminal**
  are fixed buttons and cannot be edited from this profile. Use **Import from
  Workspace...** to choose individual buttons from another registered
  workspace. Imported buttons replace buttons with matching keys or are added
  when their keys are new; all unselected buttons remain unchanged. The copy
  is saved in the active workspace's own profile, so changing it later does
  not affect the source workspace. Use **Load Profile** or **Export Profile**
  to import or save a complete button configuration as one YAML file.
- **Reload Configuration** re-reads `gui_settings.yaml` and the active
  workspace toolbar button profile without restarting the GUI. Use it after
  editing a button command (for example adding a launch argument to the Sim
  button); the new command applies to the next press and running processes
  keep running.
- **Setup Wizard** opens the setup flow. It opens automatically at startup only
  when no Docker image matching the configured discovery filters is installed,
  including after setup was previously completed. If the configured default is
  absent but another matching image is installed, the GUI silently uses the
  active workspace's image, then a compatible host-user image, then another
  matching image for that session without changing the saved default. Missing
  optional host tools are listed in the GUI Log tab with the functionality they
  disable; set `MOBIPICK_GUI_SUPPRESS_OPTIONAL_DEPENDENCY_WARNINGS=1` to hide
  those warnings. The first page explains operating
  system and hardware compatibility, including the tested Ubuntu releases,
  the dedicated CUDA-capable NVIDIA GPU requirement, and reference GPU memory
  usage. The next page checks common Ubuntu host dependencies, and each wizard
  page title is numbered as **Step 1/N**, **Step 2/N**, and so on as you move
  through the flow. The host dependency
  page lets you choose packages, copy short, commented terminal commands to
  the clipboard, run them in a terminal, and click **Run Checks**. The commands
  plainly show each package update, installation, Docker repository, and
  user-group change without an interactive script around them. **Run Checks**
  opens a details
  window with a color-coded
  summary, each check's purpose, and the evidence used for the result. A
  black command section lists the exact Bash probe commands used for each host
  dependency. If a check still fails, **Open Bug Report** prepares the
  standard diagnostics so you can copy, save, email, or open a prefilled
  GitHub issue.
  On the **Setup Guide** page, **Install the app launcher and add it to the
  Ubuntu dock** creates a current-user application-menu entry and pins it to
  the Ubuntu/GNOME dock for one-click startup. It is selected by default,
  requires no `sudo`, preserves existing dock favorites, and is safe to run
  again. Clear it on non-GNOME desktops or when you manage launchers yourself.
  Independently of that choice, every GUI start also refreshes hidden
  per-user desktop entries for RViz, RQt, and Gazebo so that the dock,
  Alt-Tab switcher, and overview show their real icons instead of a generic
  gear for windows opened from the containers. The entries do not appear in
  the application grid and cannot start the tools by themselves; use the GUI
  buttons. Custom buttons and custom commands that run an rqt tool get the
  RQt icon as well, and so do the rqt panels started by the Sim button.
  Native-Wayland RQt windows are recognized by the `python3` identity that
  Qt reports for them, so other Python Qt programs without their own desktop
  entry may show the RQt icon while they run. Because GNOME only matches
  windows that come from the local host, the simulation and tool containers
  now run with the host's hostname; prompts inside container terminals show
  that name instead of a container ID.
  After the host dependencies, the **NVIDIA Container Toolkit** page links to
  NVIDIA's official installation guide and provides buttons to open the guide,
  copy its URL, and copy the terminal test command. Complete the guide before
  pressing **Run GPU Test**. The GUI runs an equivalent current-user Docker
  test without `sudo` so it cannot become stuck at an invisible password
  prompt. The test passes only when the container exits successfully and its
  output contains NVIDIA-SMI, driver, CUDA, and GPU-table evidence. If Docker
  still requires elevated access, use **Copy Test Command** and run the shown
  `sudo docker run --rm --runtime=nvidia --gpus all ubuntu nvidia-smi` command
  in a terminal, then retry after fixing current-user Docker access.
  The setup guide page includes **Learn More About These Choices**, which opens
  a separate explanation window for the public-image pull, host-user image
  build, source workspace install, and setup-completion checkboxes.
  Later pages pull public images on the host PC with streamed output, select
  defaults, build host-user images, and optionally clone and build
  `mobipick_labs` from source in a host-mounted workspace. The progress page
  labels selected automatic setup work as **Step 1/N**, **Step 2/N**, and so
  on, where **N** is the number of automatic steps that will run. The manual
  pull option pauses setup until you confirm that the command has finished.
  Each optional wizard page has **Skip This Step**.
  On the final summary page, **Test Simulation** launches the stock Mobipick
  Docker simulation and captures its combined terminal output. This basic test
  disables host workspace mounting, runs as root with the workspace baked into
  the image, and automatically prefers a locally available `mobipick_labs`
  image, then `x_mobipick_labs`. If neither family is available locally, it
  uses the wizard's default image. Press **I Can See the Simulation** to stop a
  successful smoke test. If Gazebo does not appear, press **I Cannot See the
  Simulation** to stop the test and open a privacy-scrubbed bug report with the
  captured output and relevant host, GPU, image, workspace, and GUI diagnostics.
  From there you can preview, copy, save, email, or create a prefilled GitHub
  issue.
  Docker cp paths are not configured by default; add them later from
  **Tools > Docker > Configure Docker cp Paths** when a workflow needs file
  copies.

### Docker

- **Manage Images** lists Docker images matching the GUI filters and lets you
  remove selected images.
- **Configure Image Filters** edits the local-image discovery filters and image
  refs or patterns that should be ignored by setup, image discovery, and Docker
  cp path setup.
- **Build Custom Image** opens the setup flow with the host-user development
  image option enabled.
- **Commit Current Tab** creates a Docker image from the running container
  behind the current tab. You can overwrite the selected tag, create a
  timestamped snapshot, or enter a custom tag.
- **Execute Docker cp** copies configured paths from the selected running
  container to the host.
- **Configure Docker cp Paths** edits host-to-container and container-to-host
  copy paths for the Docker image default workspace or a configured ROS
  workspace. **Add Row** opens a path dialog, checks that the selected host
  file exists, and lets you choose a setup container or workspace-match image
  for container-side path browsing. The container path field remains editable,
  so destinations that do not exist yet can still be entered manually.

The wizard shows the writable configuration and data paths it can affect. The
source install step creates
`<master folder>/clean_mobipick_labs_ws/src/mobipick_labs` by default, runs
`install-deps.sh` and `build.sh` inside Docker, and streams the full output in
the **Install Source** tab.

### Layout

- **Window Layout** opens a small always-on-top helper for saving current
  simulator and tool window positions.
- **Save Window State** records visible windows so the GUI can reapply the
  layout during later Auto Launch runs. The helper closes automatically after
  the layout is saved. On X11 sessions this needs `wmctrl` and `xprop`; on a
  GNOME Wayland session install the bundled GNOME Shell extension with
  `mobipick-labs-docker-gui --install-gnome-window-extension` and log out and
  back in. Repeat this after GUI upgrades so GNOME Shell loads extension
  changes. The setup wizard's dependency page detects when it is needed and
  includes that command in its generated Host Dependencies commands.

### Automation

- **Configure Auto Launch** opens the sequence editor for Auto Launch and the
  extra recording start delay.
- **Use Robot Race Animations** persistently replaces Auto Launch progress bars
  with synchronized robot animations for the current user. The choice applies
  to the next Auto Launch run.

### Remote Control

- **Enable Remote Control API** starts or stops an HTTP server that lets
  another computer, or an automation agent, press toolbar buttons, wait for
  launch events, read log tabs, and run commands in a ROS terminal inside the
  container. The address, port, and optional token come from the
  `remote_control` section of `gui_settings.yaml`. Enable the server for a
  launch with the `--remote-control`
  command line options, or the `MOBIPICK_GUI_REMOTE_*` environment variables.
- **Show Remote Control Info** displays the listening address, whether a token
  is required, and how many remote shells are open.
- Slow or unavailable Docker operations remain pending without freezing the
  window. The toolbar, repainting, and remote presence/events remain usable.
  Remote `GET /status`, `GET /buttons`, and `GET /tabs` calls use the most
  recent GUI snapshot and therefore keep answering even while Docker is hung.
  Requests that must touch a widget fail explicitly if the GUI cannot service
  them promptly.
- While the API is enabled the window icon glows: **light blue** when idle,
  **green** while a remote client has announced that it is working (`POST
  /presence`, `mobipick-labs-docker-gui-remote hello <name>`), and pulsing
  while a request is being served. The GUI log records which client is using
  the GUI and when it finished. A client has to renew its announcement at
  least every 10 minutes (30 minutes at most); when it stops doing so, or
  says goodbye, the GUI stops every button process, custom command, and
  remote shell that client started and writes what it stopped to the log.
- Remote shells appear as closable **Remote Shell N** tabs that mirror their
  output. Closing the tab closes the shell and its container. Anyone who can
  reach the port can run commands in the containers, so use a token on
  shared networks.
- A remote client can also open a shell **on the robot PC** over `ssh` instead
  of in a ROS container, to debug that machine (its processes, drivers, logs).
  It has to ask for it; ROS work stays in the container shell, which sees the
  same ROS master. Robot shells need **Use remote ROS master** and
  password-less ssh; the user and host come from the `ros` section of
  `gui_settings.yaml` (`robot_ssh_user`, `robot_ssh_host`; an empty host means
  the host of the remote `ROS_MASTER_URI`), and `robot_shell_by_default: true`
  makes the robot the default instead. The tab header and the GUI log name the
  machine each shell runs on.
- Remote clients can select the toolbar argument dropdowns (for example
  `anygrasp_mode`) and the world exactly as you would in the toolbar
  (`POST /args`, `mobipick-labs-docker-gui-remote set-args anygrasp_mode=real`,
  or `--arg name=value` on a button press); the choice shows in the toolbar
  and in the GUI log. They also see when a running process is expected to
  be ready: each button reports its **Configure Automation** startup estimate
  (`duration_seconds`) and a `button_ready` event fires when that time has
  elapsed, so an agent waits only as long as the estimate instead of a fixed
  timeout.
- The command line client is `mobipick-labs-docker-gui-remote`; run it with
  `--help` for the available actions.
- A Claude Code skill describing the curl workflow ships with the package;
  `mobipick-labs-docker-gui-remote skill --install ~/.claude/skills` copies
  it to another machine or project.

### Status

- **Update Status** refreshes the GUI view of Docker container status.

## View menu

- **Recording Controls** shows or hides Auto Launch recording controls.
- **Script Controls** shows or hides script controls.
- **Command Controls** shows or hides custom command controls.
- **Remote ROS Master** shows or hides remote ROS master controls.
- **Refresh Images** rescans installed Docker images.

## Logs menu

- **Save Current Log** saves the current tab as an HTML file.
- **Load Log** opens a saved HTML log into a new tab.
- **Save All Logs** saves every non-empty log tab to a selected folder.
- **Clear Current Tab** clears the visible tab.
- **Clear All Tabs** clears all log tabs.

## Help menu

- **Documentation** opens this user documentation window.
- **File Bug Report...** opens a diagnostic report builder. Choose which
  sections to include, add notes, and save or copy the report. The preview is
  automatically anonymized before it can be copied, saved, emailed, or sent to
  GitHub: local user and PC names, filesystem paths, network identifiers, and
  common secret values are removed. Stars retain only the length of masked
  numeric identifiers. Review the preview before submitting it because free-form
  text can still contain personal information the automatic checks do not
  recognize.
  If the report is too large for a broadly compatible GitHub issue link, the
  GUI opens the issue with the first part and explains how to use **Copy
  Remaining** to paste the rest into GitHub before submitting.
- **About** shows the GUI version, maintainer contact, GUI source code
  repository, and Mobipick Labs link.

## Workspace and image warnings

The GUI warns when a selected Docker image does not clearly match the active
workspace. Continue only when the image/workspace pair is intentional.
Use **Mark as Workspace Match** when the current image/workspace pair is known
to be valid. The GUI adds that workspace to the image profile's
`compatible_workspaces` setting and updates the image label to
**workspace match**.

If the selected image does not support host workspace mounting, commands run
against the workspace inside the image even if a host workspace is selected.

## Closing the GUI

When the GUI closes, it stops running Mobipick containers, stops recording if
needed, runs stop commands for toolbar buttons that are still active, revokes
temporary X11 access, runs cleanup, and then exits. Buttons that were not
started during the session do not have their stop commands run. Wait for the
shutdown dialog to finish before starting a new GUI session.

Container stops are slow by default: after sending SIGINT, the GUI waits for
ROS nodes to unregister cleanly from a surviving master. Select **Fast stop**
to skip that grace period. Stopping the local roscore always uses fast stop
because its registration database is being removed too. With a remote ROS
master, a fast stop enables **Clean stale ROS nodes** beside the master URI;
use it when unreachable registrations remain. The cleanup can unregister a
temporarily unavailable node, so review the warning before continuing.

The main window remembers its last normal size, position, and maximized state
and restores them the next time the GUI opens.
