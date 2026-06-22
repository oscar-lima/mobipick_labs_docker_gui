# Mobipick Labs Docker GUI User Documentation

This guide explains the controls in the Mobipick Labs Docker GUI from a user
perspective. It describes what each button and menu option does while you are
running the simulator, ROS tools, scripts, terminals, recordings, and logs.

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

## ROS 1 Workspace selector

The ROS 1 workspace selector controls which workspace is mounted into Docker
when the selected image supports host workspaces.

- **Docker image default** uses the workspace already inside the Docker image.
- A listed workspace mounts that host workspace into the containers.
- The tooltip tells you whether the workspace is built and whether the selected
  image can mount it.
- The GUI asks for confirmation before switching, because switching clears the
  current tabs and log output.

Use **Configure Workspaces** to add, create, import, export, build, and activate
workspaces.

## Configure Workspaces

The workspace manager is used to prepare ROS 1 workspaces for the GUI.

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
  commands, and custom commands connect to an external ROS master.
- **ROS_MASTER_URI** selects the external ROS 1 master, for example
  `http://mobipick-os-sensor:11311`.
- Local Roscore and the local simulation are disabled while remote mode is on.

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

Recording starts after the Auto Launch timeline and window layout delay finish.
It stops when you uncheck **Record Auto Launch**, press **Stop Recording**, stop
Auto Launch, stop Roscore, or exit the GUI.

Each recording creates a timestamped session folder containing the MP4, the
`ffmpeg.log`, and saved GUI logs when recording ends through an Auto Launch
stop.

## World and Image selectors

The **world_config** selector chooses the Gazebo world setting passed to the
simulation and RQt tables launch.

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

## Tools menu

- **Configure Toolbar Buttons** edits the active workspace toolbar button
  profile in a separate window. You can add, remove, reorder, and edit button
  labels, commands, compose services, and tooltips. **Sim** and **RViz** cannot
  be removed, but their commands can be changed. **Roscore** and **Terminal**
  are fixed buttons and cannot be edited from this profile. Use **Load
  Profile** or **Export Profile** in the editor to import or save a complete
  button configuration as one YAML file.

### Docker

- **Manage Images** lists Docker images matching the GUI filters and lets you
  remove selected images.
- **Setup Wizard** opens the setup flow for pulling public images, selecting
  defaults, building host-user images, and optionally cloning and building
  `mobipick_labs` from source in a host-mounted workspace. Each optional wizard
  page has **Skip This Step**.
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
  the layout is saved.

### Automation

- **Configure Auto Launch** opens the sequence editor for Auto Launch and the
  extra recording start delay.

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
  sections to include, add notes, and save or copy the report.
- **About** shows the GUI version, maintainer contact, and Mobipick Labs link.

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
needed, revokes temporary X11 access, runs cleanup, and then exits. Wait for the
shutdown dialog to finish before starting a new GUI session.

The main window remembers its last normal size, position, and maximized state
and restores them the next time the GUI opens.
