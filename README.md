# Mobipick Labs Docker GUI

The Mobipick Labs Docker GUI is a PyQt5 desktop application that orchestrates the
Docker-based Mobipick Labs robotics simulation. Instead of manually invoking
`docker compose` yourself, you launch the `mobipick-labs-docker-gui` command (or
`python -m mobipick_gui`) and drive the bring-up,
monitoring, and shutdown of the simulation through an interactive interface.
The GUI reads the bundled configuration files, runs Docker commands on your
behalf, and streams live logs so you can follow what is happening in each
container.

<img src="doc/mobipick_labs_docker_gui.png" alt="mobipick tables sim and real" width="420">

## Repository layout

```
├── gui.py                  # Legacy CLI shim that forwards to the packaged entry point
├── mobipick_gui/
│   ├── resources/          # Bundled compose file, default configs, helper scripts
│   └── …                   # PyQt5 widgets, process orchestration, and helpers
├── MANIFEST.in             # Source distribution manifest
└── pyproject.toml          # Packaging metadata for PyPI distribution
```

The compose file is **not** intended to be executed directly. The GUI manages it
for you by spawning `docker compose` subprocesses, supervising their lifecycle,
and performing cleanup logic when you close the application.

## Prerequisites

* Ubuntu Linux (tested on versions 20.04, 22.04, and 24.04)
* Python 3.8+ with PyQt5 available (e.g. `pip install PyQt5`).
* Docker Engine and the Docker Compose plugin accessible to your user.
* Access to the Mobipick Labs image repository (for example
  `ozkrelo/mobipick_labs:noetic`).
* An X11 server that allows the containers to create GUI windows. The GUI
  issues the required `xhost` commands automatically when needed.

## Installation

- Install Docker Engine (`docker.io`) and ensure it is running.
```bash
sudo apt update && sudo apt install docker.io
sudo systemctl enable docker
sudo systemctl start docker
sudo apt install wmctrl graphviz
sudo apt install ffmpeg -y
```

- Configure Docker to run without sudo:

```bash
sudo usermod -aG docker $USER
```

Then log out and back in, or do:

```bash
newgrp docker
```

- Install the Mobipick Labs GUI from PyPI (this also installs the package dependencies):

```bash
pip install mobipick-labs-docker-gui
```

- Install the Docker Compose plugin and pull the Mobipick Labs image.

```bash
sudo apt install docker-compose-plugin
# Verify that the Compose plugin is available
docker compose version
# pull mobipick labs docker image from docker hub
docker pull ozkrelo/mobipick_labs:noetic
```

- Optional but strongly recommended if you have an NVIDIA graphics card: install [nvidia-docker2](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/nvidia-docker.html). After installation, restart Docker and test with:

```bash
sudo docker run --rm --runtime=nvidia --gpus all ubuntu nvidia-smi
```

Note: If you run Mobipick Labs on the CPU, the simulation can be slow.

## Launching the GUI

1. Clone the repository *or* install the package from PyPI.
1. Start the application:
   ```bash
   mobipick-labs-docker-gui
   ```
1. When the window opens, use the top row of buttons to bring up ROS core,
   start or stop the simulator, toggle RViz/RQt, or open a Docker-backed
   terminal. The GUI ensures the correct container sequence is followed.

You can interrupt the GUI with <kbd>Ctrl</kbd>+<kbd>C</kbd> in the launch
terminal; the application traps the signal, stops the running containers, and
then exits gracefully.

### ROS 1 workspaces on Ubuntu 24.04

ROS Noetic runs inside Docker; it does not need to be installed on the Ubuntu
24.04 host. The **ROS 1 workspace** selector at the top of the window controls
which host catkin workspace is bind-mounted into each container.

Use **Configure Workspaces** to:

* choose or create a master folder such as `~/ros_ws`;
* discover existing child workspaces that contain a `src/` directory;
* add a standalone workspace outside the master folder;
* create a new workspace without creating fake `devel` build outputs;
* declare which workspaces it extends;
* assign workspace-specific button and auto-launch YAML profiles;
* export all workspace settings and referenced profiles to one portable YAML
  file;
* import that file on another machine and remap workspaces under a selected
  master folder;
* set a workspace-specific simulator command;
* view the inheritance graph with Graphviz; and
* build the selected workspace with `catkin_make` inside the Docker image.

The registry is stored in
`~/.config/mobipick-labs-docker-gui/workspaces.yaml`. Set
`MOBIPICK_WORKSPACE_CONFIG` to use another file. The registry remembers the
configured workspaces and the last active workspace. Selecting **Docker image
default** skips the host workspace mount and uses the workspace bundled in the
image.

General per-user overrides live in
`~/.config/mobipick-labs-docker-gui/gui_settings.yaml`; set
`MOBIPICK_GUI_CONFIG` to use another file. Window layouts are stored beside that
file, while recordings default to
`~/.local/share/mobipick-labs-docker-gui/recordings`. These writable locations
keep installed package files immutable and prevent local state from entering a
wheel or source distribution.

Use **Export Settings** in the workspace manager to create a single portable
YAML file containing the workspace registry, user GUI overrides, and the
contents of referenced button and auto-launch profiles. **Import Settings**
asks for the destination machine's workspace master folder, restores profiles
under the user config directory, and rewrites workspace paths for that folder.
It does not copy or modify ROS workspace source trees.

The host master folder is mounted once at `~/ros_ws` inside Docker, regardless
of its host-side folder name. Workspace, devel, and source paths exposed to ROS
all use that canonical container root, preventing duplicate package discovery
through names such as `ros1_ws` and `ros_ws`.

Existing catkin-tools builds may contain absolute paths from an older host
location. The entrypoint creates temporary container-local compatibility
symlinks while sourcing those generated files, normalizes and deduplicates the
resulting ROS environment back to `~/ros_ws`, redirects legacy linked-devel
lookups to the real `.private` package outputs, and removes the aliases before
the requested command or terminal starts. If the selected workspace has not
been built, the entrypoint still sources `/opt/ros/noetic/setup.bash`, so
commands such as `roscore` remain available.

For development, `python gui.py` and an editable
`mobipick-labs-docker-gui` installation run this checkout. A command installed
by `pipx` under `~/.local/bin` runs the package in its pipx virtual environment,
which may be a different version unless that environment was installed from
the checkout.

### Command-line options

The CLI accepts a single verbosity switch that controls how much diagnostic
information the GUI prints to its log tabs and the launch terminal:

```bash
mobipick-labs-docker-gui --verbose        # Same as -v or --v
mobipick-labs-docker-gui -v 3             # Maximum verbosity
mobipick-labs-docker-gui -v 1             # Quietest mode (default)
```

You can also pass through any Qt-specific arguments (for example `-platform`)
after the GUI options; they are forwarded automatically to `QApplication`.

## Understanding the GUI workflow

* **Process supervision:** Each button spawns a `QProcess` that executes a
  Docker command (`docker compose up`, `docker compose exec`, `docker cp`, etc.).
  Environment variables from `mobipick_gui/resources/config/gui_settings.yaml` ensure the commands run
  with consistent settings (for example `COMPOSE_IGNORE_ORPHANS=1`).
* **State polling:** Timers defined in `config/gui_settings.yaml` periodically
  inspect Docker to reflect whether the ROS core, simulator, RViz, or RQt
  containers are alive before updating the button states.
* **Log streaming:** Every subprocess pipes its stdout/stderr into a dedicated
  tab, colourised via `mobipick_gui/ansi.py` so you can tail the container logs
  without leaving the GUI.
* **Graceful shutdown:** When you exit, the GUI stops active containers in a
  safe order, runs `clean.bash` (bundled in `mobipick_gui/resources/`) to remove
  temporary resources, and only then closes the window.

## Configuring the GUI

Bundled defaults live in `mobipick_gui/resources/config/`. Keep user overrides
in `~/.config/mobipick-labs-docker-gui/gui_settings.yaml`; the GUI merges that
file over the bundled defaults. Workspace locations and workspace-specific
profiles live in the per-user workspace registry described above.

* **`config/gui_settings.yaml`** – Controls UI behaviour such as window geometry
  and log styling, defines timer intervals, button colours, terminal launcher
  settings, and Docker environment variables. Most keys mirror the defaults
  declared in `mobipick_gui/config.py` so you can override just the values you
  need.
* **`config/worlds.yaml`** – Lists the world configurations that populate the
  drop-down selector when launching the simulator. Edit or append entries to
  expose additional Gazebo worlds shipped in your Mobipick Labs Docker image.
* **`config/docker_cp_image_tag.yaml`** – Declares optional `docker cp`
  synchronisation rules keyed by image name. Host-to-container copies run
  automatically after the container starts, while container-to-host copies are
  triggered by the "Execute Docker cp" button inside the GUI.

## Working with the compose file

Although `docker-compose.yml` lives in the repository, the GUI is responsible for
translating user actions into compose commands. Typical sequences are:

1. **ROS core toggle:** `docker compose up roscore` starts the lightweight
   orchestration container. The GUI remembers the container name and watches for
   it to become healthy before enabling the simulator button.
2. **Simulator toggle:** `docker compose up mobipick-run` launches the main
   Gazebo environment. When you stop it, the GUI optionally synchronises files
   defined in `docker_cp_image_tag.yaml` and then calls `docker compose stop`
   with a configurable timeout.
3. **Visualization tools:** RViz and RQt are launched with `docker compose run`
   so each tool receives its own tabbed log stream.

Because the GUI tracks container state, you should avoid running the compose
file manually in parallel—it can confuse the state machine and lead to orphaned
containers. If you need a manual clean slate, run `mobipick_gui/resources/clean.bash`
with the GUI closed to remove stopped containers and networks.

### Avoiding Git "dubious ownership" warnings in bind mounts

When Docker bind-mounts a host workspace into a container, Git 2.35+ refuses to
run if the repository is owned by a different UID/GID than the process inside
the container. The GUI now auto-detects your numeric UID/GID and user metadata
(with sudo-aware fallbacks) and exports them to every `docker compose` command
it spawns. Containers are still allowed to start as root—this keeps the original
entrypoint behaviour intact—but the GUI-provided terminal session immediately
drops privileges inside the container via
`/scripts_430ofkjl04fsw/enter_host_shell.py`. As a result, interactive
shells run with the same UID/GID as the bind-mounted repository, preventing Git
from flagging the workspace as "dubious". When you genuinely need a privileged
shell, tick the **Run as root** checkbox next to the **Open Terminal** button
before launching it. The GUI will export root credentials to Docker Compose and
skip the user drop so the terminal starts as the container's root user. You can
also make this behaviour the default by setting `terminal.drop_to_host_user` to
`false` in `config/gui_settings.yaml`. When you keep the checkbox cleared the
helper enables passwordless `sudo` for the synthesized host user, so you can run
administrative commands without re-owning files created later by non-privileged
processes.

If you invoke the compose file manually (outside the GUI), pass the same
variables explicitly so Docker uses your login credentials:

```bash
MOBIPICK_UID="$(id -u)" MOBIPICK_GID="$(id -g)" docker compose up
```

For ad-hoc `docker run` commands, either specify `--user "$(id -u):$(id -g)"` or
replicate the GUI behaviour with the `MOBIPICK_UID`/`MOBIPICK_GID` environment
pair. To obtain an interactive shell that mirrors the GUI behaviour, invoke the
helper directly:

```
docker compose run --rm \
  --env MOBIPICK_UID="$(id -u)" \
  --env MOBIPICK_GID="$(id -g)" \
  --env MOBIPICK_HOST_USER="$USER" \
  --env MOBIPICK_HOST_GROUP="$(id -gn)" \
  --env MOBIPICK_HOST_HOME="$HOME" \
  mobipick_cmd python3 /scripts_430ofkjl04fsw/enter_host_shell.py bash
```

The helper keeps the hinted `MOBIPICK_HOST_HOME` when possible, creating the
directory (and a matching passwd/group entry) if it does not already exist. When
the hinted home lacks a `.bashrc` but `/root/.bashrc` is available, the helper
creates a lightweight wrapper `~/.bashrc` that sources the container's default
profile. This preserves your user-specific writable home—editors such as `nano`
can persist history under `~/.local`—while still executing the image-provided
initialisation scripts automatically.

GUI terminals use a dedicated RC file that loads the image's
`~/scripts/permanent.sh` framework selectively. Programs such as `general` and
`git` remain enabled, so aliases like `..` and the Git-aware prompt are
available. The legacy `ros1` program is skipped because the GUI sources the
selected workspace and its underlays after loading the other shell helpers.

## Tips and troubleshooting

* Verify that Docker commands succeed from your shell before launching the GUI;
  it executes the same binaries with your current user.
* If the GUI cannot discover your Mobipick Labs image, adjust the
  `images.discovery_filters` list in `mobipick_gui/resources/config/gui_settings.yaml`.
* When experimenting with new Gazebo worlds or launch files, consider adding a
  custom button tab via `mobipick_gui/process_tab.py` so you can track logs in
  the same window.
* Logs are retained up to `log.max_block_count` lines per tab. Lower the value
  if you experience sluggishness on resource-constrained machines.
