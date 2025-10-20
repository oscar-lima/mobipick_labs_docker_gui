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
```

- Configure Docker to run without sudo:

```bash
sudo usermod -aG docker $USER
```

Then log out and back in.

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
docker run --rm --gpus all nvidia/cuda:12.4.1-base nvidia-smi
```

Note: If you run Mobipick Labs on the CPU, the simulation will be very slow.

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

All customisation lives in the `mobipick_gui/resources/config/` directory. You
can copy these files and adapt them to your workflow. When running from an
installed package the directory is read-only; export the environment variable
`MOBIPICK_GUI_DATA_ROOT` and point it at a writable copy of the resources if you
need to override the defaults.

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
