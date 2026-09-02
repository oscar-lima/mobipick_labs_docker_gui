# ROS workspace templating tools

> Deprecated: the GUI workspace manager now stores workspace selection in
> `~/.config/mobipick-labs-docker-gui/workspaces.yaml` and passes paths to
> Docker at runtime. These scripts remain only to migrate existing local
> setups; do not use them for new configurations.

## Quick use

1. Run the workspace chooser
   ```bash
   python3 choose_ws_and_render.py
````

2. Enter the number of the workspace you want
3. The tool updates `select_ros1_ws.sh` and renders templates into the parent directory

NOTE: Don't manually edit `select_ros1_ws.sh` unless you are adding a new workspace.

## Adding a workspace

1. Edit select_ros1_ws.sh by adding it to the list.
2. Edit config.yml with the command you wan't to run when clicking bringup sim button.

The current GUI can also edit the active toolbar button profile from
**Tools > Configure Toolbar Buttons**. This is preferred for adding, removing,
or changing workspace-specific command buttons after migration.

---

## Contents

* `choose_ws_and_render.py`
  Interactive selector. Lets you pick a ROS workspace, updates `select_ros1_ws.sh`, then runs the renderer.

* `ros_ws_template_renderer.py`
  Reads the selected workspace and configuration, renders all files inside `jinja_templates`.

* `select_ros1_ws.sh`
  Small shell file that defines `DESIRED_NUMBER` and the `rws_list`.
  The Python helper updates only the number.

* `config.yml`
  Mapping from workspace name to a command list.
  The renderer picks the correct command list based on the chosen workspace.

* `jinja_templates/`
  Directory with Jinja template files.
  All templates here get rendered one directory up from where scripts are run.
  Its legacy GUI settings template keeps automatic window-layout replay at one
  second after all Auto Launch processes are ready. Its compose template does
  not statically mount X11 or `/run/user`; the current GUI adds the detected
  X11/XWayland or Wayland socket to each container invocation. The legacy
  entrypoint template also creates the private runtime directory and links a
  forwarded Wayland socket into it when present.

---

## Detailed workflow

### 1. Workspace selection

`select_ros1_ws.sh` contains an array like:

```bash
rws_list=(
    $HOME/catkin_ws/src
    $HOME/ros_ws/mobipick_labs_ws/src
    $HOME/ros_ws/rae_upom_mobipick_ws/src
)
```

Run

```bash
python3 choose_ws_and_render.py
```

The script:

* Parses the entries in the array
* Shows them with their index
* You type an index
* It rewrites `DESIRED_NUMBER=<index>` inside `select_ros1_ws.sh`
* Then automatically calls `ros_ws_template_renderer.py`

No manual editing of the shell file.

### 2. Rendering logic

`ros_ws_template_renderer.py` does the following:

* Sources `select_ros1_ws.sh` to get the chosen ROS workspace
* Extracts the path segment between `$HOME` and `src`
  Example: `$HOME/ros_ws/mobipick_labs_ws/src`
  yields `ros_ws/mobipick_labs_ws`
* Loads `config.yml` and selects the right command entry
* Renders all files inside `jinja_templates` using

  * `ros1_workspace`
  * `command` (list of strings)
  * `command_str` (same command list but concatenated string)
* Writes output files to the parent directory, preserving names and execute bits

### 3. Configuration

Example `config.yml`:

```yaml
commands:
  default: ["roslaunch", "tables_demo_bringup", "demo_sim.launch"]
  catkin_ws: ["roslaunch", "tables_demo_bringup", "demo_sim.launch"]
  mobipick_labs_ws: ["roslaunch", "tables_demo_bringup", "demo_sim.launch"]
  rae_upom_mobipick_ws: ["roslaunch", "rqt_simple_launcher_config", "collect_objs_moelk.launch"]
```

### 4. Template usage

Inside template files you can write for example:

```jinja
source "/root/{{ ros1_workspace }}/devel/setup.bash"
command: {{ command }}
```

or get command as a string:

```jinja
command: "{{ command_str }}"
```

### 5. Output

All rendered files appear **one directory up** from where you run the script, with the same relative paths and names as in the templates folder.

---

## Notes

* Do not edit `DESIRED_NUMBER` directly. Use the chooser tool
* Template folder is entirely processed each time
* Config file determines which ROS launch command is used per workspace
