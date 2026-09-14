# Repository Guidelines

## Project Structure & Module Organization
The PyQt5 application lives in `mobipick_gui/`, with `main_window.py` orchestrating the window and `process_tab.py` managing the log consoles. Docker orchestration assets reside under `mobipick_gui/resources/`; edit `resources/config/gui_settings.yaml` for UI behaviour, `resources/config/worlds.yaml` for world choices, and drop helper scripts in `resources/scripts/` when a container needs extra setup. The legacy `gui.py` script simply forwards to the packaged entry point, while `doc/` stores imagery. Keep new modules small, cohesive, and colocated beside related widgets or helpers.

## Build, Test, and Development Commands
- `python -m pip install -e .` installs an editable environment with the GUI dependencies.
- `mobipick-labs-docker-gui --verbose 2` launches the interface with extra logging; `python -m mobipick_gui` is equivalent when debugging.
- `python -m build` produces distributable wheels and source archives; run it before publishing.
Cache Docker images locally (`docker pull ozkrelo/x_mobipick_labs:noetic-v1.1`) so the simulator starts promptly during development reviews.

## Coding Style & Naming Conventions
Follow PEP 8 with four-space indents and keep functions under 80 columns where practical. Qt derived classes stay in CamelCase, while helpers, signals, and module-level constants use snake_case and UPPER_SNAKE. Continue annotating public APIs with type hints and short docstrings explaining side effects. Prefer logging to the GUI log tab instead of raw `print` to retain colour formatting.

## GUI Layout Guidelines
Dialogs and tool windows must use responsive horizontal sizing for long user-editable values such as file paths, commands, Docker image tags, and workspace paths. Form rows with `QLineEdit` path or command fields must give those fields an expanding horizontal size policy and a sensible initial/minimum width based on realistic content, not a narrow default dialog width. For `QTableWidget` and `QTreeWidget` views, assign at least one content-heavy column a `QHeaderView.Stretch` resize mode instead of leaving important fields at fixed widths; resize-to-contents columns should be limited to compact labels, buttons, status, and short identifiers. When adding or adjusting a window, verify that widening the window makes the relevant path or command fields wider without requiring the user to drag column separators manually.

All desktop-window changes must remain compatible with both Xorg and Wayland.
Inspect `XDG_SESSION_TYPE`, with `WAYLAND_DISPLAY` and `DISPLAY` as fallbacks,
before choosing window-management behavior. Reuse the session helpers in
`mobipick_gui.window_control` and the cached session flags on `MainWindow`;
do not issue X11-only or unsupported Wayland activation/placement requests
unconditionally.

Container display routing is a user-space compatibility invariant. In
`display.mode: auto`, expose both usable transports and let ordinary Qt tools
follow the native session. ROS Noetic's Gazebo and RViz builds are a special
case: their OGRE 1.9 renderer creates an X11 GLX child window and cannot use a
native Wayland parent. On Wayland sessions with XWayland available, their
launches must override `QT_QPA_PLATFORM=xcb`. On NVIDIA hosts they must also
set `__NV_PRIME_RENDER_OFFLOAD=1` and `__GLX_VENDOR_LIBRARY_NAME=nvidia` so
XWayland selects accelerated NVIDIA GLX; losing that selection can leave
Gazebo's menus and sidebar visible while its 3D viewport is black. Do not
force these OGRE applications onto native Wayland: RViz reports `Invalid
parentWindowHandle` and Gazebo aborts.

Xorg sessions must continue to select X11/XCB, and `display.mode: x11` must
remain an explicit compatibility override. Native Wayland Qt applications on
NVIDIA require `qtwayland5` and a container `libwayland-client` that exports
`wl_proxy_marshal_flags` (Wayland 1.20 or newer), because current NVIDIA
Container Toolkit releases inject `libnvidia-egl-wayland2.so.1`. Non-root
containers must also receive the numeric groups owning the host's
`/dev/dri/renderD*` and `/dev/dri/card*` nodes; do not assume the container's
`render` and `video` group IDs match the host. Keep the ABI compatibility in
the Noetic base image hierarchy and the device-group and OGRE GLX mapping in
the GUI's Compose launch path. Preserve regression tests for both session
types whenever changing display detection, base-image graphics libraries, or
container display environment variables.

### Gazebo and RViz display regression lessons

Treat the Qt window, OGRE child window, GL loader, and GPU device access as
separate layers. A visible menu bar or `Using Wayland-EGL` only proves that Qt
created its top-level window; it does not prove that OGRE can create or render
the embedded 3D viewport. Classify failures by their evidence before changing
display routing:

- `wl_proxy_marshal_flags` with exit 127 is a Wayland client ABI mismatch.
- `/dev/dri/renderD*` permission errors mean the host graphics GIDs were not
  propagated into the container.
- `Invalid parentWindowHandle` or a Gazebo abort after Wayland-EGL means the
  GLX-only OGRE build received a native Wayland parent.
- Normal menus with a black 3D viewport on NVIDIA XWayland mean accelerated
  NVIDIA GLX selection is missing or broken; retain the PRIME and GLX vendor
  overrides on the Gazebo and RViz launch paths.

Do not declare a display fix complete from package presence, symbol checks,
unit tests, or successful Qt startup alone. After display-related changes,
run both Gazebo and RViz in an actual Wayland session, confirm that each 3D
viewport renders, inspect the process logs for GLX/EGL errors, and repeat the
smoke check on Xorg. Stop the simulator immediately after the observation.
Keep this manual runtime check alongside the automated transport, environment,
Compose-group, and privilege-drop regression tests.

The proven NVIDIA Wayland baseline is: expose both Wayland and XWayland,
launch ordinary compatible Qt tools natively, launch Gazebo and RViz with
XCB plus NVIDIA PRIME/GLX selection, and add the host DRI device GIDs to the
container. Preserve that complete combination; changing one part requires the
full runtime check above before calling the user-space behavior fixed.

## Testing Guidelines
Add regression tests under a top-level `tests/` package (create it if missing) and mirror the package path (e.g., `tests/mobipick_gui/test_process_tab.py`). Use `pytest` plus `pytest-qt` for widget exercises, and stub Docker subprocesses with `unittest.mock` so tests run without containers. Name tests after the scenario (`test_roscore_button_disables_when_process_stops`) and include a smoke test that launches the application headless to verify resource loading.

## Documentation Updates
Consider whether every considerable code, configuration, or workflow change also needs documentation updates. Use `README.md` for developer and maintainer documentation, and `mobipick_gui/resources/gui_user_documentation.md` for user-facing GUI behavior rendered from **Help > Documentation**. If a change affects deprecated private workspace templating under `mobipick_gui/resources/private/`, update `mobipick_gui/resources/private/README.md` as well. Skip documentation edits for trivial internal changes that do not alter behavior, setup, configuration, or maintenance expectations.

## Commit & Pull Request Guidelines
Git history favours concise, imperative subject lines such as `use gpu to run the simulation inside the container`; stay under 72 characters and focus each commit on one concern. End the final response with one suggested commit message for the current user request only when this turn leaves actual code, documentation, configuration, or test changes to commit. Do not suggest a commit message after inspection-only, explanation-only, or clean-worktree turns. Pull requests should describe the user impact, note Docker or configuration changes, link relevant issues, and attach GUI screenshots when adjusting visuals or button flows.
<!-- To restore session-wide commit message suggestions, replace the previous sentence with: If multiple changes are made in the same conversation thread, keep updating that suggestion as one squashed commit message that covers the accumulated work rather than listing separate per-turn messages. -->

## Configuration & Runtime Tips
Treat `mobipick_gui/resources/config/` as the single source of truth for defaults. When testing overrides, point `MOBIPICK_GUI_DATA_ROOT` at a writable copy and document any new keys in `config.py`. Never commit credentials or local Docker contexts, and keep `docker-compose.yml` changes backward compatible for existing lab setups.
