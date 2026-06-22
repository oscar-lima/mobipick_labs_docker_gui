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
Dialogs and tool windows must use responsive horizontal sizing for long user-editable values such as file paths, commands, Docker image tags, and workspace paths. For `QTableWidget` and `QTreeWidget` views, assign at least one content-heavy column a `QHeaderView.Stretch` resize mode instead of leaving important fields at fixed widths; resize-to-contents columns should be limited to compact labels, buttons, status, and short identifiers. When adding or adjusting a window, verify that widening the window makes the relevant path or command fields wider without requiring the user to drag column separators manually.

## Testing Guidelines
Add regression tests under a top-level `tests/` package (create it if missing) and mirror the package path (e.g., `tests/mobipick_gui/test_process_tab.py`). Use `pytest` plus `pytest-qt` for widget exercises, and stub Docker subprocesses with `unittest.mock` so tests run without containers. Name tests after the scenario (`test_roscore_button_disables_when_process_stops`) and include a smoke test that launches the application headless to verify resource loading.

## Documentation Updates
Consider whether every considerable code, configuration, or workflow change also needs documentation updates. Use `README.md` for developer and maintainer documentation, and `mobipick_gui/resources/gui_user_documentation.md` for user-facing GUI behavior rendered from **Help > Documentation**. If a change affects deprecated private workspace templating under `mobipick_gui/resources/private/`, update `mobipick_gui/resources/private/README.md` as well. Skip documentation edits for trivial internal changes that do not alter behavior, setup, configuration, or maintenance expectations.

## Commit & Pull Request Guidelines
Git history favours concise, imperative subject lines such as `use gpu to run the simulation inside the container`; stay under 72 characters and focus each commit on one concern. After code or documentation changes, end the final response with a suggested commit message. If multiple changes are made in the same conversation thread, keep updating that suggestion as one squashed commit message that covers the accumulated work rather than listing separate per-turn messages. Pull requests should describe the user impact, note Docker or configuration changes, link relevant issues, and attach GUI screenshots when adjusting visuals or button flows.

## Configuration & Runtime Tips
Treat `mobipick_gui/resources/config/` as the single source of truth for defaults. When testing overrides, point `MOBIPICK_GUI_DATA_ROOT` at a writable copy and document any new keys in `config.py`. Never commit credentials or local Docker contexts, and keep `docker-compose.yml` changes backward compatible for existing lab setups.
