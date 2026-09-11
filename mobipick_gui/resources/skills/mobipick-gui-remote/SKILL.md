---
name: mobipick-gui-remote
description: Drive the Mobipick Labs Docker GUI over its HTTP remote-control API with curl - check what is already running, press toolbar buttons, wait for launches to settle, read process logs, and run ROS 1 commands in a container shell. Use when asked to start/stop the sim, roscore, RViz, RQt, tables demo or any toolbar button, to test ROS behaviour, or to inspect output of processes the user launched from the GUI.
---

# Mobipick Labs GUI remote control

The GUI (`mobipick-labs-docker-gui`) normally runs **on the host**. Most
things it launches are **Docker containers** (roscore, sim, RViz, RQt,
terminals, custom buttons), but a toolbar button can also run a process
**on the host** (`runs_on: "host"` in `/buttons`). Either way the GUI owns
the process and its log tab, and this API is the only way to reach both.

Only `curl` is needed. No MCP server, no Python client.

## Setup

```bash
GUI=${MOBIPICK_GUI_REMOTE_URL:-http://127.0.0.1:8765}
# add -H "Authorization: Bearer $MOBIPICK_GUI_REMOTE_TOKEN" to every curl when a token is set
curl -s $GUI/            # endpoint list, if you forget anything below
```

If the connection is refused, the API is off (it is off by default and the
menu toggle is not remembered across relaunches). Stop and ask the user to
enable it with **Tools > Remote Control > Enable Remote Control API**, or to
relaunch with `--remote-control` (env `MOBIPICK_GUI_REMOTE_CONTROL=1`), or
to set `remote_control.enabled: true` in
`~/.config/mobipick-labs-docker-gui/gui_settings.yaml` so it stays on.
There is no fallback: do not run `docker exec` against the GUI's containers
and do not start a second GUI. A second instance shares the Docker daemon
and its exit cleanup stops the user's containers, and `docker exec` bypasses
the GUI's log tabs, state tracking, and audit trail.

Every response is JSON with `ok` and `seq` (newest event number). Pipe
through `python3 -m json.tool` only when you need to read the whole thing;
prefer the narrow queries below to save tokens.

## 1. Always check what is already running first

The user usually launches things by hand. Never bring something up before
checking; if it is up, reuse it and read its existing output.

```bash
curl -s $GUI/buttons | python3 -c '
import json,sys
for b in json.load(sys.stdin)["buttons"]:
    print("%-12s %-7s %-9s tab=%-16s %r  tip=%r" % (b["key"], b["state"], b["runs_on"], b["tab"], b["text"], b["tooltip"]))'
```

- `state`: `green` = running, `red` = stopped, `yellow` = starting or
  stopping (do not press it; wait), `grey` = unavailable in this mode.
- `tooltip` explains what the button does; `command` shows the exact
  command it runs; `kind` is `builtin` or `command`; `runs_on` tells you
  whether a shell inside the container will see it directly.
- `tab` is the log tab key holding that process's output, present even when
  the process was started long before you connected.
- `curl -s $GUI/status` adds `roscore_running`, `sim_running`,
  `auto_launch_running`, active workspace, image, world, and `dialog`.

`curl -s $GUI/tabs` lists all log tabs with `running`; keys usually match
button keys (`roscore`, `sim`, `rviz`, `rqt`, `tables`, custom keys,
`log` for the GUI's own log, `customN`, `terminalN`, `terminal-remoteN`).

## 2. Read output of a process (yours or the user's)

```bash
curl -s "$GUI/tabs/sim?tail=40"                       # last 40 lines
curl -sG "$GUI/tabs/sim" --data-urlencode "tail=20" --data-urlencode "grep=ERROR|WARN|Exception"   # -G + --data-urlencode: regex/spaces are URL-encoded
curl -s "$GUI/tabs/log?tail=30"                       # GUI log: commands it ran, cleanup, warnings
```

Use `tail` and `grep` aggressively; a sim tab can hold 20k lines. Any `grep`
with `|` or spaces must be URL-encoded (`curl -G --data-urlencode`). Print
`lines` only:

```bash
curl -sG "$GUI/tabs/sim" --data-urlencode tail=40 --data-urlencode grep=ERROR | python3 -c 'import json,sys; print("\n".join(json.load(sys.stdin)["lines"]))'
```

## 3. Start or stop things

Use `start`/`stop` (idempotent) rather than `click` (toggle) so a re-run
never accidentally stops something.

```bash
curl -s -X POST $GUI/buttons/roscore/start -H 'Content-Type: application/json' -d '{}'
curl -s -X POST $GUI/buttons/sim/start -H 'Content-Type: application/json' \
     -d '{"wait_for":["button_state"],"timeout":60}'
curl -s -X POST $GUI/buttons/rviz/stop -H 'Content-Type: application/json' -d '{}'
```

`accepted:false` with `reason` means already running / not running / busy /
disabled; read the reason and move on. Most container buttons auto-start
roscore when needed.

Wait until a launch has settled. The reliable signal is the GUI replaying
the saved window layout (`window_layout_applied`); `auto_launch_complete`
fires when every step of an Auto Launch reached its ready time:

```bash
curl -s -X POST $GUI/buttons/auto_launch/start -H 'Content-Type: application/json' \
     -d '{"wait_for":["window_layout_applied","auto_launch_complete"],"timeout":240}'
# or later, using the seq from a previous response so nothing is missed:
curl -s -X POST $GUI/wait -H 'Content-Type: application/json' \
     -d '{"events":["window_layout_applied"],"since":SEQ,"timeout":240}'
curl -s "$GUI/events?since=SEQ"          # history; add &follow=1&timeout=60 with curl -N to stream
```

For a single process without a layout, poll `state` until it is `green`
and then check its tab for the line you expect (for example `grep=ready`
or the launch's final message). `process_finished` events report a tab
key and exit code when something dies.

## 4. Modal dialogs

If a request returns HTTP 504 with a `dialog` entry, or `status.dialog` is
not null, the GUI is waiting on a dialog (workspace/image mismatch warning,
"Please wait", confirmations). Inspect and answer it:

```bash
curl -s $GUI/dialogs
curl -s -X POST $GUI/dialogs/dismiss -H 'Content-Type: application/json' -d '{"button":"Continue"}'
```

Use the exact button text from `buttons`, or `accept` / `reject`. Ask the
user before answering anything destructive.

## 5. ROS shell inside the container

Open one session and reuse its id; it is a stateful bash in the ROS tool
container with the ROS environment and workspace sourced, connected to the
current ROS master. It appears in the GUI as a **Remote Shell N** tab so
the user can watch you.

```bash
curl -s -X POST $GUI/shell -H 'Content-Type: application/json' -d '{"name":"claude"}'   # blocks until ready, returns id
ID=1
curl -s -X POST $GUI/shell/$ID/exec -H 'Content-Type: application/json' \
     -d '{"command":"rostopic list | head -20"}'
```

Controlling how much comes back (this is what saves tokens):

- `"stream": false` returns only exit code and `line_count`; the output
  stays buffered. Fetch selectively later:
  `curl -s "$GUI/shell/$ID/output?command=CMD_ID&tail=30&grep=ERROR"`.
- `"tail": N` and `"grep": "RE"` filter the returned lines; `"max_lines"`
  caps them (default 400).
- Set the session default once with
  `POST /shell/$ID/settings -d '{"stream":false}'` and opt in per command.
- `"timeout": s` (default 60) bounds the wait; a `timed_out:true` response
  means the command is still running and the session is busy.

Long-running commands (roslaunch, rosrun nodes, rosbag play):

```bash
curl -s -X POST $GUI/shell/$ID/exec -H 'Content-Type: application/json' \
     -d '{"command":"roslaunch my_pkg thing.launch","wait":false}'
curl -sNG "$GUI/shell/$ID/output" -d follow=1 -d timeout=90 --data-urlencode "grep=ERROR|started|ready"   # NDJSON lines until done or timeout
curl -s -X POST $GUI/shell/$ID/interrupt -H 'Content-Type: application/json' -d '{}'    # SIGINT, like Ctrl-C
```

A busy session rejects new `exec` with HTTP 409; wait, poll `/output`, or
interrupt. Commands read stdin from `/dev/null`, so interactive prompts fail
fast instead of hanging. `cd`, `source`, and exported variables persist.
Prefer the GUI buttons for things the user has buttons for, and the shell
for tests, `rostopic`/`rosservice`/`rosparam` queries, and scripts.

Close when finished: `curl -s -X DELETE $GUI/shell/$ID`.

The shell's startup output names the sourced workspace, its underlay chain,
`ROS_MASTER_URI`, and `ROS_IP`; `/status` reports the same active workspace.
The sim runs from that same chain, so a package found in an underlay (for
example a world file under `clean_mobipick_labs_ws`) does not mean the sim
used a different workspace.

When testing whether a topic delivers anything, run the plain
`timeout 15 rostopic echo -n1 /topic` and filter with the API's `grep`/`tail`
afterwards; a shell-side `| grep`/`| head` filter can hide the output and make
a working topic look silent.

If a subscriber gets no data (`rostopic echo` prints nothing) while services
work, the publisher is the problem, not the shell: check
`rosnode ping /gazebo`, `rostopic info /clock` (a live publisher URI must be
listed), and whether the sim is paused
(`rosservice call /gazebo/get_physics_properties | grep pause`). A crashed
gzserver keeps its stale registrations at the master, so services from other
nodes keep answering while every Gazebo topic stays silent. Read
`/tabs/sim` with `grep=died|Assertion` before blaming networking. Verified
working from a fresh remote shell: `/clock`, `/gazebo/model_states`, joint
states, and `rospy.wait_for_message`.

## 6. Other

- `POST /command -d '{"command":"..."}'` runs text through the GUI's own
  Custom Command box (a `customN` tab, roscore auto-started).
- `POST /quit` closes the GUI **and stops all its containers**. Only on
  explicit user request.
- The GUI log tab (`/tabs/log`) records every remote action, so the user
  can audit what you did.

## Typical session

1. `GET /status` and `/buttons`: note what is green, read tooltips of
   anything unfamiliar.
2. Reuse running processes; `GET /tabs/<key>?tail=…&grep=…` for their
   history.
3. `start` only what is missing, wait for `window_layout_applied` or a
   green state plus an expected log line.
4. Open one shell, run checks with `stream:false`/`tail`/`grep`, follow
   long commands with `--no-wait` plus `follow=1`.
5. Report findings with the relevant log lines; stop only what you started
   unless told otherwise; close your shell.
