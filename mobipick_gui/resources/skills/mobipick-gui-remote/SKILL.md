---
name: mobipick-gui-remote
description: Drive the Mobipick Labs Docker GUI over its HTTP remote-control API with curl - check what is already running, press toolbar buttons, wait for launches to settle, read process logs, run ROS 1 commands in a container shell, and open a shell on the real robot PC over ssh when that machine itself has to be debugged. Use when asked to start/stop the sim, roscore, RViz, RQt, tables demo or any toolbar button, to test ROS behaviour, to inspect output of processes the user launched from the GUI, or to look at the real robot's own processes, drivers or logs.
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
relaunch with `--remote-control` (env `MOBIPICK_GUI_REMOTE_CONTROL=1`). A
persisted `remote_control.enabled` value does not enable the API by itself.

**Without remote access there is no fallback.** Do not drive the GUI's
containers with `docker` commands (`exec`, `run`, `stop`, `compose`), do not
launch ROS yourself, and do not start a second GUI: a second instance shares
the Docker daemon and its exit cleanup stops the user's containers, and raw
`docker` commands bypass the GUI's log tabs, state tracking, ownership
cleanup, and audit trail. Stop and ask the user to enable the API.

**With remote access granted, curl is the way to work.** `docker` commands
are acceptable only for what the API cannot do (read-only inspection such as
`docker ps`, `docker stats`, `docker logs`, `docker inspect`); everything
that starts, stops, or enters a container goes through the API so the user
sees it in the GUI and it is cleaned up with your presence.

Every response is JSON with `ok` and `seq` (newest event number). Pipe
through `python3 -m json.tool` only when you need to read the whole thing;
prefer the narrow queries below to save tokens.

## 0. Say hello first, bye last

The GUI shows the user who is working on it: its window icon glows light
green while a client is present (light blue when idle) and the GUI log
records the name. Declare yourself before the first real request and
withdraw as the very last call (also when you give up or fail).

**Use a name that is unique to your session** (`claude-<4 hex chars>`,
e.g. `claude-51bf`), never a bare `claude`: several agents may drive the GUI
at the same time and a bye for a shared name withdraws the other session and
makes the GUI stop what *it* started. First `GET /presence` and, if someone
else is listed, tell the user before pressing buttons that could disturb
them; never withdraw or refresh a name that is not yours.

```bash
ME=claude-$(head -c2 /dev/urandom | xxd -p)
curl -s -X POST $GUI/presence -H 'Content-Type: application/json' -d "{\"name\":\"$ME\",\"note\":\"what you are doing\",\"ttl_s\":1800}"
# the FIRST reply carries "token": keep it, the bye needs it (a refresh does not repeat it)
# ... work ...
curl -s -X DELETE $GUI/presence -H 'Content-Type: application/json' -d "{\"name\":\"$ME\",\"token\":\"$TOKEN\"}"
```

Presence is kept alive by activity: every request you send refreshes it, as
does a shell command that is still running or a `follow=1` stream you keep
open, so no separate heartbeat process is needed (one that outlives you keeps
a dead agent "present"). Only `ttl_s` (default 600 s, at most 1800 s) of
complete idleness lapses it; ask for 1800 when a robot run will keep you
waiting. The GUI remembers every button, `/command` tab, shell and screen
recording you start. When your presence lapses, or you say bye, it **stops
all of them** and logs what it stopped. That is the safety net for a crashed
or forgetful agent, not a substitute for stopping things yourself: stop them,
then bye. Only when the user asked you to leave something running (e.g.
"start the sim for me") add `"keep":true` to the bye and say in your final
message what you left running. `GET /status` lists present `clients` and
`in_use`.

## 1. Always check what is already running first

The user usually launches things by hand. Never bring something up before
checking; if it is up, reuse it and read its existing output.

```bash
curl -s $GUI/buttons | python3 -c '
import json,sys
for b in json.load(sys.stdin)["buttons"]:
    print("%-12s %-7s ready=%-5s in=%-5s %-9s tab=%-16s args=%s  tip=%r" % (b["key"], b["state"], b["ready"], b["ready_in_s"], b["runs_on"], b["tab"], b["args"], b["tooltip"]))'
```

- `state`: `green` = running, `red` = stopped, `yellow` = starting or
  stopping (do not press it; wait), `grey` = unavailable in this mode.
- `ready` is `true` once a running button has been up for its configured
  startup estimate (`startup_seconds`, from the Auto Launch plan; `null`
  when the profile has none, then `ready` equals `running`); `ready_in_s`
  says how much of the estimate is left. **A green button with
  `ready: true` is up: use it right away, do not wait any further.**
- `tooltip` explains what the button does; `command` shows the configured
  command, `args` the toolbar argument values it currently receives (for
  example `{"anygrasp_mode": "mockup"}`) and `full_command` the two
  combined; `kind` is `builtin` or `command`; `runs_on` tells you whether
  a shell inside the container will see it directly.
- `tab` is the log tab key holding that process's output, present even when
  the process was started long before you connected.
- `curl -s $GUI/status` adds `roscore_running`, `sim_running`,
  `auto_launch_running`, active workspace, image, world, and `dialog`.

The status, button-list, tab-list, presence, and event endpoints do not wait
for the GUI thread. They keep answering from a recent state snapshot while a
Docker start/stop is slow or hung. An endpoint that must operate a widget
instead returns an explicit error promptly if Qt cannot accept the action;
report that error instead of silently retrying it in a tight loop.

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
     -d '{"wait_for":["button_ready"],"timeout":90}'
curl -s -X POST $GUI/buttons/rviz/stop -H 'Content-Type: application/json' -d '{}'
```

`accepted:false` with `reason` means already running / not running / busy /
disabled; read the reason and move on.

Buttons that take a toolbar argument (a dropdown next to the toolbar, e.g.
`anygrasp_mode: real|mockup`, or the world selector) get it from `GET /args`.
Select a value over the API instead of editing the profile YAML: either
`POST /args` or `args` in the press body, which is applied before the click.
Unknown names or values are rejected with HTTP 400 and nothing is pressed.
Each entry's `invalid` maps options the workspace's option rules forbid right
now (e.g. worlds the real robot cannot run in remote ROS master mode) to the
reason; selecting one is rejected the same way. The rules can also block a
button start (e.g. on the real robot nothing but `tables_demo_bringup` starts
until it runs): the press returns `accepted:false` with a reason beginning
`blocked by option rules:`. Start the named prerequisite, then retry.

```bash
curl -s $GUI/args | python3 -c 'import json,sys; [print(a["name"], a["value"], a["options"], a["buttons"]) for a in json.load(sys.stdin)["args"]]'
curl -s -X POST $GUI/buttons/anygrasp/start -H 'Content-Type: application/json' \
     -d '{"args":{"anygrasp_mode":"real"},"wait_for":["button_ready"],"timeout":60}'
curl -s -X POST $GUI/args -H 'Content-Type: application/json' -d '{"world":"moelk_tables"}'
```

The selection is visible in the toolbar and logged, so tell the user when
you leave a dropdown on a different value than you found it. Most container buttons auto-start
roscore when needed; if you started roscore, or a button auto-started it for
you, you stop it too (`/buttons/roscore/stop`) once everything else you
started is down.

How to stop each kind of thing you can start:

| You started it with | Stop it with |
| --- | --- |
| `POST /buttons/{key}/start` or `/click` | `POST /buttons/{key}/stop` |
| `POST /command` (a `customN` tab) | `POST /tabs/customN/stop` (the `tab` from the `/command` reply) |
| `POST /shell` | `DELETE /shell/{id}`; a running command first `POST /shell/{id}/interrupt` |
| anything, by its log tab | `POST /tabs/{key}/stop` works for all three |

Do not use `POST /command` for anything long-running (a `roslaunch`, a node,
a bag replay): prefer the toolbar button that exists for it, otherwise run it
in a shell session with `"wait": false` where `interrupt` and `DELETE` give
you full control. `/command` is for one-shot commands.

Treat the simulator as an expensive resource. Record which processes were
already running before the task. If you start **Sim** or **Auto Launch** for
inspection or testing, stop every process you started as soon as the required
observations are complete and before yielding back to the user. Do not leave
the simulator running while waiting for visual confirmation; stop it first,
then ask what the user saw. Never stop a process that was already running
unless the user explicitly requests it.

Wait exactly as long as the GUI's own estimate, no longer. Every button
carries the startup time the user configured for it (`startup_seconds`), and
`button_ready` fires for that button when the time has elapsed (immediately
when there is no estimate). Waiting for it in the press body is keyed to
that button, and if the button was already running and ready the call
returns at once with `already_ready: true` instead of blocking:

```bash
curl -s -X POST $GUI/buttons/sim/start -H 'Content-Type: application/json' \
     -d '{"wait_for":["button_ready","process_finished"],"timeout":90}'
# later, for a button someone else started, keyed so other buttons do not satisfy it:
curl -s -X POST $GUI/wait -H 'Content-Type: application/json' \
     -d '{"events":["button_ready"],"key":"sim","since":SEQ,"timeout":90}'
```

Include `process_finished` in `wait_for` so a launch that dies returns
early (the event carries the tab key and exit code) instead of running out
the timeout. Do **not** wait for `window_layout_applied` after a single
button press: that event only follows an Auto Launch. Prefer
`button_ready` over a fixed sleep, a large `timeout`, or polling the tab
for a "ready" line; read the tab only afterwards, to confirm the last
lines look healthy. When you pressed **Auto Launch**, the equivalent
signals are `auto_launch_complete` (every step reached its ready time) and
`window_layout_applied` (the saved layout was replayed):

```bash
curl -s -X POST $GUI/buttons/auto_launch/start -H 'Content-Type: application/json' \
     -d '{"wait_for":["window_layout_applied","auto_launch_complete"],"timeout":240}'
curl -s "$GUI/events?since=SEQ"          # history; add &follow=1&timeout=60 with curl -N to stream
```

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

## 5. ROS shell: in the container, or on the real robot

Open one session and reuse its id; it is a stateful bash connected to the
current ROS master. It appears in the GUI as a **Remote Shell N** tab so
the user can watch you.

```bash
curl -s -X POST $GUI/shell -H 'Content-Type: application/json' -d '{"name":"claude"}'   # blocks until ready, returns id
ID=1
curl -s -X POST $GUI/shell/$ID/exec -H 'Content-Type: application/json' \
     -d '{"command":"rostopic list | head -20"}'
```

**You choose the machine** with `"robot"`, and the reply says which you got
(`runs_on`, `target`), as does the startup output and the tab header:

| | `"robot": false` (the default) | `"robot": true` |
| --- | --- | --- |
| where | the ROS tool container | the robot PC, over `ssh` |
| for | **everything ROS**: `rostopic`, `rosnode`, `rosservice`, `rosparam`, `roslaunch`, `rosrun`, bags, the workspace's packages and scripts | **debugging the robot machine**: its processes (`ps`, `systemctl`, `journalctl`), disks, network, drivers, `~/.ros/log`, the robot's own workspace and launch files |
| has | this workspace chain sourced, the GUI's `ROS_MASTER_URI` | the robot user's login environment, the robot's files |

```bash
curl -s -X POST $GUI/shell -H 'Content-Type: application/json' -d '{"name":"claude"}'                # container: ROS work
curl -s -X POST $GUI/shell -H 'Content-Type: application/json' -d '{"name":"claude","robot":true}'    # robot PC: debug that machine
curl -s $GUI/status | python3 -c 'import json,sys; print(json.load(sys.stdin)["shell"])'              # default, robot_available, robot_target
```

**Default to the container.** A container shell reaches the same ROS master
as the robot, so a node that runs on the robot answers `rosnode`/`rostopic`
there just as well; reach for the robot shell only when the question is
about the robot machine itself (a node that will not start, a driver, a log
file, disk space, a service), or when the user asks for it.

`{"robot": true}` needs remote ROS master mode - the robot is the master -
and is refused with HTTP 400 otherwise (`/status` `shell.robot_available`
says so in advance). It also needs password-less ssh
(`ssh robot@mobipick-os-sensor`); without a key the session fails to start
with an ssh error instead of hanging on a password prompt. Everything below
(exec, output, interrupt, close) works the same for both kinds. A robot
shell runs as the robot user on the real machine: read and diagnose freely,
but do not restart the robot's services, kill its nodes or edit its files
unless the user asked for exactly that.

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

Close when finished: `curl -s -X DELETE $GUI/shell/$ID`. Closing a robot
shell ends the ssh connection and kills what it still ran there, so stop
long-running robot commands yourself rather than leaving them to the bye.
`GET /shell` lists every session with its `runs_on` and `target`, so a
hand-over check sees at a glance whether one of them is on the robot.

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
- `POST /reload` re-reads `gui_settings.yaml` and the workspace button
  profile. Use it only when a toolbar *command* has to change (for example
  adding `grasp_fix:=true` to the Sim button): edit the profile YAML,
  `POST /reload`, then press the button. This replaces asking the user to
  restart the GUI. Running processes are preserved, so stop a process first
  if you need it relaunched with the new command. Picking a value of an
  existing dropdown never needs this: use `/args` (section 3).
- `POST /quit` closes the GUI **and stops all its containers**. Only on
  explicit user request.
- The GUI log tab (`/tabs/log`) records every remote action, so the user
  can audit what you did.

## 7. Robot experiments go through the planner/executor agent

Do **not** drive the robot yourself for an experiment (`mobipick_api`,
action clients, `base.move`, teleporting models): you lack the operating
knowledge the planner has (retract the arm to `transport`/`home` before
navigating, perceive before picking, which poses work) and you would test
only a slice of the system. Give the goal to the GPT planner/executor agent
instead; that exercises the whole pipeline. For tests always select the cheap
`model_profile: deepseek-v4.1-flash` (it also serves the VLM verifier, see
`gpt_robot_demo` tab: `models={...}` and `VLM verifier ready: model=...`) and
the mockups (`disc_mode: mockup`, `anygrasp_mode: mockup`) unless the user
asks for the real modules.

```bash
curl -s -X POST $GUI/args -H 'Content-Type: application/json' \
     -d '{"model_profile":"deepseek-v4.1-flash","disc_mode":"mockup","anygrasp_mode":"mockup"}'
curl -s -X POST $GUI/buttons/auto_launch/start -H 'Content-Type: application/json' \
     -d '{"wait_for":["window_layout_applied","auto_launch_complete"],"timeout":300}'
```

**Auto Launch brings up everything the agents need** (roscore, sim, RViz,
DISC, LiteLLM, AnyGrasp, GPT Robot Demo, ...); afterwards check `/buttons`
for anything still red and the `gpt_robot_demo`, `disc`, `anygrasp` and
`litellm` tabs for `ready`/`ERROR` lines before sending a goal.

The goal is a `std_msgs/String` on `/recognized_speech` (what the GUI's
speech input publishes). The agent talks back on `/mobipick_gpt/gpt_debug`
(JSON per message; `kind` is `human`, `agent_run`, `env_snapshot`, `act`,
...; `act` lines read `11) pick(sugar box, table_1)` and later
`pick(sugar box, table_1) -> success`), `/mobipick_gpt/gpt_debug_reasoning`
and `/speak` (what it says to the human). The `gpt_robot_demo` tab holds the
same log. Use `mobipick_gpt/scripts/agent_experiment.py` from a remote shell
(section 5): it publishes the goal, logs everything with timestamps, takes
the evidence snapshots and returns when the agent has spoken and gone quiet.

```bash
# in a remote shell, after starting the recorder of section 8; wait:false, then follow the log
rosrun mobipick_gpt agent_experiment.py 'Grab the sugar box from table 1 and insert it into the box on table 2.' --log /home/oscar/ros_ws/claude_tmp/run.log
```

**Take what the agent says with a pinch of salt**: it sometimes hallucinates
success. Trust the tool result lines (`insert(klt_1) -> insert succeeded`)
over its prose, and verify the outcome yourself: in simulation
`/gazebo/model_states` (object position relative to the box/table), the pose
selector (`/pick_pose_selector_node/pose_selector_get_all`), the planning
scene, or a snapshot of the cameras. Report the agent's claim and your check
separately.

## 8. Evidence: photos and videos of every experiment

The user wants to *see* what happened, every time. Two recorders, both
holding only the moments the robot moves (idle phases such as the LLM
thinking are cut) and both producing a 4x version:

1. **Camera videos** (cannot be blocked by windows): the simulation has a
   fixed camera over the three tables, `/experiment_camera/image_raw`.
   Start the recorder in a remote shell *before* the goal
   (`wait:false`; add the robot camera as a second topic):

   ```bash
   roslaunch experiment_camera_recorder video_recorder.launch name:=<experiment> \
       image_topics:=/experiment_camera/image_raw,/mobipick/eef_main_cam/rgb/image_raw fps:=6
   ```

   It writes frames only while `/mobipick/cmd_vel` or an arm joint moves
   (`auto_pause`), `rosservice call /experiment_video_recorder/pause|resume`
   overrides that, `rostopic pub -1 /experiment_video_recorder/snapshot
   std_msgs/String "data: 'label'"` saves a JPEG of every camera
   (`agent_experiment.py` does this for each robot action and each spoken
   sentence), and `rosservice call /experiment_video_recorder/stop` closes
   the videos and writes `summary.json`. Output:
   `/data/experiment_recordings/<timestamp>_<experiment>/` with
   `<topic>.mp4`, `<topic>_4x.mp4`, `snapshots/NNN_<label>_<topic>.jpg`,
   `events.jsonl` (host and container see the same path).
2. **Screen recording** of the GUI/RViz/Gazebo windows through the API,
   started paused and resumed by `agent_experiment.py` whenever the camera
   recorder reports motion:

   ```bash
   curl -s -X POST $GUI/recording/start -H 'Content-Type: application/json' -d '{}'   # then /recording/pause
   ...
   curl -s -X POST $GUI/recording/stop  -H 'Content-Type: application/json' -d '{}'   # concat + 4x export
   ```

   `GET /recording` shows `segments`, `recorded_s`, `video_path` and
   `video_speedup_path`; the `recording_exported` event (and the GUI log)
   tells when the files are written. The folder is the GUI's
   `recording.output_dir` setting.

After every experiment: stop both recorders, pick the key snapshots (goal
sent, pick, transport, insert/place, the state after the agent's final
sentence; add the robot-camera frame when it shows the grasp), **send them
to the user in the chat** (`SendUserFile`), and state the folder of the
camera videos and the path of the screen recording (`_4x.mp4` first). Never
send every snapshot; four to six tell the story.

## Typical session

0. `GET /presence`, then `POST /presence` with your unique name (section 0)
   and keep the token from the reply.
1. `GET /status` and `/buttons`: note what is green and `ready`, read
   tooltips of anything unfamiliar. Write down what was already running:
   that is the state you hand back. A ready process needs no waiting.
2. Reuse running processes; `GET /tabs/<key>?tail=…&grep=…` for their
   history.
3. `start` only what is missing, with the `args` it needs, and wait for
   `button_ready` (plus `process_finished` to catch a crash); after an
   Auto Launch wait for `window_layout_applied`. For an experiment: mockups
   + `deepseek-v4.1-flash`, Auto Launch (section 7).
4. Open shells (section 5; the container unless the robot machine itself is
   the question, one per background process: a `wait:false` command keeps
   its shell busy, HTTP 409 for anything else), run checks with
   `stream:false`/`tail`/`grep`, follow long commands with `wait:false` plus
   `follow=1`. Start the recorders (section 8) before the goal, send the goal
   through `agent_experiment.py` (section 7), verify the outcome yourself.
5. Hand-over checklist, before your final message and before any message
   that asks the user to do something (restart the GUI, look at the screen,
   answer a question): run `GET /status` and compare `buttons` (`green` or
   `yellow`), `tabs` (`running`) and `shells` with what was running when you
   arrived. Stop every difference with the table in section 3, wait until
   the button is `red` or the tab is not `running`, and close your shells.
   Stop the camera recorder (`/experiment_video_recorder/stop`) and the
   screen recording (`POST /recording/stop`) first so the videos get
   exported, then send the evidence photos (section 8).
   Leave pre-existing processes alone unless the user asked you to stop them.
   If something you started cannot be stopped, the first line of your message
   says so and asks the user to stop it.
6. `DELETE /presence` with your name and token as the very last call. The
   reply's `clients` should list only other sessions (or nobody) and the GUI
   log will show whether it had to clean anything up after you; if it did,
   say so.
