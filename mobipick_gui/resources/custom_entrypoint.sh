#!/bin/bash
set -Eeuo pipefail

log() {
  echo "[entrypoint] $*" >&2
}

trap 'log "error while running: ${BASH_COMMAND} (line ${LINENO})"' ERR

log "starting entrypoint"
log "current user: $(id -u):$(id -g)"
log "working directory: $(pwd)"
log "command: $*"

if [[ -f /usr/share/gazebo/setup.sh ]]; then
  log "sourcing /usr/share/gazebo/setup.sh"
  # setup gazebo environment
  source "/usr/share/gazebo/setup.sh" --
else
  log "warning: /usr/share/gazebo/setup.sh not found"
fi

if [[ -f /root/catkin_ws/devel/setup.bash ]]; then
  log "sourcing /root/catkin_ws/devel/setup.bash"
  # setup ros environment
  source "/root/catkin_ws/devel/setup.bash" --
else
  log "warning: /root/catkin_ws/devel/setup.bash not found"
fi

log "executing: $*"
exec "$@"
