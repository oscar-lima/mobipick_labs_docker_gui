#!/bin/bash

# docker cleanup with safety checks and clear logging

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DEFAULT_COMPOSE_FILE="$SCRIPT_DIR/docker-compose.yml"
if [ -z "${COMPOSE_FILE:-}" ] && [ -f "$DEFAULT_COMPOSE_FILE" ]; then
  export COMPOSE_FILE="$DEFAULT_COMPOSE_FILE"
fi
if [ -z "${COMPOSE_PROJECT_NAME:-}" ]; then
  export COMPOSE_PROJECT_NAME="mobipick"
fi
if [ -z "${COMPOSE_IGNORE_ORPHANS:-}" ]; then
  export COMPOSE_IGNORE_ORPHANS=1
fi

log() { echo "[clean.bash] $*"; }

command_exists() { command -v "$1" >/dev/null 2>&1; }

reachable_docker() { docker info >/dev/null 2>&1; }

compose_config_ok() { docker compose config >/dev/null 2>&1; }

compose_has_running_or_stopped() {
  # any containers associated with the current compose project
  test -n "$(docker compose ps -q 2>/dev/null)"
}

network_exists() {
  docker network inspect "$1" >/dev/null 2>&1
}

network_in_use() {
  # returns true if any container is attached
  local net="$1"
  # count containers attached to the network
  local count
  count="$(docker network inspect "$net" -f '{{ len .Containers }}' 2>/dev/null || echo 0)"
  [ "${count:-0}" -gt 0 ]
}

log "starting docker cleanup"

# check docker cli presence
if ! command_exists docker; then
  log "docker cli not found, doing nothing because docker is not installed"
  exit 0
fi

# check docker daemon reachability
if ! reachable_docker; then
  log "docker daemon unreachable, doing nothing because cannot talk to docker"
  exit 0
fi

# check docker compose availability
if ! docker compose version >/dev/null 2>&1; then
  log "docker compose not available, skipping compose operations"
else
  log 'evaluating "docker compose down --remove-orphans"'

  if compose_config_ok; then
    if compose_has_running_or_stopped; then
      log 'compose project detected with containers, executing "docker compose down --remove-orphans"'
      docker compose down --remove-orphans
      rc=$?
      if [ $rc -eq 0 ]; then
        log "compose down finished successfully"
      else
        log "compose down returned exit code $rc"
      fi
    else
      log "compose config present but no containers found for this project, doing nothing because there is nothing to stop"
    fi
  else
    log "no valid compose config in this directory, doing nothing because there is no compose project here"
  fi
fi

# handle network removal
NET_NAME="mobipick"
log "evaluating removal of network \"$NET_NAME\""

if network_exists "$NET_NAME"; then
  if network_in_use "$NET_NAME"; then
    log "network \"$NET_NAME\" exists but is in use, doing nothing because containers are attached"
  else
    log "network \"$NET_NAME\" exists and is unused, executing \"docker network rm $NET_NAME\""
    docker network rm "$NET_NAME"
    rc=$?
    if [ $rc -eq 0 ]; then
      log "network \"$NET_NAME\" removed successfully"
    else
      log "failed to remove network \"$NET_NAME\" with exit code $rc"
    fi
  fi
else
  log "network \"$NET_NAME\" does not exist, doing nothing because there is nothing to remove"
fi

log "docker cleanup complete"
