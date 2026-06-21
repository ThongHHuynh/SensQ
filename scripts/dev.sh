#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BACKEND_DIR="$ROOT_DIR/backend"
UI_DIR="$ROOT_DIR/ui"
ROS_WS_DIR="${ROS_WORKSPACE:-$ROOT_DIR/ros2_ws}"

BACKEND_HOST="${BACKEND_HOST:-0.0.0.0}"
BACKEND_PORT="${BACKEND_PORT:-8000}"
FRONTEND_HOST="${FRONTEND_HOST:-0.0.0.0}"
FRONTEND_PORT="${FRONTEND_PORT:-5200}"
ROS_DISTRO="${ROS_DISTRO:-jazzy}"
PUBLIC_HOST="${PUBLIC_HOST:-localhost}"

API_BASE_URL="${VITE_API_BASE_URL:-http://$PUBLIC_HOST:$BACKEND_PORT}"
WS_BASE_URL="${VITE_WS_BASE_URL:-ws://$PUBLIC_HOST:$BACKEND_PORT}"
DATABASE_URL="${DATABASE_URL:-postgresql+asyncpg://sensq:sensq@localhost:5432/sensq}"

BACKEND_PID=""
FRONTEND_PID=""

log() {
  printf '[sensq] %s\n' "$1"
}

cleanup() {
  if [[ -z "$FRONTEND_PID" && -z "$BACKEND_PID" ]]; then
    return
  fi

  log "Stopping services..."
  if [[ -n "$FRONTEND_PID" ]] && kill -0 "$FRONTEND_PID" 2>/dev/null; then
    kill "$FRONTEND_PID" 2>/dev/null || true
  fi
  if [[ -n "$BACKEND_PID" ]] && kill -0 "$BACKEND_PID" 2>/dev/null; then
    kill "$BACKEND_PID" 2>/dev/null || true
  fi
}

require_file() {
  local path="$1"
  local message="$2"

  if [[ ! -e "$path" ]]; then
    printf '%s\n' "$message" >&2
    exit 1
  fi
}

require_command() {
  local command="$1"

  if ! command -v "$command" >/dev/null 2>&1; then
    printf 'Missing required command: %s\n' "$command" >&2
    exit 1
  fi
}

trap cleanup EXIT INT TERM

require_command npm
require_command node
require_command python3

require_free_port() {
  local port="$1"
  local label="$2"

  if ! python3 - "$port" <<'PY' >/dev/null 2>&1
import socket
import sys

port = int(sys.argv[1])
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
try:
    sock.bind(("0.0.0.0", port))
finally:
    sock.close()
PY
  then
    cat >&2 <<EOF
$label port $port is already in use.

Find the process:

  sudo ss -ltnp | grep ':$port'

Stop it:

  sudo fuser -k ${port}/tcp

Or run SensQ on another port:

  BACKEND_PORT=8001 FRONTEND_PORT=5201 ./scripts/dev.sh

EOF
    exit 1
  fi
}

NODE_MAJOR="$(node -p 'process.versions.node.split(".")[0]')"
if (( NODE_MAJOR < 18 )); then
  cat >&2 <<EOF
Node.js $NODE_MAJOR is too old for the SensQ UI.

Install Node.js 18 or newer, then reinstall frontend dependencies:

  cd $UI_DIR
  rm -rf node_modules package-lock.json
  npm install

Recommended options:

  nvm install 20
  nvm use 20

or install Node.js 20 from NodeSource.

EOF
  exit 1
fi

require_file "$BACKEND_DIR/.venv/bin/activate" "Backend virtualenv missing. Run: cd backend && python3 -m venv .venv && source .venv/bin/activate && pip install -r requirements.txt"
require_file "$UI_DIR/node_modules" "Frontend dependencies missing. Run: cd ui && npm install"

if ! python3 - <<'PY' >/dev/null 2>&1
import socket

with socket.create_connection(("localhost", 5432), timeout=1.5):
    pass
PY
then
  cat >&2 <<'EOF'
PostgreSQL is not accepting connections on localhost:5432.

Start PostgreSQL, then create the SensQ database if needed:

  sudo systemctl start postgresql
  sudo -u postgres psql
  CREATE USER sensq WITH PASSWORD 'sensq';
  CREATE DATABASE sensq OWNER sensq;
  \q

EOF
  exit 1
fi

set +e
DB_CHECK_OUTPUT="$(
DATABASE_URL="$DATABASE_URL" "$BACKEND_DIR/.venv/bin/python" - <<'PY' 2>&1
import asyncio
import os

import asyncpg


async def main() -> None:
    url = os.environ["DATABASE_URL"].replace("postgresql+asyncpg://", "postgresql://", 1)
    conn = await asyncpg.connect(url)
    await conn.close()


asyncio.run(main())
PY
)"
DB_CHECK_STATUS="$?"
set -e

if [[ "$DB_CHECK_STATUS" -ne 0 ]]; then
  cat >&2 <<EOF
PostgreSQL is running, but the backend cannot log in with:

  $DATABASE_URL

Backend database check error:

$DB_CHECK_OUTPUT

Fix one of these:

  1. Reset the database password to match the default:

     sudo -u postgres psql
     ALTER USER sensq WITH PASSWORD 'sensq';
     ALTER DATABASE sensq OWNER TO sensq;
     \\q

  2. Reinstall backend Python dependencies:

     cd $BACKEND_DIR
     source .venv/bin/activate
     pip install --upgrade pip setuptools wheel
     pip install --force-reinstall --no-cache-dir asyncpg
     pip install --force-reinstall --no-cache-dir greenlet pydantic-core
     pip install -r requirements.txt

  3. Or run with your real database URL:

     DATABASE_URL="postgresql+asyncpg://USER:PASSWORD@localhost:5432/DB_NAME" ./scripts/dev.sh

EOF
  exit 1
fi

require_free_port "$BACKEND_PORT" "Backend"
require_free_port "$FRONTEND_PORT" "Frontend"

log "Backend: http://$PUBLIC_HOST:$BACKEND_PORT"
log "Frontend: http://$PUBLIC_HOST:$FRONTEND_PORT"
log "Database: $DATABASE_URL"

(
  cd "$BACKEND_DIR"
  # shellcheck disable=SC1091
  source "$BACKEND_DIR/.venv/bin/activate"

  if [[ -f "/opt/ros/$ROS_DISTRO/setup.bash" ]]; then
    set +u
    # shellcheck disable=SC1091
    source "/opt/ros/$ROS_DISTRO/setup.bash"
    set -u
  else
    log "Warning: /opt/ros/$ROS_DISTRO/setup.bash not found. ROS imports may be unavailable."
  fi

  if [[ -f "$ROS_WS_DIR/install/setup.bash" ]]; then
    set +u
    # shellcheck disable=SC1091
    source "$ROS_WS_DIR/install/setup.bash"
    set -u
  else
    log "Warning: $ROS_WS_DIR/install/setup.bash not found. Build ros2_ws before launching robot hardware."
  fi

  export ROS_WORKSPACE="$ROS_WS_DIR"
  export DATABASE_URL="$DATABASE_URL"
  exec uvicorn app.main:app --reload --host "$BACKEND_HOST" --port "$BACKEND_PORT"
) &
BACKEND_PID="$!"

(
  cd "$UI_DIR"
  export VITE_API_BASE_URL="$API_BASE_URL"
  export VITE_WS_BASE_URL="$WS_BASE_URL"
  exec npm run dev -- --host "$FRONTEND_HOST" --port "$FRONTEND_PORT" --strictPort
) &
FRONTEND_PID="$!"

wait -n "$BACKEND_PID" "$FRONTEND_PID"
