#!/usr/bin/env bash
# Record the actual control-station UI: the full tile wall, then a series of
# single-tile "zoom in and talk about it" shots. Drives a real sim session via
# the service API and a real Electron window via a command file (no clicks).
#
# Usage: tools/record_ui.sh [scene] [drones]
set -euo pipefail
cd "$(dirname "$0")/.."
ROOT="$(pwd)"

SCENE="${1:-warehouse}"
DRONES="${2:-4}"
DISPLAY_ID="${DISPLAY:-:1}"
PORT=8799
OUT="$ROOT/film/assets/clips/ui"
CMD="/tmp/drone_rec_cmd"
mkdir -p "$OUT"
: > "$CMD"

PY="$ROOT/.venv/bin/python"
EL="$ROOT/node_modules/.bin/electron"

cleanup() {
  [ -n "${SESSION_STARTED:-}" ] && curl -s -X POST "http://127.0.0.1:$PORT/api/session/stop" >/dev/null 2>&1 || true
  [ -n "${EL_PID:-}" ] && kill "$EL_PID" 2>/dev/null || true
  [ -n "${SVC_PID:-}" ] && kill "$SVC_PID" 2>/dev/null || true
}
trap cleanup EXIT

echo "== starting service on :$PORT =="
PYTHONPATH="$ROOT" "$PY" -m drone_control.service --host 127.0.0.1 --port "$PORT" \
  > /tmp/drone_svc.log 2>&1 &
SVC_PID=$!
WS=""
for _ in $(seq 1 60); do
  WS=$(grep -m1 '^WS_READY ' /tmp/drone_svc.log | sed 's/^WS_READY //' | tr -d '\r' || true)
  grep -q '^SERVICE_READY ' /tmp/drone_svc.log && break
  sleep 0.25
done
echo "service ws=$WS"

echo "== launching electron (frameless full-screen) =="
DISPLAY="$DISPLAY_ID" DRONE_REC=1 DRONE_REC_CMD="$CMD" \
  DRONE_SERVICE_URL="http://127.0.0.1:$PORT" DRONE_WS_URL="$WS" \
  "$EL" --no-sandbox "$ROOT" > /tmp/drone_electron.log 2>&1 &
EL_PID=$!
sleep 6

echo "== stopping any auto-restored session =="
curl -s -X POST "http://127.0.0.1:$PORT/api/session/stop" >/dev/null 2>&1 || true
sleep 1

echo "== starting sim session: $SCENE / $DRONES drones =="
curl -s -X POST "http://127.0.0.1:$PORT/api/session/start" \
  -H 'Content-Type: application/json' \
  -d "{\"kind\":\"sim\",\"name\":\"film\",\"options\":{\"numDrones\":$DRONES,\"task\":\"goto\",\"scene\":\"$SCENE\",\"cameraModel\":\"ov2640\",\"cameraNoise\":\"medium\",\"maxSpeed\":false,\"record\":true}}" \
  >/tmp/drone_session.json
SESSION_STARTED=1
cat /tmp/drone_session.json; echo
# Make sure we start from the full wall (no tile maximized).
echo "window.location.hash=''" > "$CMD"
sleep 8   # let tiles populate (frames, trajectories, point cloud)

grab() {  # grab <seconds> <outfile>
  ffmpeg -y -hide_banner -loglevel error \
    -f x11grab -draw_mouse 0 -framerate 30 -video_size 1920x1080 -i "${DISPLAY_ID}.0" \
    -t "$1" -c:v libx264 -preset veryfast -pix_fmt yuv420p -crf 18 \
    -movflags +faststart "$2"
  echo "  wrote $2"
}

maximize() { echo "window.location.hash='#max=$1'" > "$CMD"; sleep 1.5; }
restore()  { echo "window.location.hash=''" > "$CMD"; sleep 1.0; }

echo "== shot 1: full tile wall =="
grab 24 "$OUT/ui_${SCENE}_wall.mp4"

for TILE in omniscient "camera-sim-0" estimated-trajectory trajectory pointcloud "seg-sim-0" "depth-sim-0" world-seg; do
  echo "== zoom: $TILE =="
  maximize "$TILE"
  grab 14 "$OUT/ui_${SCENE}_tile_${TILE//[^a-zA-Z0-9]/_}.mp4"
  restore
done

echo "== done =="
