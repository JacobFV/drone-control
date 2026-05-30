#!/usr/bin/env bash
# Record the control-station UI off-screen (no X display needed) and encode each
# phase to mp4. Robust on shared/headless machines. Usage: tools/record_ui_offscreen.sh [scene] [drones]
set -euo pipefail
cd "$(dirname "$0")/.."
ROOT="$(pwd)"

SCENE="${1:-warehouse}"
DRONES="${2:-4}"
FPS=30
PORT=8799
OUT="$ROOT/film/assets/clips/ui"
FRAMES="$OUT/_frames"
PY="$ROOT/.venv/bin/python"
EL="$ROOT/node_modules/.bin/electron"
mkdir -p "$OUT"
rm -rf "$FRAMES"

cleanup() {
  [ -n "${SVC_PID:-}" ] && kill "$SVC_PID" 2>/dev/null || true
}
trap cleanup EXIT

echo "== service on :$PORT =="
PYTHONPATH="$ROOT" "$PY" -m drone_control.service --host 127.0.0.1 --port "$PORT" > /tmp/drone_svc.log 2>&1 &
SVC_PID=$!
WS=""
for _ in $(seq 1 80); do
  WS=$(grep -m1 '^WS_READY ' /tmp/drone_svc.log | sed 's/^WS_READY //' | tr -d '\r' || true)
  grep -q '^SERVICE_READY ' /tmp/drone_svc.log && break
  sleep 0.25
done
echo "ws=$WS"

echo "== offscreen electron recording ($SCENE, $DRONES drones) =="
REC_OUT="$FRAMES" REC_SCENE="$SCENE" REC_DRONES="$DRONES" REC_FPS="$FPS" \
  DRONE_SERVICE_URL="http://127.0.0.1:$PORT" DRONE_WS_URL="$WS" \
  "$EL" --no-sandbox "$ROOT/electron/record.js" 2>&1 | grep -E "PHASE|REC_DONE|REC_ERROR" || true

echo "== encoding phases =="
for dir in "$FRAMES"/*/; do
  [ -d "$dir" ] || continue
  phase="$(basename "$dir")"
  n=$(ls "$dir"/f*.jpg 2>/dev/null | wc -l)
  [ "$n" -lt 5 ] && { echo "  skip $phase ($n frames)"; continue; }
  out="$OUT/ui_${SCENE}_${phase}.mp4"
  # Offscreen frames can be odd-sized (e.g. 1919x1079); crop to even for libx264.
  ffmpeg -y -hide_banner -loglevel error -framerate "$FPS" -i "$dir/f%05d.jpg" \
    -vf "crop=trunc(iw/2)*2:trunc(ih/2)*2" \
    -c:v libx264 -preset veryfast -pix_fmt yuv420p -crf 18 -movflags +faststart "$out" \
    && echo "  wrote $out ($n frames)" || echo "  FAILED $out"
done
[ -z "${KEEP_FRAMES:-}" ] && rm -rf "$FRAMES"
echo "== done =="
