#!/usr/bin/env bash
# Start the full Orin canonical telemetry stack:
#   1. canonical aggregator (PUB tcp://*:5590, REP tcp://*:5600, ingest 5570)
#   2. docking OAK camera adapter (192.168.50.21) -> MJPEG 1080p (supervised)
#   3. cutting OAK camera adapter (192.168.50.22) -> MJPEG 1080p (supervised)
#   4. dashboard (system python, Jetson hardware decode)
#
# The two OAK adapters start with a delay between them: connecting two OAK
# devices back-to-back triggers an intermittent "stack smashing" firmware
# crash, so we stagger the launches.  Each adapter also runs under
# ``--supervise`` so a native crash auto-restarts the feed.
#
# Usage:
#   ./run_all.sh               # launch in the current terminal group via tmux
#   ./run_all.sh foreground    # run in foreground (blocking; Ctrl-C stops all)
#
# Requirements:
#   - depthai-env python: /home/marcop/depthai-env/bin/python3 (steps 1-3)
#   - system python:      /usr/bin/python3 (step 4, has PySide2 + GStreamer)
#   - tmux (for the default background mode)

set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DAI_PY="/home/marcop/depthai-env/bin/python3"
SYS_PY="/usr/bin/python3"
DISPLAY_TARGET="${DISPLAY:-:1}"
OAK_START_DELAY_S="${OAK_START_DELAY_S:-5}"
CODEC="${CODEC:-jpeg}"

AGG_CMD="PYTHONPATH=canonical_zmq:. ${DAI_PY} -m canonical_zmq_publisher.main --ingest tcp://*:5570"
DOCK_CMD="PYTHONPATH=canonical_zmq:. ${DAI_PY} -m canonical_zmq_publisher.oak_capture --camera-role docking_camera --ingest-endpoint tcp://127.0.0.1:5570 --supervise --codec ${CODEC} --ev-compensation 0 --brightness 0 --contrast 0"
CUT_CMD="PYTHONPATH=canonical_zmq:. ${DAI_PY} -m canonical_zmq_publisher.oak_capture --camera-role cutting_camera --ingest-endpoint tcp://127.0.0.1:5570 --supervise --codec ${CODEC} --ev-compensation 0 --brightness 0 --contrast 0"
# MALLOC_ARENA_MAX=2 caps glibc at two malloc arenas (default is cores*8=32 on
# aarch64), which keeps the per-frame 6 MB numpy buffers from fragmenting the
# process address space into hundreds of mmap'd arenas.  This is the dominant
# contributor to the dashboard's ~3 GB steady-state RSS.
DASH_CMD="DISPLAY=${DISPLAY_TARGET} MALLOC_ARENA_MAX=2 PYTHONPATH=harvester_dashboard ${SYS_PY} -m harvester_dashboard.main --pub tcp://127.0.0.1:5590 --status tcp://127.0.0.1:5600"

log() { printf '\033[1;32m[run_all]\033[0m %s\n' "$*"; }

# Stop any already-running instance of this stack first so we never double-bind
# the canonical 5590/5600/5570 ports.
"$ROOT/stop_all.sh" 2>/dev/null || true

if [[ "${1:-}" == "foreground" ]]; then
  log "Launching in foreground (Ctrl-C to stop all)."
  log "aggregator:  ${AGG_CMD}"
  log "docking:     ${DOCK_CMD}"
  log "cutting:     ${CUT_CMD}"
  log "dashboard:   ${DASH_CMD}"

  # Start the three background services, then run the dashboard in the
  # foreground so Ctrl-C tears everything down.
  ( cd "$ROOT" && eval "$AGG_CMD" ) &
  AGG_PID=$!
  ( cd "$ROOT" && eval "$DOCK_CMD" ) &
  DOCK_PID=$!
  # Stagger the second OAK device to avoid the back-to-back connect crash.
  sleep "$OAK_START_DELAY_S"
  ( cd "$ROOT" && eval "$CUT_CMD" ) &
  CUT_PID=$!

  trap 'log "Stopping..."; kill $AGG_PID $DOCK_PID $CUT_PID 2>/dev/null || true; wait 2>/dev/null || true' EXIT INT TERM

  cd "$ROOT"
  eval "$DASH_CMD"
else
  command -v tmux >/dev/null 2>&1 || { log "tmux not found; use './run_all.sh foreground'"; exit 1; }
  log "Launching in tmux session 'harvest' (attach with: tmux attach -t harvest)."

  tmux kill-session -t harvest 2>/dev/null || true

  tmux new-session -d -s harvest -n agg  "cd '$ROOT' && $AGG_CMD"
  tmux new-window   -t harvest -n docking "cd '$ROOT' && $DOCK_CMD"
  # Stagger the cutting camera: connect the second OAK after a delay so the
  # two devices never negotiate XLink back-to-back (firmware crash trigger).
  tmux new-window   -t harvest -n cutting "cd '$ROOT' && sleep $OAK_START_DELAY_S && $CUT_CMD"
  tmux new-window   -t harvest -n dash    "cd '$ROOT' && $DASH_CMD"

  log "All 4 components started in tmux windows: agg, docking, cutting, dash."
  log "Attach:        tmux attach -t harvest"
  log "Stop:          ./stop_all.sh"
fi
