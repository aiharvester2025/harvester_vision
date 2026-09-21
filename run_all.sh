#!/usr/bin/env bash
# Start the full Orin canonical telemetry stack:
#   1. canonical aggregator (PUB tcp://*:5590, REP tcp://*:5600, ingest 5570)
#   2. MID-360 LiDAR producer -> v1/lidar/raw (idle until enabled)
#   3. MQTT PLC sensor ingest (boom state + docking range)
#   4. dashboard (system python, Jetson hardware decode)
#   5. [optional] docking OAK camera adapter (192.168.50.21)
#   6. [optional] cutting OAK camera adapter (192.168.50.22)
#
# CAMERAS=0 (the default) does NOT start the two OAK adapters: the LiDAR is the
# priority workload and the OAK encode pipeline competes with it for Orin CPU.
# Set CAMERAS=1 to restore the previous camera behaviour.
#
# LIDAR=1 (the default) starts the LiDAR producer, which begins idle and waits
# for {"enabled": true} on its control endpoint.  LIDAR_MODE=livox-sdk drives
# the real MID-360 (needs Livox-SDK2 built and installed); the default
# LIDAR_MODE=synthetic needs no hardware.  LIDAR=0 disables it entirely.
#
# The legacy Raspberry Pi PLC stream adapter (range_ingest) is no longer part
# of the default stack: the MQTT subscriber now supplies the boom/range data.
# Set WITH_RANGE_INGEST=1 to launch it as well (code retained for reference).
#
# When CAMERAS=1 the two OAK adapters start with a delay between them:
# connecting two OAK devices back-to-back triggers an intermittent "stack
# smashing" firmware crash, so we stagger the launches.  Each adapter also runs
# under ``--supervise`` so a native crash auto-restarts the feed.
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
CODEC="${CODEC:-h265}"
DEPTH="${DEPTH:-1}"
IMU="${IMU:-1}"

# Cameras are OFF by default: the LiDAR is the priority workload and the OAK
# encode pipeline competes with it for Orin CPU.  Set CAMERAS=1 to launch the
# two OAK adapters as before.
CAMERAS="${CAMERAS:-0}"

# LiDAR is ON by default in synthetic mode; LIDAR=0 disables it, and
# LIDAR_MODE=livox-sdk drives the real MID-360 (requires Livox-SDK2 installed
# and --sdk-config pointing at the deployment 192.168.50.30/10 JSON).
LIDAR="${LIDAR:-1}"
LIDAR_MODE="${LIDAR_MODE:-synthetic}"
LIDAR_SECTOR_DEG="${LIDAR_SECTOR_DEG:-120}"

# Depth is on by default; DEPTH=0 keeps the OAK adapters RGB-only (byte-for-
# byte the pre-depth behaviour).  Depth streams on the canonical
# v1/camera/<name>/depth + camera_info channels and never affects RGB.
# DEPTH=0 appends --no-depth to the OAK commands below.

# IMU is on by default; IMU=0 disables the v1/camera/<name>/imu stream (the
# vibration-compensation source for the point cloud), appending --no-imu below.

# Commands are built as ARGUMENT ARRAYS, not strings that get eval'd.  Shell
# metacharacters or spaces in an operator-supplied env value (LIDAR_SECTOR_DEG,
# MQTT_TOPIC, ...) must never be able to change the command that runs.  `env`
# supplies the per-process environment (PYTHONPATH, DISPLAY) without needing
# inline VAR=value prefixes, which only work inside an eval'd string.
AGG_CMD=(env "PYTHONPATH=canonical_zmq:." "${DAI_PY}" -m canonical_zmq_publisher.main --ingest tcp://*:5570)
DOCK_CMD=(env "PYTHONPATH=canonical_zmq:." "${DAI_PY}" -m canonical_zmq_publisher.oak_capture --camera-role docking_camera --ingest-endpoint tcp://127.0.0.1:5570 --supervise --codec "${CODEC}" --ev-compensation 0 --brightness 0 --contrast 0)
CUT_CMD=(env "PYTHONPATH=canonical_zmq:." "${DAI_PY}" -m canonical_zmq_publisher.oak_capture --camera-role cutting_camera --ingest-endpoint tcp://127.0.0.1:5570 --supervise --codec "${CODEC}" --ev-compensation 0 --brightness 0 --contrast 0)
if [[ "${DEPTH}" == "0" ]]; then
  DOCK_CMD+=(--no-depth)
  CUT_CMD+=(--no-depth)
fi
if [[ "${IMU}" == "0" ]]; then
  DOCK_CMD+=(--no-imu)
  CUT_CMD+=(--no-imu)
fi
# LiDAR producer: starts idle (no stream) and waits for a remote enable command
# on the control endpoint, so the sensor only runs when the operator asks.
# --sdk-config is required by livox-sdk mode; the deployment JSON targets the
# sensor LAN (LiDAR 192.168.50.30, host 192.168.50.10).
LIDAR_SDK_CONFIG="${LIDAR_SDK_CONFIG:-${ROOT}/mid360_config.json}"
LIDAR_CONTROL_ENDPOINT="${LIDAR_CONTROL_ENDPOINT:-tcp://127.0.0.1:5571}"
LIDAR_LIDAR_IP="${LIDAR_LIDAR_IP:-192.168.50.30}"
LIDAR_HOST_IP="${LIDAR_HOST_IP:-192.168.50.10}"
LIDAR_CMD=(env "PYTHONPATH=canonical_zmq:." "${DAI_PY}" -m canonical_zmq_publisher.lidar_capture --sdk-mode "${LIDAR_MODE}" --ingest-endpoint tcp://127.0.0.1:5570 --control-endpoint "${LIDAR_CONTROL_ENDPOINT}" --sdk-config "${LIDAR_SDK_CONFIG}" --lidar-ip "${LIDAR_LIDAR_IP}" --host-ip "${LIDAR_HOST_IP}" --sector-deg "${LIDAR_SECTOR_DEG}" --max-points 2000 --disabled)
# Legacy Raspberry Pi PLC stream ingest: disabled by default (the MQTT
# subscriber now supplies the boom/range data).  Enable with WITH_RANGE_INGEST=1.
WITH_RANGE_INGEST="${WITH_RANGE_INGEST:-0}"
RANGE_CMD=(env "PYTHONPATH=canonical_zmq:." "${DAI_PY}" -m canonical_zmq_publisher.range_ingest --sensor-sub "${SENSOR_SUB:-tcp://192.168.50.40:5555}" --topic "${SENSOR_TOPIC:-harvester.sensors.v1}" --ingest-endpoint tcp://127.0.0.1:5570)
# MQTT ingest: SUB to the Mosquitto broker (Node-RED PLC stream on
# harvester/sensors/v1) and PUSH canonical packets into the aggregator.
MQTT_CMD=(env "PYTHONPATH=canonical_zmq:." "${DAI_PY}" -m canonical_zmq_publisher.mqtt_ingest --mqtt-host "${MQTT_HOST:-192.168.50.40}" --mqtt-port "${MQTT_PORT:-1883}" --topic "${MQTT_TOPIC:-harvester/sensors/v1}" --ingest-endpoint tcp://127.0.0.1:5570)
# MALLOC_ARENA_MAX=2 caps glibc at two malloc arenas (default is cores*8=32 on
# aarch64), which keeps the per-frame 6 MB numpy buffers from fragmenting the
# process address space into hundreds of mmap'd arenas.  This is the dominant
# contributor to the dashboard's ~3 GB steady-state RSS.
DASH_CMD=(env "DISPLAY=${DISPLAY_TARGET}" MALLOC_ARENA_MAX=2 "PYTHONPATH=harvester_dashboard" "${SYS_PY}" -m harvester_dashboard.main --pub tcp://127.0.0.1:5590 --status tcp://127.0.0.1:5600)

log() { printf '\033[1;32m[run_all]\033[0m %s\n' "$*"; }

# Stop any already-running instance of this stack first so we never double-bind
# the canonical 5590/5600/5570 ports.
"$ROOT/stop_all.sh" 2>/dev/null || true

if [[ "${1:-}" == "foreground" ]]; then
  log "Launching in foreground (Ctrl-C to stop all)."
  log "aggregator:  ${AGG_CMD[*]}"
  if [[ "${CAMERAS}" == "1" ]]; then
    log "docking:     ${DOCK_CMD[*]}"
    log "cutting:     ${CUT_CMD[*]}"
  else
    log "cameras:     disabled (CAMERAS=1 to enable)"
  fi
  if [[ "${LIDAR}" == "1" ]]; then
    log "lidar:       ${LIDAR_CMD[*]}"
  else
    log "lidar:       disabled (LIDAR=0)"
  fi
  log "mqtt:        ${MQTT_CMD[*]}"
  if [[ "${WITH_RANGE_INGEST}" == "1" ]]; then
    log "range:       ${RANGE_CMD[*]}"
  fi
  log "dashboard:   ${DASH_CMD[*]}"

  # Start the background services, then run the dashboard in the
  # foreground so Ctrl-C tears everything down.
  ( cd "$ROOT" && exec "${AGG_CMD[@]}" ) &
  AGG_PID=$!
  DOCK_PID=""
  CUT_PID=""
  if [[ "${CAMERAS}" == "1" ]]; then
    ( cd "$ROOT" && exec "${DOCK_CMD[@]}" ) &
    DOCK_PID=$!
    # Stagger the second OAK device to avoid the back-to-back connect crash.
    sleep "$OAK_START_DELAY_S"
    ( cd "$ROOT" && exec "${CUT_CMD[@]}" ) &
    CUT_PID=$!
  fi
  LIDAR_PID=""
  if [[ "${LIDAR}" == "1" ]]; then
    ( cd "$ROOT" && exec "${LIDAR_CMD[@]}" ) &
    LIDAR_PID=$!
  fi
  ( cd "$ROOT" && exec "${MQTT_CMD[@]}" ) &
  MQTT_PID=$!
  RANGE_PID=""
  if [[ "${WITH_RANGE_INGEST}" == "1" ]]; then
    ( cd "$ROOT" && exec "${RANGE_CMD[@]}" ) &
    RANGE_PID=$!
  fi

  trap 'log "Stopping..."; kill $AGG_PID $DOCK_PID $CUT_PID $LIDAR_PID $MQTT_PID $RANGE_PID 2>/dev/null || true; wait 2>/dev/null || true' EXIT INT TERM

  cd "$ROOT"
  "${DASH_CMD[@]}"
else
  command -v tmux >/dev/null 2>&1 || { log "tmux not found; use './run_all.sh foreground'"; exit 1; }
  log "Launching in tmux session 'harvest' (attach with: tmux attach -t harvest)."

  tmux kill-session -t harvest 2>/dev/null || true

  # tmux runs a shell command line, so quote each argv element.  This keeps the
  # array form (no eval) while still surviving spaces in a path or value.
  shell_quote() { printf '%q ' "$@"; }

  tmux new-session -d -s harvest -n agg  "cd '$ROOT' && $(shell_quote "${AGG_CMD[@]}")"
  if [[ "${CAMERAS}" == "1" ]]; then
    tmux new-window -t harvest -n docking "cd '$ROOT' && $(shell_quote "${DOCK_CMD[@]}")"
    # Stagger the cutting camera: connect the second OAK after a delay so the
    # two devices never negotiate XLink back-to-back (firmware crash trigger).
    tmux new-window -t harvest -n cutting "cd '$ROOT' && sleep $OAK_START_DELAY_S && $(shell_quote "${CUT_CMD[@]}")"
  fi
  if [[ "${LIDAR}" == "1" ]]; then
    tmux new-window -t harvest -n lidar   "cd '$ROOT' && $(shell_quote "${LIDAR_CMD[@]}")"
  fi
  if [[ "${WITH_RANGE_INGEST}" == "1" ]]; then
    tmux new-window -t harvest -n range   "cd '$ROOT' && $(shell_quote "${RANGE_CMD[@]}")"
  fi
  tmux new-window -t harvest -n mqtt    "cd '$ROOT' && $(shell_quote "${MQTT_CMD[@]}")"
  tmux new-window -t harvest -n dash    "cd '$ROOT' && $(shell_quote "${DASH_CMD[@]}")"

  log "Components started in tmux windows: agg, [docking, cutting], [lidar], mqtt, dash."
  log "Cameras: ${CAMERAS} (CAMERAS=1 to enable)  LiDAR: ${LIDAR} (mode ${LIDAR_MODE})."
  log "Attach:        tmux attach -t harvest"
  log "Stop:          ./stop_all.sh"
fi
