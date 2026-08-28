#!/usr/bin/env bash
# Stop every process of the Orin canonical telemetry stack.
#
# Kills: the canonical aggregator, both OAK capture adapters, and the dashboard.
# Idempotent — safe to run even when nothing is running.

set -uo pipefail

PATTERNS=(
  "canonical_zmq_publisher.main"
  "canonical_zmq_publisher.oak_capture"
  "canonical_zmq_publisher.range_ingest"
  "harvester_dashboard.main"
)

killed_any=0
for pattern in "${PATTERNS[@]}"; do
  pids="$(pgrep -f "$pattern" || true)"
  if [[ -n "$pids" ]]; then
    printf '[stop_all] stopping %s (pids: %s)\n' "$pattern" "$(echo "$pids" | tr '\n' ' ')"
    # shellcheck disable=SC2086
    kill $pids 2>/dev/null || true
    killed_any=1
  fi
done

# Give graceful SIGTERM a moment, then force-kill survivors.
if [[ "$killed_any" == "1" ]]; then
  sleep 1
  for pattern in "${PATTERNS[@]}"; do
    pids="$(pgrep -f "$pattern" || true)"
    if [[ -n "$pids" ]]; then
      # shellcheck disable=SC2086
      kill -9 $pids 2>/dev/null || true
    fi
  done
fi

# Also tear down the tmux session if it exists.
if command -v tmux >/dev/null 2>&1 && tmux has-session -t harvest 2>/dev/null; then
  tmux kill-session -t harvest
  printf '[stop_all] killed tmux session "harvest"\n'
fi

printf '[stop_all] done\n'
