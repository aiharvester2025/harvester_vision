#!/usr/bin/env bash
# Configure the Orin as a PTP MASTER for the Livox MID-360 and verify it.
#
# The MID-360 has no UTC clock of its own: it slaves to a PTP/gPTP master (or
# GPS) and only then stamps packets with absolute time (time_type==1/2). With no
# master the packets carry time_type==0, i.e. nanoseconds since the LiDAR powered
# on. Livox's protocol makes the LiDAR the slave always, so THIS host must be the
# master. This script installs linuxptp, launches ptp4l on the sensor NIC, ties
# the NIC clock to the Orin's CLOCK_REALTIME, and reports whether the LiDAR
# actually locked (via the adapter's readback, not by assumption).
#
# Usage:
#   sudo deploy/ptp/setup_ptp_master.sh [SENSOR_IF] [--check-only]
#
# SENSOR_IF defaults to $SENSOR_IF or eth1 (the deployment sensor NIC).
#
# HARDWARE REALITY CHECK (run first: `sudo deploy/ptp/setup_ptp_master.sh --check-only`)
# The dev-kit sensor NIC is a Realtek RTL8168 (r8168 driver) with NO PTP hardware
# clock, so ptp4l falls back to software timestamping (~us-to-ms jitter, fine for
# a scan timestamp, not a precision-time setup). The Orin's only PHC (/dev/ptp0)
# belongs to the *other* NIC (Microchip lan743x). If the LiDAR can be moved to a
# PTP-capable NIC, run this script with that interface name and switch the cfg's
# time_stamping to `hardware`.

set -euo pipefail

# Parse args without positional-order traps: a flag must never be mistaken for
# the interface name (that silently probed an interface called "--check-only").
CHECK_ONLY=0
SENSOR_IF=""
for arg in "$@"; do
  case "$arg" in
    --check-only) CHECK_ONLY=1 ;;
    -*)           printf 'unknown option: %s\n' "$arg" >&2; exit 2 ;;
    *)            if [[ -z "$SENSOR_IF" ]]; then SENSOR_IF="$arg"; else
                    printf 'unexpected extra argument: %s\n' "$arg" >&2; exit 2
                  fi ;;
  esac
done
# Fall back to $SENSOR_IF from the environment, then to eth1 (deployment NIC).
SENSOR_IF="${SENSOR_IF:-${SENSOR_IF_ENV:-eth1}}"

CFG_SRC="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/ptp4l-orin-master.cfg"
CFG_DST="/etc/linuxptp/ptp4l-orin-master.cfg"

log()  { printf '\033[1;32m[ptp]\033[0m %s\n' "$*"; }
warn() { printf '\033[1;33m[ptp]\033[0m %s\n' "$*"; }
fail() { printf '\033[1;31m[ptp]\033[0m %s\n' "$*" >&2; }

if [[ "$(id -u)" -ne 0 && "$CHECK_ONLY" -eq 0 ]]; then
  fail "must run as root to install/launch PTP (use --check-only to just inspect)"
  exit 1
fi

log "sensor interface: ${SENSOR_IF}"

# --- 1. Preflight: does this NIC support hardware timestamping? ---------------
HW_TS=0
if command -v ethtool >/dev/null 2>&1; then
  if ethtool -T "$SENSOR_IF" 2>/dev/null | grep -q "SOF_TIMESTAMPING_RAW_HARDWARE"; then
    HW_TS=1
    log "hardware timestamping: YES (PHC present)"
  else
    warn "hardware timestamping: NO on ${SENSOR_IF} -> software timestamping"
    warn "  (jitter ~us-to-ms; acceptable for scan stamps, not precision PTP)"
    warn "  check 'ethtool -T ${SENSOR_IF}' and /sys/class/net/${SENSOR_IF}/ptp*"
  fi
else
  warn "ethtool not installed; cannot confirm timestamping capability"
fi

# --- 2. Preflight: is the Orin's own clock disciplined? ------------------------
# The host-arrival fallback is only real UTC if the host clock is synced.
if command -v chronyc >/dev/null 2>&1; then
  ref="$(chronyc tracking 2>/dev/null | awk -F': *' '/^Reference ID/{print $2}')"
  if [[ "$ref" == "7F7F0101"* || "$ref" == "127.127.1.1"* ]]; then
    warn "Orin clock is NOT synchronized (chrony ref=${ref:-unknown}, own local clock)"
    warn "  fix chrony/PLC RTC first, or host-arrival LiDAR timestamps are not UTC"
  else
    log "Orin clock reference: ${ref:-unknown}"
  fi
fi

if [[ "$CHECK_ONLY" -eq 1 ]]; then
  log "--check-only: stopping before any install/launch"
  exit 0
fi

# --- 3. Install linuxptp ------------------------------------------------------
if ! command -v ptp4l >/dev/null 2>&1; then
  log "installing linuxptp (offline apt cache expected)..."
  if ! apt-get install -y linuxptp; then
    fail "apt-get install linuxptp failed; install it offline and re-run"
    exit 1
  fi
fi
log "ptp4l: $(command -v ptp4l)"

# --- 4. Install the config ----------------------------------------------------
install -d /etc/linuxptp
install -m 0644 "$CFG_SRC" "$CFG_DST"
log "installed ${CFG_DST}"

# --- 5. Launch ptp4l (master) + phc2sys (NIC clock <- CLOCK_REALTIME) ---------
# phc2sys is only meaningful with a hardware PHC. On a software-timestamping NIC
# there is no NIC clock to discipline, so it is skipped rather than started
# pointlessly.
# Match the ptp4l binary by name (-x), not a `-f` command-line regex: `pkill -f`
# can match this script's own argv or an unrelated process that merely mentions
# the pattern, and the interface name is interpolated into that regex.
pkill -x ptp4l 2>/dev/null || true
sleep 0.5

log "starting ptp4l as master on ${SENSOR_IF} (domain 0, E2E, two-step)..."
# Validate the config BEFORE backgrounding ptp4l: an unknown key makes ptp4l
# exit immediately, and a backgrounded failure would otherwise look like success.
CONFIG_CHECK="$(timeout 3 ptp4l -i "$SENSOR_IF" -f "$CFG_DST" -m 2>&1 | head -1 || true)"
if echo "$CONFIG_CHECK" | grep -qiE "unknown option|failed to parse|invalid"; then
  fail "ptp4l rejected the config: ${CONFIG_CHECK}"
  fail "  fix ${CFG_DST} and re-run"
  exit 1
fi

ptp4l -i "$SENSOR_IF" -f "$CFG_DST" -m >/var/log/ptp4l-"${SENSOR_IF}".log 2>&1 &
PTP_PID=$!
sleep 2
if ! kill -0 "$PTP_PID" 2>/dev/null; then
  fail "ptp4l exited immediately; last log lines:"
  tail -5 /var/log/ptp4l-"${SENSOR_IF}".log >&2 || true
  exit 1
fi
log "ptp4l pid ${PTP_PID}; log /var/log/ptp4l-${SENSOR_IF}.log"

if [[ "$HW_TS" -eq 1 ]]; then
  if [[ -e /dev/ptp0 ]]; then
    pkill -x phc2sys 2>/dev/null || true
    # -O 0: no offset between PHC and system clock. -s CLOCK_REALTIME makes the
    # NIC PHC follow the (chrony-disciplined) system clock, so PTP time == UTC.
    phc2sys -s CLOCK_REALTIME -c /dev/ptp0 -O 0 -m \
      >/var/log/phc2sys-"${SENSOR_IF}".log 2>&1 &
    log "phc2sys pid $!; PHC /dev/ptp0 <- CLOCK_REALTIME"
  fi
else
  warn "skipping phc2sys: ${SENSOR_IF} has no PHC to discipline"
fi

sleep 3
log "ptp4l state:"
grep -E "selected|master offset|assuming the grand master|FAULTY|port .* (MASTER|SLAVE)" \
  /var/log/ptp4l-"${SENSOR_IF}".log | tail -5 || true

cat <<EOF

Next: confirm the LiDAR actually locked. The device, not this script, is the
source of truth:

  PYTHONPATH=canonical_zmq:. /home/marcop/depthai-env/bin/python3 -m \\
    canonical_zmq_publisher.lidar_capture --sdk-mode livox-sdk \\
    --sdk-config ./mid360_config.json --enabled

Watch for the adapter's line:
  [livox] time sync: SYNCHRONIZED via ptp (time_type=1) ...
If it still says NOT SYNCHRONIZED (time_type=0), the LiDAR did not accept us as
master: check that ptp4l is on the SAME interface the LiDAR is on, that only one
master exists on the LAN, and that the LiDAR firmware supports PTP.
EOF
