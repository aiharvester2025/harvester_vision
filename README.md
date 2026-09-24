# Harvester Vision

## OAK RGB applications

`oak_rgb_publisher.py` connects to one OAK camera through DepthAI v3, encodes
its color stream as MJPEG, and publishes MessagePack frames over ZeroMQ. It
also listens for viewer enable/disable commands, so inactive camera streams do
not send frames.

| Role | OAK address | Video PUB | Control PULL |
| --- | --- | --- | --- |
| `docking_camera` | `192.168.50.21` | `tcp://*:5556` | `tcp://*:5566` |
| `cutting_camera` | `192.168.50.22` | `tcp://*:5557` | `tcp://*:5567` |

Run both publishers on the Orin. Each process connects to its configured PoE
OAK address and binds a different pair of **local** ZeroMQ ports. Start them
disabled so the viewer explicitly enables only its selected camera:

```bash
# Terminal 1: docking OAK at 192.168.50.21
python3 oak_rgb_publisher.py --camera-role docking_camera --disabled
```

```bash
# Terminal 2: cutting OAK at 192.168.50.22
python3 oak_rgb_publisher.py --camera-role cutting_camera --disabled
```

Publisher defaults are 1280×720 at 15 FPS and MJPEG quality 65. Use
`--width`, `--height`, `--fps`, and `--quality` to override them. `--device`,
`--topic`, `--pub-port`, and `--ctl-port` are available for a custom setup,
while `--camera-role` always identifies the camera's configured role.

Each MessagePack payload contains MJPEG bytes in `frame` and these metadata
fields: `topic`, `camera_role`, `sequence_number`, `fps`, `timestamp_us`,
`timestamp_monotonic_us`, `received_timestamp_us`, `timestamp_source`,
`time_authority`, `time_quality`, and `chrony_offset_us`.

`oak_rgb_viewer.py` subscribes to both local publishers, displays one selected
camera, and sends control commands to pause the other publisher. Start it with:

```bash
# Terminal 3: viewer on the Orin
python3 oak_rgb_viewer.py --display-fps 15
```

Press `1` for the docking camera, `2` for the cutting camera, `3` to toggle
the Raspberry Pi docking-sensor dashboard overlay, and `q` or `Esc` to exit.
The viewer shows the capture UTC timestamp and time-sync health
on live frames. It marks a stream offline after two seconds by default; change
this with `--timeout`. Use repeated `--camera TOPIC SUB_ADDR CTL_ADDR` options
to connect to non-default publisher endpoints.

## Docking sensor viewer

The Raspberry Pi at `192.168.50.40` publishes five simulated docking-sensor
readings on ZeroMQ port `5555`. Run this viewer on the Orin (`192.168.50.10`)
to display the live measurements, phase, and alignment estimates:

```bash
python3 sensor_viewer.py
```

It subscribes to `tcp://192.168.50.40:5555`, topic `harvester.sensors.v1`, by
default. Use `--host`, `--port`, or `--topic` if the publisher changes. The
panel indicates stale telemetry after two seconds; press `q` or `Esc` to quit.
It has no DepthAI dependency, so its subscriber and drawing functions can be
reused as a later overlay in `oak_rgb_viewer.py`.

### Stream and resource behavior

The viewer opens one display window only. Selecting a camera sends
`enabled=True` to that publisher and `enabled=False` to the other publisher;
the paused publisher stops sending MJPEG frames to the viewer. The current
pause mechanism does not stop the inactive DepthAI pipeline itself, so it does
not eliminate all OAK-to-Orin traffic or device processing.

The standard low-load configuration is 1280×720, 15 FPS, MJPEG quality 65,
and a 15 FPS viewer redraw rate. These settings are suitable for a single live
camera and leave more CPU headroom for other workloads:

```bash
python3 oak_rgb_viewer.py --display-fps 15
```

The publisher uses bounded DepthAI output queues and a short idle sleep when
no frame is ready, avoiding full-speed polling. Use `tegrastats` to monitor
CPU, temperature, RAM, and swap during deployment. For an accurate camera-only
baseline, close or minimize remote-desktop and browser workloads first.

### Run with systemd

After installing `deploy/systemd/oak-rgb-publisher@.service` into
`/etc/systemd/system/`, start both camera publishers with:

```bash
sudo systemctl daemon-reload
sudo systemctl enable --now oak-rgb-publisher@docking_camera.service
sudo systemctl enable --now oak-rgb-publisher@cutting_camera.service
sudo systemctl status 'oak-rgb-publisher@*.service'
```

Run the viewer in the logged-in desktop session, not as a system service:

```bash
cd /home/marcop/harvester_vision
python3 oak_rgb_viewer.py --display-fps 15
```

To stop the manual publisher processes, press `Ctrl+C` in their terminals. To
stop systemd publishers, run:

```bash
sudo systemctl disable --now oak-rgb-publisher@docking_camera.service
sudo systemctl disable --now oak-rgb-publisher@cutting_camera.service
```

## Camera time synchronization (phase 1)

The PLC's battery-backed RTC is the persistent UTC authority. The offline Orin
synchronizes to the PLC NTP server at `192.168.50.40`, then each OAK uses
DepthAI v3 host-clock synchronization. `ImgFrame.getTimestamp()` is
host-aligned monotonic time, not UTC. The publisher maps it to capture-time
Unix epoch microseconds and emits `timestamp_us`, `timestamp_monotonic_us`,
`received_timestamp_us`, `timestamp_source`, `time_authority`, and
`time_quality` in every ZMQ payload.

### Orin deployment

1. Replace `REPLACE_WITH_SENSOR_NIC` in
   `deploy/networkmanager/orin-sensor-lan.nmconnection` with the dedicated
   sensor Ethernet interface, then install the connection through NetworkManager.
   It assigns `192.168.50.10/24` with no default route.
2. Configure the PLC at `192.168.50.40` to serve its battery-backed UTC clock
   over NTP. Include `deploy/chrony/harvester-sensor.conf` from the Orin
   chrony configuration and restart chrony. Verify `chronyc tracking`: `Leap
   status: Normal` and reference `192.168.50.40` are reported as
   `synchronized`; a different reference is `unexpected_source`, and a PLC
   outage is `holdover` or `unknown`.
3. Connect one camera in bootloader mode at a time and use
   `python3 set_oak_ip.py --camera-role docking_camera --yes`, then repeat
   with `cutting_camera`. This is a physical-device write.
4. Install the provided systemd template and enable both instances:
   `oak-rgb-publisher@docking_camera` and
   `oak-rgb-publisher@cutting_camera`.

Run `python3 scripts/validate_camera_time_sync.py --duration 300
--require-synchronized` while both publishers are active. It fails if a camera
is missing, timestamps regress, chrony is not synchronized, or inferred
combined clock-offset drift exceeds 10 ms.

PLC/Modbus timestamp registers are intentionally not configured in this phase.

## MID-360 LiDAR time synchronization

**The MID-360 does NOT run a UTC clock on its own.** By default its point
timestamps are nanoseconds *since the LiDAR powered on*. The device only stamps
absolute time while it is slaved to an external time master over PTP/gPTP (or
GPS). Livox's protocol makes the LiDAR always the PTP **slave**, so **the Orin
must be the PTP master**. Until a master is present, the LiDAR is genuinely
unsynchronized — this is not a configuration oversight that a small code change
can paper over.

Each point packet carries a `time_type` byte that states this explicitly:

| `time_type` | Meaning | Usable as UTC? |
|---|---|---|
| `0` | no sync source; timestamp = ns since device power-on | **No** |
| `1` | PTP or gPTP; timestamp = master clock in ns | Yes |
| `2` | GPS; timestamp = GPS time in ns | Yes |

`lidar/livox_source.py` decodes this field per packet and never assumes the
sensor is synchronized. `LivoxMid360Source.timing_snapshot()` is the single
source of truth: it returns the chosen acquisition time **and** the status that
describes it in one read, so the timestamp and its label can never disagree (a
separate status call could otherwise label a stale host-clock value as `ptp`).
`timestamp_status()` remains available for device-state diagnostics.

The snapshot picks the acquisition time in this order:

1. the LiDAR's **own** PTP/GPS timestamp when `time_type` is 1 or 2 **and** the
   sample is fresh (immune to host scheduling jitter), else
2. the **Orin's `CLOCK_REALTIME` at packet arrival** when the LiDAR is not
   synchronized (or the absolute sample is stale) — a boot-relative counter must
   never be published as epoch time, and the Orin clock is real UTC only insofar
   as chrony disciplines it (see the camera section above), else
3. `host_now` (publish time, clearly labelled) when no packet has ever arrived.

The canonical `v1/lidar/raw` header reports which happened, so a consumer can
tell the difference without guessing:

| Field | Values |
|---|---|
| `clock_domain` | `lidar_ptp_utc` (LiDAR synced) or `orin_realtime` (host fallback) |
| `timestamp_source` | `livox_ptp`, `livox_gps`, `host_arrival`, or `host_now` |
| `lidar_time_sync` | bool — was the sensor clock absolute for this scan |
| `lidar_time_type` | the raw `time_type` byte |
| `lidar_time_source` | `ptp` / `gps` / `device_uptime` / `unknown` / `no_data` |
| `time_quality` | Orin chrony health (`synchronized` / `holdover` / ...) |

`time_quality` and `lidar_time_sync` are **independent**: the host can be
disciplined while the LiDAR is not, and vice versa.

### Enabling PTP (making the LiDAR timestamps absolute)

Status: **verified working on the deployment Orin NX** (2026-09-23). The
MID-360 reports `time_type=1` and the adapter publishes `livox_ptp` /
`lidar_ptp_utc`.

```bash
# Inspect first: reports whether the sensor NIC can hardware-timestamp and
# whether the Orin's own clock is disciplined. Read-only; safe to run any time.
sudo deploy/ptp/setup_ptp_master.sh --check-only

# Install linuxptp, install the master config, launch ptp4l, and self-check
# that it actually started (it fails loudly instead of backgrounding a dead
# service).
sudo deploy/ptp/setup_ptp_master.sh eth1

# Persist across reboots:
sudo cp deploy/ptp/ptp4l-orin-master.cfg /etc/linuxptp/
sudo cp deploy/systemd/ptp4l-master@.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable --now ptp4l-master@eth1.service
```

Confirm the master role, then confirm from the **device** (not from the config):

```bash
# Orin is grandmaster on eth1:
journalctl -u ptp4l-master@eth1.service -n 20 | grep -E "MASTER|grand master"

# Device accepted us (this is the only authoritative check):
PYTHONPATH=canonical_zmq:. python3 -m canonical_zmq_publisher.lidar_capture \
    --sdk-mode livox-sdk --sdk-config ./mid360_config.json --enabled
# look for: [livox] time sync: SYNCHRONIZED via ptp (time_type=1)
```

Measured on the deployment unit with PTP locked: the LiDAR and Orin clocks agree
to roughly **1–2 ms (σ ≈ 0.5 ms)**. That offset is dominated by software
timestamping plus UDP/kernel/SDK delivery latency, not by clock disagreement.

**Hardware caveat (verified on the current Orin NX dev kit).** The sensor NIC
here is `eth1` = Realtek RTL8168 (`r8168`), which has **no PTP hardware clock
and no hardware timestamping**; the Orin's only PHC (`/dev/ptp0`) belongs to
`eth0` (Microchip lan743x, the desk/WAN NIC). `ptp4l` therefore uses *software*
timestamping, which carries microseconds-to-milliseconds of NIC/IRQ/scheduler
jitter. That is acceptable for dating a LiDAR scan and for the `time_type==1`
handshake, but it is not precision time transfer. For sub-microsecond accuracy,
move the MID-360 onto a NIC with its own PHC and set `time_stamping hardware` in
`deploy/ptp/ptp4l-orin-master.cfg`. Verify any candidate NIC with
`ethtool -T <iface>` (needs `SOF_TIMESTAMPING_RAW_HARDWARE`).

**Two independent clocks — do not conflate them.** PTP makes the **LiDAR** clock
absolute. The Orin's **own** `CLOCK_REALTIME` is disciplined separately by chrony
against the PLC's battery-backed RTC at `192.168.50.40`. When the PLC is stopped
(as during commissioning) the Orin falls back to its own local clock, so the
host-arrival fallback path is only as good as the Orin RTC — while the LiDAR PTP
path stays correct regardless. Check both:

```bash
chronyc tracking     # Orin clock: want Leap status Normal + the PLC reference
```

`SetLivoxLidarPpsSyncMode` is bound when the installed SDK exposes it (the
header documents it for the MID-360S). It is a GPS time-*robustness* filter, not
a switch that enables PTP, so a missing symbol is harmless and never blocks the
start sequence.

## Frame transforms and calibration foundation

The repository now has a safe, pre-implementation coordinate-frame contract in
[`calibration/README.md`](calibration/README.md). It defines the rail, harvester,
OAK camera optical, MID-360, docking-reference, and five range-sensor frames;
the timestamp/calibration metadata every future spatial measurement must carry;
and the commissioning sequence.

Use `calibration/frames.nominal.json` only for Xavier/RViz simulation. It uses
explicitly illustrative mount geometry and the validator rejects it for
deployment. `calibration/frames.deployment.template.json` intentionally leaves
all physical survey values blank until a measured, approved calibration session
is recorded.

## MID-360 LiDAR: on-demand canonical capture

The Orin ingests the Livox MID-360 without ROS: points arrive over UDP via
Livox-SDK2 and are republished onto the canonical bus as `v1/lidar/raw`
(`codec: lidar_xyz_f32`), matching the OAK publisher pattern. See
`canonical_zmq/canonical_zmq_publisher/lidar_capture.py` (the producer),
`lidar/livox_source.py` (the only file that touches the Livox SDK),
`lidar/leveling.py`, and `plc_sensor_bridge.py`.

**Network (sensor LAN `192.168.50.0/24`).** The MID-360 is addressed at
`192.168.50.30`; the Orin host runs the SDK at `192.168.50.10`. The LiDAR IP is
recorded in `calibration/frames.deployment.template.json` (`lidar_binding`),
and the SDK2 config is `mid360_config.json`.

**On-demand operation.** The LiDAR does not stream 24/7: the producer starts
idle, and in `livox-sdk` mode the SDK is not even started until the operator
enables it, so there is no UDP stream while idle. The control channel is a PULL
socket accepting `{"enabled": true|false}`, the same contract the OAK publishers
use. While disabled the backlog is discarded and nothing is published.

The control socket binds **loopback by default** (`tcp://127.0.0.1:5571`)
because the channel is unauthenticated. To control the LiDAR from another host,
set `LIDAR_CONTROL_ENDPOINT=tcp://*:5571` (or pass `--control-endpoint`) and only
do so on the trusted sensor LAN.

```bash
# On the Orin (default loopback bind):
python3 -c "import zmq; s=zmq.Context().socket(zmq.PUSH); \
  s.connect('tcp://127.0.0.1:5571'); s.send_json({'enabled': True})"
```

The same channel can be reached over SSH without exposing the port:

```bash
ssh marcop@<orin> "python3 -c \"import zmq; s=zmq.Context().socket(zmq.PUSH); \
  s.connect('tcp://127.0.0.1:5571'); s.send_json({'enabled': True})\""
```

**Pipeline.** Each scan is drained from the SDK, leveled to gravity
(rotation-only; the sensor stays at the HUD origin), clipped to a **120°
forward sector** and to `[0.15, 40] m`, then downsampled to `--max-points`
(default 2000). Leveling re-aligns the cloud to gravity using orientation
*only* (rotation-only, no translation). Yaw is not required and is deliberately
unused (IMU yaw drifts; the 2-axis tilt sensor has no yaw).

Three orientation sources are supported via `--level-source`:
- `imu`  — MID-360 built-in IMU (pitch/roll are gravity-referenced and stable).
- `tilt` — platform 2-axis tilt sensor (via `plc_sensor_bridge.py`).
- `boom` — boom angle sensor (via `plc_sensor_bridge.py`).

The MID-360's native vendor frame differs from the project's `+X forward /
+Y left / +Z up` mechanical convention; confirm the installed vendor frame and
convert it in `lidar/livox_source.py` before commissioning.

### Building Livox-SDK2 (required for `--sdk-mode livox-sdk`)

The SDK shared library is **not** shipped in this repo. Build it once on the
Orin so `liblivox_lidar_sdk_shared.so` is on the loader path:

```bash
git clone https://github.com/Livox-SDK/Livox-SDK2.git
cd Livox-SDK2 && mkdir build && cd build
cmake .. && make -j"$(nproc)" && sudo make install
sudo ldconfig
```

### Sensor-LAN requirements (learned on real hardware)

Two network conditions are mandatory; without them detection succeeds but **no
points ever arrive**:

1. **The multicast group must route via the sensor interface.** The SDK joins
   `224.1.1.5` on whichever interface the routing table selects; if the default
   route is the wifi/LAN NIC, the group is joined on the wrong port and the
   LiDAR data is dropped. A dispatcher script installs this persistently:

   ```bash
   # /etc/NetworkManager/dispatcher.d/90-livox-multicast
   SENSOR_IF=eth1
   if [ "$1" = "$SENSOR_IF" ] && [ "$2" = "up" ]; then
       # Guard: if the interface name was not the sensor NIC, routing the group
       # here silently sends LiDAR data off the wrong device. Fail loudly instead.
       if ! ip -o link show "$SENSOR_IF" >/dev/null 2>&1; then
           logger -t livox-multicast "sensor interface $SENSOR_IF not present; skipping"
           exit 1
       fi
       ip route replace 224.1.1.5/32 dev "$SENSOR_IF" scope link
   fi
   ```

   Confirm the sensor NIC name before relying on this: the interface is `eth1`
   on the current Orin, but a renamed or rebound NIC would need `SENSOR_IF`
   changed to match. Verify: `ip maddr show dev eth1 | grep 224.1.1.5` (must be
   present) and `ip route get 224.1.1.5` (must say `dev eth1`).

2. **The LiDAR must send to the multicast group.** The SDK socket is bound to
   `224.1.1.5:56301`, so a LiDAR left on its default unicast destination
   (`192.168.50.10:56301`) is never received. The adapter sends
   `SetLivoxLidarPointDataHostIPCfg` / `...ImuDataHostIPCfg` for `224.1.1.5`
   **before** starting the motor; see `_start_device()` in `livox_source.py`.

Also note the SDK does **not** emit `SetLivoxLidarInfoChangeCallback` on the
non-view path for normal firmware (its `is_load_mode` guard is inverted), so the
adapter starts the device via a poll fallback keyed on `--lidar-ip` rather than
waiting for that callback.

### Running

Offline (no hardware), exercising the full canonical path:

```bash
PYTHONPATH=canonical_zmq:. python3 -m canonical_zmq_publisher.lidar_capture \
    --sdk-mode synthetic --enabled
```

Live MID-360 (starts idle; enable over the control socket):

```bash
PYTHONPATH=canonical_zmq:. python3 -m canonical_zmq_publisher.lidar_capture \
    --sdk-mode livox-sdk --sdk-config ./mid360_config.json \
    --level-source imu --sector-deg 120 --max-points 2000
```

The whole stack (aggregator + LiDAR + dashboard, **cameras off**) is one call:

```bash
./run_all.sh              # CAMERAS=0 (default) and LIDAR=1 (default)
CAMERAS=1 ./run_all.sh    # re-enable the two OAK adapters
LIDAR_MODE=livox-sdk ./run_all.sh   # drive the real sensor
```

The dashboard already decodes `v1/lidar/raw` through `LidarDecoder` and renders
it in `LidarInset.qml`; no dashboard change is needed.

**PLC/Modbus bridge.** `plc_sensor_bridge.py` is a skeleton that will poll the
PLC (boom angle, 2-axis tilt, five range sensors) and republish as MessagePack
over ZMQ. The Modbus register map must be filled from the PLC program.

**H.264/H.265 + Jetson hardware decode** is the successor to MJPEG (see
`todo.txt`). The OAK cameras encode H.264/H.265 (or MJPEG via `--codec jpeg`),
and the Orin decodes in hardware (`nvv4l2decoder` for H.264/H.265,
`nvjpegdec` for JPEG) in `harvester_dashboard/.../decoders/`. Timestamps
continue to follow the PLC-RTC UTC domain established in `time_sync.py`.

## Boom kinematics: LiDAR height above ground

The MID-360's height above ground is recovered from PLC length-sensor values
using `lidar/boom_kinematics.py` (dependency-free). The measured boom degrees
of freedom are the pivot angle and the telescopic extension (both computed in
the PLC from length sensors) plus the 2-axis platform tilt; the unmeasured
turret yaw, rail position, and cutting-arm lift do not affect the LiDAR's
*vertical* datum except the cutting-arm lift, which is a calibrated constant.

```text
ground -> base_link -> [yaw: unmeasured] -> pivot angle (measured)
       -> extension (measured) -> platform tilt (measured)
       -> [rail: unmeasured] -> [cutting-arm lift: calibrated constant]
       -> cutting_arm_base_link -> mid360_link (LiDAR)
```

This chain matches the RViz/Gazebo URDF
(`oil_palm_harvester_kinematic.urdf` in `ros2_ws`), which mounts the LiDAR
(`vehicle_lidar_link`) rigidly on `cutting_arm_base_link`.

`lidar_height_above_ground(state, geometry)` returns the LiDAR's Z above ground
from pivot angle + extension + tilt + calibrated offsets, so a leveled cloud's
tree height can be converted to an absolute height. See
`calibration/README.md` for the exact offsets to survey during commissioning.

## End-to-end example: tree height estimate

`examples/estimate_tree_height.py` shows the full pipeline from a raw MID-360
cloud to an absolute tree height and trunk-end height. It combines leveling
(IMU orientation) with boom kinematics (PLC pivot/extension/tilt) and a simple
trunk/canopy split. Run it from the repo root:

```bash
PYTHONPATH=. python3 examples/estimate_tree_height.py
PYTHONPATH=. python3 examples/estimate_tree_height.py --pivot-deg 45 --extension-m 1.5
PYTHONPATH=. python3 examples/estimate_tree_height.py --points my_cloud.json
```

The script is illustrative: its trunk/canopy classifier and synthetic cloud are
placeholders to be replaced by your real segmentation once live data arrives.
