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

PTP/Livox and PLC/Modbus timestamp registers are intentionally not configured
in this phase.

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

## MID-360 LiDAR leveling and publishing (pre-implementation)

The Orin will ingest the Livox MID-360 without ROS: points arrive over UDP via
Livox-SDK2 and are republished over ZeroMQ, matching the OAK publisher pattern.
See `mid360_publisher.py`, `lidar/leveling.py`, `lidar/livox_source.py`, and
`plc_sensor_bridge.py`.

**Leveling.** The LiDAR is arm-mounted, so raw points rotate with arm pitch/roll
and a world-vertical tree appears to lean. Leveling re-aligns the cloud to
gravity using orientation *only* (rotation-only, no translation), so the sensor
stays at the HUD origin while the tree stands straight. Yaw is not required and
is deliberately unused (IMU yaw drifts; the 2-axis tilt sensor has no yaw).

Three orientation sources are supported via `--level-source`:
- `imu`  — MID-360 built-in IMU (pitch/roll are gravity-referenced and stable).
- `tilt` — platform 2-axis tilt sensor (via `plc_sensor_bridge.py`).
- `boom` — boom angle sensor (via `plc_sensor_bridge.py`).

The MID-360's native vendor frame differs from the project's `+X forward /
+Y left / +Z up` mechanical convention; confirm the installed vendor frame and
convert it in `lidar/livox_source.py` before commissioning.

Run offline (no hardware) to exercise the ZMQ/leveling plumbing:

```bash
python3 mid360_publisher.py --sdk-mode synthetic
```

**PLC/Modbus bridge.** `plc_sensor_bridge.py` is a skeleton that will poll the
PLC (boom angle, 2-axis tilt, five range sensors) and republish as MessagePack
over ZMQ. The Modbus register map must be filled from the PLC program.

**H.264/H.265 + Jetson hardware decode** is the planned successor to MJPEG
(see `todo.txt`); it is not yet implemented. Timestamps continue to follow the
PLC-RTC UTC domain established in `time_sync.py`.

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
