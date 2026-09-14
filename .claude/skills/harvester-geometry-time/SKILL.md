---
name: harvester-geometry-time
description: "Work with the harvester_vision coordinate frames, calibration, LiDAR leveling/kinematics, and time-synchronization foundations."
disable-model-invocation: true
argument-hint: "the frame/transform, leveling, kinematics, or time-sync task"
metadata:
  author: harvester
  version: "2.0.0"
  status: stable
---

# Harvester Geometry, Calibration & Time

Dependency-free foundations for the harvester's spatial + timing correctness: rigid transforms,
frame calibration, LiDAR leveling, boom kinematics, and PLC-RTC time synchronization. Everything
here is **pure Python (no ROS, no numpy for the core geometry/leveling/kinematics modules)** so it
runs identically on the Orin now and a future Xavier/ROS 2 integration later.

## Coordinate-frame contract (`calibration/README.md`)

Frame tree (station-mounted assumptions — reparent if hardware actually moves):

```text
rail_frame ──> docking_reference ──> docking_sensor_array_link ──> five sensor beam frames
     ├────> cutting_reference
     └────> base_link (dynamic rail-localisation transform)
                 ├────> docking_camera_link ──> docking_camera_optical_frame
                 ├────> cutting_camera_link ──> cutting_camera_optical_frame
                 └────> mid360_link
```

- `rail_frame` is the datum: origin at docking mechanical datum, +X along rail toward cutting,
  +Y left, +Z up (right-handed SI).
- Camera optical frames use the standard optical convention: +X image-right, +Y image-down,
  +Z forward through the lens.
- `T_parent_child` maps a point in `child` into `parent`: `p_parent = R @ p_child + t`.
- Config files: `calibration/frames.nominal.json` (illustrative, `simulation_only`) and
  `calibration/frames.deployment.template.json` (blank survey values, deployment-only).
  `sensor_telemetry_bindings` maps the five Pi keys (`diagonal_left_45deg`, `diagonal_right_45deg`,
  `center_line`, `c_channel_left`, `c_channel_right`) to beam frames.

Validation (`geometry/transforms.py::validate_configuration`): modes `planning` (blank values ok),
`simulation` (nominal numeric only), `deployment` (every static transform must be `verified`).
Run via `scripts/validate_frame_setup.py --mode <mode>`.

## Transforms (`geometry/transforms.py`)

- `rotation_from_rpy(rpy_rad)` → `Rz(yaw)@Ry(pitch)@Rx(roll)` (intrinsic RPY).
- `rotation_from_quaternion(x,y,z,w)` → normalized rotation matrix.
- `Transform` (`parent`, `child`, `translation_m`, `rotation`): `.apply`, `.inverse`, `.compose`.
- `FrameGraph(transforms)`: validates a static transform tree (no cycles, single parent),
  `.lookup(target, source)`, `.transform_point(point, source, target)`.
- `level_points_rotation_only(points, rotation)`: rotation-only leveling (no translation), the core
  operation that keeps the LiDAR at the HUD origin while static geometry stands straight.

## LiDAR leveling (`lidar/leveling.py`)

Gravity leveling is **rotation-only** and **yaw is deliberately unused** (IMU yaw drifts; the
2-axis tilt sensor has no yaw). Orientation sources (`--level-source`): `imu` (MID-360 built-in),
`tilt` (platform tilt sensor via PLC bridge), `boom` (boom angle via PLC bridge).

- `OrientationSource(name, quaternion, valid, timestamp_us)`.
- `quaternion_from_euler(roll, pitch, yaw=0)` — matches `rotation_from_rpy` convention.
- `level_orientation_quaternion(source)` → gravity rotation (raises `TransformConfigurationError`
  if `source.valid` is false, so a stale orientation never silently produces a wrong cloud).
- `level_points(points, source)`, `level_from_tilt_rpy(points, roll, pitch, ...)`.

`mid360_publisher.py` is the SDK-agnostic publisher (synthetic / livox-sdk / file modes); it levels
in-process and publishes a MessagePack envelope with `point_blob` (flat little-endian float32
`<f` triples), `point_count`, `point_stride=3`, plus `level_source`/`level_valid`. `lidar/livox_source.py`
is the **only** file that knows the Livox SDK; confirm the MID-360 vendor frame (+X forward / +Y left
/ +Z up project convention) and convert there.

## Boom kinematics (`lidar/boom_kinematics.py`)

Recovers the LiDAR's **height above ground** from PLC readings (no orientation — leveling uses IMU):

```text
ground -> base_link -> [yaw UNMEASURED] -> pivot angle (PLC) -> extension (PLC)
       -> platform tilt (PLC) -> [rail UNMEASURED] -> [cutting-arm lift: calibrated const]
       -> cutting_arm_base_link -> mid360_link
```

- `BoomGeometry(pivot_height_m, boom_stage0_length_m, platform_level_offset_m,
  cutting_arm_lift_offset_m)` — calibrated static offsets (never CAD guesses).
- `BoomState(pivot_angle_rad, extension_m, platform_pitch_rad, platform_roll_rad)`.
- `lidar_height_above_ground(state, geometry)` = `pivot_height + (stage0 + ext)*sin(pivot) +
  platform_level_offset*cos(pitch) + cutting_arm_lift_offset`.
- `height_of_point_above_ground(point_z_in_lidar, lidar_height)` = absolute Z for a leveled-cloud point.
- `examples/estimate_tree_height.py` chains leveling + kinematics for an absolute tree height
  (its trunk/canopy classifier is a placeholder).

## Time synchronization (`time_sync.py`)

- `PLC_NTP_SERVER = "192.168.50.40"`, `TIME_AUTHORITY = "plc_rtc_ntp"` — the PLC's battery-backed
  RTC is the persistent UTC authority; the Orin syncs via chrony.
- `depthai_timestamp_to_monotonic_us(td)` — DepthAI host-aligned `timedelta` → µs.
- `capture_timestamp_us(td, now_monotonic_ns, now_utc_ns)` — maps host-monotonic frame time to
  Unix-epoch µs by sampling the UTC↔monotonic offset at receipt (preserves capture time, not
  publish time).
- `ChronyStatus(refresh_seconds=30, expected_reference_ip=PLC_NTP_SERVER)` — best-effort cached
  `chronyc tracking` parse; `.get()` returns `(quality, offset_us)` with quality ∈
  `{synchronized, unexpected_source, holdover, unknown}`. Capture must never depend on it.

Deployment: `deploy/chrony/harvester-sensor.conf`, `deploy/networkmanager/orin-sensor-lan.nmconnection`
(assigns `192.168.50.10/24`, no default route). Verify with `scripts/validate_camera_time_sync.py`.

## Verify

```bash
cd ~/harvester_vision
PYTHONPATH=. python3 -m unittest discover -s tests -v   # test_transforms, test_boom_kinematics,
                                                        # test_leveling, test_time_sync, test_tree_height
python3 scripts/validate_frame_setup.py --mode planning
python3 scripts/validate_frame_setup.py --config calibration/frames.nominal.json --mode simulation \
  --source docking_camera_optical_frame --target docking_camera_link --point 0 0 1   # expect ~(1,0,0)
```

## Guardrails

- Never substitute CAD/photo estimates for measured calibration values; survey them (3-point or
  fiducial fixture).
- Never fuse a stale transform with a live measurement; reject `holdover`/unknown/uncalibrated
  inputs for geometry-dependent automation.
- Keep these modules dependency-free (no ROS, no numpy in `transforms`/`leveling`/`boom_kinematics`)
  so they run on both Orin and Xavier.
- `frames.nominal.json` is `simulation_only` and must never be used for deployment; deployment
  validation rejects it.
