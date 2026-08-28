# Plan: MID-360 3D LiDAR Integration (120° forward sector, bounded, on-demand)

## Goal

Bring the Livox MID-360 3D LiDAR into the canonical ZeroMQ stack so the
dashboard renders a live, gravity-leveled point cloud of the **forward 120°
sector only**, at a bounded point count and CPU cost, using the same
"bounded / visible-only / throttled" discipline that fixed the OAK depth path.

Concrete outcomes:

1. A new **canonical LiDAR producer** publishes `v1/lidar/raw`
   (`codec: lidar_xyz_f32`) into the existing aggregator, replacing the
   legacy `mid360_publisher.py` (which speaks a non-canonical MessagePack
   envelope on port 5560 and is not consumed by the canonical dashboard).
2. The **360° horizontal scan is filtered to a 120° forward sector**
   (±60° azimuth about `+X` forward) **at the source**, before it reaches the
   wire, so decode + bandwidth + projection + render all drop by ~2/3.
3. The dashboard reuses the existing `LidarDecoder`, `projection.py`, and
   `LidarInset.qml` (already schema-driven and vehicle-frame) with **no new
   QML**; only the producer and the leveling/sector filter are new.
4. CPU/memory stay bounded via point-count caps and a documented
   "enable cores 4-7" fallback (config-only, no code) if the LiDAR still
   saturates the 4 online cores.

## Verified hardware facts (Livox MID-360, from `livoxtech.com/mid-360/specs`)

| Parameter | Value | Planning impact |
|---|---|---|
| Horizontal FOV | 360° | 120° sector = 1/3 of points |
| Vertical FOV | -7° to +52° (59°) | tree trunk from below, canopy above |
| Point rate | 200,000 pts/s (first return) | ~20,000 pts per 10 Hz frame |
| Frame rate | 10 Hz (typical) | cap/throttle target |
| Range | 40 m @ 10% refl, 70 m @ 80% | far plane for range clipping |
| Blind zone | 0.1 m | drop < 0.1 m as invalid |
| Data link | 100 BASE-TX Ethernet | UDP ingest on the Orin |
| Sync | IEEE 1588 PTPv2 / GPS | optional; monotonic fallback |
| IMU | ICM40609 built-in | `--level-source imu` self-contained |
| Power | 6.5 W | negligible vs 15 W Orin budget |

**Sector math:** 200,000 pts/s ÷ 10 Hz = **20,000 pts per full frame**. A
120° sector = **~6,667 pts/frame** (uniform azimuth distribution). Capping to
`lidar_max_points` (existing default 2000) after sector filter further bounds
CPU to the same scale as today's synthetic 200-pt cloud, not 20k.

## What already exists (reuse, do not rebuild)

- `canonical_zmq/harvester_telemetry_contract/protocol.py` already validates
  `v1/lidar/raw` with `codec: lidar_xyz_f32`, required `point_count`,
  `point_stride_bytes`, and a non-empty `point_fields` list of
  `{name, type}` — **no contract change needed**.
- `canonical_zmq/canonical_zmq_publisher/aggregator.py` already forwards any
  canonical channel, owns per-channel sequence, and declares
  `lidar.raw_xyz` (plus `lidar.intensity`/`lidar.point_time`) capabilities.
- `canonical_zmq/canonical_zmq_publisher/ingest.py:synthetic_lidar_payload`
  already emits a canonical `v1/lidar/raw` packet with the exact header shape
  (`point_fields=[{x,y,z float32}]`, `point_stride_bytes=12`) — the producer
  mirrors this.
- `harvester_dashboard/harvester_dashboard/decoders/lidar_decoder.py`
  (`LidarDecoder.decode` + `.limit`) already decodes `lidar_xyz_f32` from the
  `point_fields` schema and uniformly downsamples — **no change**.
- `harvester_dashboard/harvester_dashboard/projection.py:project_points` and
  `qml/LidarInset.qml` already render vehicle-frame `(+X fwd, +Y left, +Z up)`
  in 5 views — **no change**.
- `lidar/leveling.py` (`level_points_rotation_only`, `rotation_from_quaternion`)
  and `lidar/boom_kinematics.py` already provide gravity leveling + height
  above ground — reused unchanged.
- `mid360_publisher.py` already has the SDK-mode scaffolding
  (`synthetic`/`livox-sdk`/`file`), control PULL enable/disable, and leveling
  plumbing — its *logic* is reused, but its *wire format* is replaced.

## What is missing / wrong

1. **`mid360_publisher.py` speaks a legacy non-canonical envelope**
   (`{point_blob, point_count, point_stride}` MessagePack, port 5560) that the
   canonical aggregator/dashboard do NOT consume. It must be rewritten to emit
   canonical three-frame packets (`pack_message('v1/lidar/raw', ...)`) and
   PUSH into the aggregator's ingest endpoint (`tcp://127.0.0.1:5570`).
2. **No 120° forward-sector filter exists.** The full 360° cloud is published.
3. **`lidar/livox_source.py` is a stub** — the actual Livox SDK2 point/IMU
   callbacks are not implemented (expected; hardware not yet available).
4. **No range clipping** — points beyond the 40 m detection range or inside the
   0.1 m blind zone are not dropped.

## Implementation

### 1. New canonical LiDAR producer (`canonical_zmq/canonical_zmq_publisher/lidar_capture.py`)

A new adapter that mirrors `oak_capture.py`'s structure (PUSH to the
aggregator ingest endpoint, `pack_message`, canonical header builders) and
absorbs the SDK-mode + leveling logic currently in `mid360_publisher.py`.

Key pieces:

- **CLI flags** (matching `mid360_publisher.py` plus new sector/range knobs):
  `--sdk-mode {synthetic,livox-sdk,file}`, `--hz 10`, `--max-points 2000`,
  `--frame-id vehicle_lidar_link`, `--level-source {imu,tilt,boom,none}`,
  `--sector-deg 120` (forward sector half-width ±60°), `--min-range-m 0.15`,
  `--max-range-m 40`, `--ingest-endpoint tcp://127.0.0.1:5570`, `--supervise`.
- **Header builder** `build_lidar_header(frame_id, point_count, ...)` producing
  the canonical shape already validated by the contract:
  ```
  {
    schema_version: 1, source_mode: 'hardware', source_id: 'orin',
    sequence: 0, frame_id: 'vehicle_lidar_link',
    acquisition_timestamp_ns, clock_domain: 'plc_rtc_utc',
    gateway_monotonic_ns: 0, calibration_id: 'mid360_v0',
    capabilities: {'lidar.raw_xyz': True, 'lidar.intensity': False,
                   'lidar.point_time': False, 'target.world_fixed': False},
    codec: 'lidar_xyz_f32',
    point_count, point_stride_bytes: 12,
    point_fields: [{name:'x',type:'float32'}, {name:'y',type:'float32'},
                   {name:'z',type:'float32'}],
  }
  ```
- **Sector filter** (the headline change): after leveling, drop points whose
  horizontal azimuth `atan2(y, x)` falls outside `±sector_degs/2` about
  `+X` forward. Implemented as a cheap Python filter on the already-leveled
  `(x, y, z)` list (no numpy needed in the producer, matching the existing
  dependency-free stance), OR in numpy if the SDK delivers a bulk array.
  Because the MID-360 has no per-ring "configure a sector" option, azimuth
  filtering is the correct approach.
- **Range clip**: drop `z < min_range_m` and `sqrt(x²+y²+z²) > max_range_m`.
- **Downsample**: reuse `mid360_publisher.downsample` (or `LidarDecoder.limit`
  logic) to cap at `--max-points` **after** sector + range filtering.
- **Leveling**: reuse `lidar.leveling.level_points_rotation_only` +
  `geometry.transforms.rotation_from_quaternion`, driven by the chosen
  `--level-source` (IMU self-contained; `tilt`/`boom` via PLC bridge later).
- **Run loop**: non-blocking control PULL (enable/disable, mirroring the
  existing `ctl` socket), acquire → level → sector-filter → range-clip →
  downsample → `pack_message` → PUSH.

### 2. SDK adapter (`lidar/livox_source.py` — implement when hardware arrives)

This is the ONLY file that touches the Livox SDK API (per its own docstring).
When the device is available, implement `sample()` against Livox-SDK2:

- Initialise the device + its UDP data callback and IMU callback.
- Buffer the latest full 10 Hz frame of points + the ICM40609 IMU quaternion.
- Convert from the **vendor frame** to the project frame `(+X fwd, +Y left,
  +Z up)` here (the mounting orientation is confirmed from the installed
  `livox_mid360` point struct + physical mount; the project convention is
  fixed in `geometry/transforms.py`).
- Return `(points, quaternion, valid, source_name)` — the same contract the
  stub already declares, so the producer needs no change.

**Until hardware arrives, `--sdk-mode synthetic` exercises the full canonical
path end-to-end** (sector filter, range clip, downsample, pack, PUSH, decode,
render) with the existing synthetic tree.

### 3. `run_all.sh` wiring

Add a `LIDAR` launch line (default on in `synthetic` mode for CI/manual
validation; `livox-sdk` once hardware is present), and a `LIDAR=${LIDAR:-1}`
env knob + `LIDAR_SECTOR_DEG=${LIDAR_SECTOR_DEG:-120}`.

```bash
LIDAR_CMD="PYTHONPATH=canonical_zmq:. ${DAI_PY} -m canonical_zmq_publisher.lidar_capture \
  --sdk-mode ${LIDAR_MODE:-synthetic} --ingest-endpoint tcp://127.0.0.1:5570 \
  --supervise --sector-deg ${LIDAR_SECTOR_DEG:-120} --max-points 2000"
```

### 4. Dashboard — no code changes expected

The existing `SocketDrainer.decode_frame` already dispatches `v1/lidar/raw` to
`LidarDecoder` (schema-driven), `bridge._set_lidar_points` already downsamples
and emits `lidar_points_changed`, and `LidarInset.qml` already renders it.
**No dashboard change is required** — the LiDAR cloud appears in the existing
inset automatically once the canonical producer publishes.

The one optional dashboard addition (defer unless needed): a small
"sector ±60°" annotation in the `LidarInset.qml` title so the operator sees the
active sector. Defer to keep this plan minimal.

### 5. Tests

Add `canonical_zmq/test/test_lidar_capture.py`:

- `test_build_lidar_header_is_canonical` — round-trips through
  `pack_message`/`unpack_message`; asserts `codec == 'lidar_xyz_f32'`,
  `point_stride_bytes == 12`, `point_fields` present.
- `test_sector_filter_drops_outside_forward` — a synthetic full ring: points
  with azimuth outside ±60° are dropped, inside are kept; count ≈ 1/3.
- `test_range_clip` — points inside 0.1 m and beyond 40 m are dropped.
- `test_downsample_caps_points` — `--max-points` caps the emitted count.
- `test_synthetic_produces_canonical_lidar` — end-to-end synthetic emission
  produces a valid `v1/lidar/raw` packet the aggregator accepts.

Extend `harvester_dashboard/test/test_decoders.py` (or add
`test_lidar_sector.py`) with a pure-Python test that a sector-filtered cloud
still round-trips through `LidarDecoder.decode` + `limit`.

### 6. Validation (manual, once hardware available)

1. Synthetic smoke (no hardware):
   `PYTHONPATH=canonical_zmq:. <dai> -m canonical_zmq_publisher.main --synthetic ...`
   already emits a canonical lidar packet; the dashboard shows the LiDAR inset.
2. Hardware: `LIDAR_MODE=livox-sdk ./run_all.sh foreground`; confirm the
   `v1/lidar/raw` stream in `scripts/validate_depth_stream.py`-style status
   shows ~6,667 pts/frame before cap and ≤2000 after cap.
3. `tegrastats` — confirm LiDAR adds bounded CPU (target: well under the
   current ~85% dashboard baseline + LiDAR ≈ acceptable). If CPU stays high,
   apply the fallback below.

### 7. CPU fallback (config-only, no code)

If the LiDAR pushes CPU too high after all the bounding/filtering:

1. **Enable cores 4-7** (currently offline):
   ```bash
   sudo nvpmodel -m 0          # MAXN: all 8 cores + max clocks
   sudo jetson_clocks           # pin to max freq (optional)
   cat /sys/devices/system/cpu/online   # expect 0-7
   ```
   Watch `tj` (junction temp) — stay well under 85°C; the current ~51°C gives
   large headroom. Trade-off: higher power/heat vs 4× cores.
2. **Lower `--hz`** (10 → 5) and/or `--max-points` (2000 → 1000) — the
   sector filter already cut ~2/3, these cut further with linear CPU savings.
3. **`LIDAR=0`** disables the LiDAR producer entirely (matches the OAK
   `DEPTH=0` opt-out pattern).

## Out of scope (explicit)

- **Point-cloud fusion** of LiDAR + OAK depth into one cloud (separate item;
  see `.kilo/plans/oak-depth-pointcloud.md` §9).
- **Tree-height estimation** (already prototyped in
  `examples/estimate_tree_height.py`; not wired to the live stream here).
- **PTPv2/GPS time sync** — use monotonic/chrony fallback first; PTP is a
  later accuracy refinement.
- **360° full-FOV operation** — the forward sector is a deliberate fixed
  design choice; a runtime-adjustable sector is a future enhancement.
- **ROS 2 / RViz** — out of scope; this is ZMQ-native only.

## Files touched

| File | Change |
|---|---|
| `canonical_zmq/canonical_zmq_publisher/lidar_capture.py` (new) | Canonical LiDAR producer: sector filter, range clip, downsample, leveling, PUSH to aggregator. |
| `lidar/livox_source.py` | Implement `sample()` against Livox-SDK2 (hardware-dependent; stub remains safe until then). |
| `run_all.sh` | Add `LIDAR`/`LIDAR_MODE`/`LIDAR_SECTOR_DEG` launch line + env knobs. |
| `canonical_zmq/test/test_lidar_capture.py` (new) | Header/sector/range/downsample/synthetic tests. |
| `mid360_publisher.py` | Mark legacy/deprecated (superseded by `lidar_capture.py`); keep for reference. |
| (dashboard) | No changes required — reuses `LidarDecoder` + `LidarInset.qml` as-is. |

## Risk + rollback

- **Risk**: sector filter assumes `+X` is forward *after* leveling. The leveling
  step (`level_points_rotation_only`) is rotation-only and preserves azimuth
  about the vertical, so filtering `atan2(y,x)` on the leveled cloud is correct.
  Verified against `lidar/leveling.py` conventions.
- **Risk**: the MID-360 azimuth distribution is not perfectly uniform (points
  are denser near the scan pattern edges); the sector count (~6,667) is an
  estimate. The `--max-points` cap is the hard bound that guarantees CPU
  regardless.
- **Rollback**: `LIDAR=0` disables the producer; the legacy `mid360_publisher.py`
  is untouched and still runs if needed. The dashboard is unchanged, so no
  consumer breaks.
- **Hardware-arrival gate**: the SDK implementation in `livox_source.py` is the
  only hardware-dependent piece; everything else is testable now via
  `--sdk-mode synthetic`.
