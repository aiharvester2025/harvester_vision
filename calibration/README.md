# Harvester frames and calibration

This directory establishes the frame contract before perception or ROS 2 work begins. It is deliberately split into two configurations:

- `frames.deployment.template.json` is the only registry that may become a real deployment configuration. All unmeasured poses are `null`, so it cannot accidentally provide a plausible but wrong transform.
- `frames.nominal.json` has illustrative geometry for a Xavier/RViz mock-up. It is tagged `simulation_only` and is rejected by deployment validation.

## Frame tree

```text
rail_frame ──> docking_reference ──> docking_sensor_array_link ──> five sensor beam frames
     │
     ├────> cutting_reference
     │
     └────> base_link  (dynamic rail-localisation transform)
                ├────> docking_camera_link ──> docking_camera_optical_frame
                ├────> cutting_camera_link ──> cutting_camera_optical_frame
                └────> mid360_link
```

`rail_frame` is the station datum: origin at the docking mechanical datum, +X along the rail toward cutting, +Y left while looking in +X, and +Z up. Every mechanical frame uses that right-handed SI convention. Each camera optical frame uses the standard optical convention: +X image-right, +Y image-down, +Z forward through the lens.

The transform name `T_parent_child` always maps a point expressed in `child` into `parent`. Thus a valid range return is represented first as `[distance_m, 0, 0]` in its sensor beam frame, then transformed at the reading timestamp. An RGB pixel is *not* a metric 3-D point until it has a depth/range association and the intrinsics for the exact delivered image geometry.

The sensor array is currently assumed station-mounted, hence its parent is `docking_reference`. If it actually moves with the harvester, change its parent to `base_link` before any measurements are implemented and record that decision in the session file. The two OAKs are likewise tentatively base-mounted; the deployment template highlights the reparenting required if either is fixed to the station.

`camera_bindings` preserves the repository's `docking_camera` and `cutting_camera` roles, both currently using `CAM_A` at 1280×720. `sensor_telemetry_bindings` maps the existing Raspberry Pi telemetry keys (`diagonal_left_45deg`, `diagonal_right_45deg`, `center_line`, `c_channel_left`, `c_channel_right`) to their beam frames. Future publishers must emit the configured frame ID rather than recreating those mappings in a viewer or control loop.

## Safe use

```bash
# Structural check: incomplete survey entries are expected here.
python3 scripts/validate_frame_setup.py --mode planning

# RViz/Xavier-only nominal geometry. This is the only accepted use of this file.
python3 scripts/validate_frame_setup.py \
  --config calibration/frames.nominal.json --mode simulation

# Example: confirm the optical-axis convention (one metre forward from the lens).
python3 scripts/validate_frame_setup.py \
  --config calibration/frames.nominal.json --mode simulation \
  --source docking_camera_optical_frame --target docking_camera_link \
  --point 0 0 1
```

The last command must produce approximately `(1, 0, 0)`: forward in the optical frame is forward in the mechanical camera-link frame. A deployment check must fail until every physical transform is numeric and marked `verified`; that failure is intentional.

`geometry/transforms.py` is the dependency-free reference implementation. It has no ROS dependency, but the configured frame names and axes are suitable for `tf2`, URDF/Xacro, and RViz. Its `FrameGraph` stores only static transforms. The runtime must supply `T_rail_frame_base_link` from rail localisation at the sensor/camera timestamp, interpolate it according to the localisation estimator policy, and preserve a pose covariance.

`urdf/harvester_vision_nominal.urdf` is the matching RViz starter model. It contains only primitive rail/body/sensor geometry and a prismatic `rail_to_base_link` joint; set that joint to the nominal 1.5 m position in `joint_state_publisher_gui` for the illustrated starting pose. It is intentionally dependency-free and can be opened by the normal ROS 2 `robot_state_publisher`/RViz workflow once ROS 2 is installed on the Xavier. The unit test prevents its fixed joint origins from drifting from `frames.nominal.json`.

## Commissioning sequence

1. Copy the deployment template to a dated, version-controlled file. Never overwrite a previously approved calibration.
2. Establish and mark the physical rail and docking datums; measure the station/cutting references with a repeatable survey fixture.
3. Measure every rigid mount translation and orientation. Use at least a three-point mechanical survey or a surveyed fiducial fixture; do not estimate from a photo or CAD drawing.
4. For each OAK, archive the factory EEPROM calibration and request intrinsics/distortion for the actual CAM_A output geometry (currently 1280×720). Luxonis’ DepthAI v3 calibration API can read the device calibration and request intrinsics for a resized/cropped output; factory intrinsics alone do not establish camera-to-robot extrinsics. Do not modify EEPROM calibration for this system-level step.
5. Calibrate the OAK-to-LiDAR rigid transform with a surveyed target visible to both sensors, then validate it on held-out target poses. Calibrate each range sensor’s beam origin/orientation, zero offset, scale, valid range and variance against known distances.
6. Calibrate rail localisation (`rail_frame` to `base_link`) against the docking and cutting datums. Record pose covariance, timeout policy, and what happens when localisation is stale.
7. Fill `calibration-session.template.json`, attach raw measurements and validation results, set each accepted transform to `verified`, and obtain sign-off before enabling any geometry-dependent guidance.

## Runtime data contract

Every derived point, cloud, target, or control recommendation must carry:

- `timestamp_us` in the existing PLC-RTC UTC time domain and the original monotonic timestamp when available;
- `frame_id`, `calibration_id`, transform status, and a covariance/uncertainty estimate;
- source device identity (`camera_role`, OAK MXID, MID-360 serial, or sensor ID) and sequence number.

Never fuse a stale transform with a live measurement. The existing publisher already labels OAK capture time and synchronization quality; future camera/LiDAR/sensor messages should retain that contract and reject `holdover`, unknown, invalid, or uncalibrated inputs for geometry-dependent automation.

## DepthAI v3 boundary

This is an OAK/DepthAI v3 project. When the camera pipeline is next changed, migrate the currently used deprecated `ColorCamera` node to the v3 `Camera` node and obtain calibration from the actual device rather than inserting assumed intrinsics. The system setup above intentionally does not flash, reset, or otherwise alter OAK EEPROM calibration.
