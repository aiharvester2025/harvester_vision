# VIO Arm Pose Estimation + World-Anchored Marker Tracking

## Goal

Track the cutting arm's 6-DOF pose in world space using IMU + camera VO + LiDAR scan matching (no magnetic encoder, no AprilTags). Use the tree trunk as the absolute yaw anchor. Keep the dashboard marker fixed to a 3D world point while the camera moves.

## Sensor roles

| Sensor | Rate | Provides | Mount |
|--------|------|----------|-------|
| OAK IMU (accel + gyro) | ~200 Hz | Gravity-referenced pitch/roll, yaw rate | `cutting_arm_base_link` |
| MID-360 IMU | ~200 Hz | Same as OAK IMU (redundant or fused) | `cutting_arm_base_link` |
| MID-360 LiDAR | 10 Hz | Scan-to-scan relative 6-DOF, trunk detection | `cutting_arm_base_link` |
| OAK RGB stereo | 15 Hz | Feature-based visual odometry (relative 6-DOF) | `cutting_arm_base_link` |

All four sensors are rigidly fixed to the same link, so they measure the same rigid-body motion. Fusion is 1-DOF redundancy (yaw) + noise averaging, not decomposition.

## Architecture

```
Sensors (on cutting_arm_base_link)
  │
  ├─ IMU ──► imu_bridge ──► v1/imu/arm
  │
  ├─ LiDAR ──► mid360_publisher ──► v1/lidar/raw
  │
  └─ OAK camera ──► oak_capture ──► v1/camera/cutter/rgb
                        │
                        └─ camera_vo ──► internal (not a separate channel)

VIO fusion node (new: arm_pose_estimator)
  ├─ subscribes: v1/imu/arm, v1/lidar/raw, v1/camera/cutter/rgb
  ├─ IMU gyro integrates yaw rate between updates (~200 Hz prediction)
  ├─ LiDAR scan matching corrects drift (10 Hz)
  ├─ Camera VO corrects drift (15 Hz)
  ├─ Trunk detection provides absolute yaw anchor
  └─ publishes: v1/pose/arm (T_world_cutting_arm_base_link + covariance)

Dashboard (harvester_dashboard)
  ├─ receives: v1/pose/arm, v1/camera/cutter/rgb, v1/lidar/raw
  ├─ stores world-fixed markers from operator clicks
  ├─ each frame: reprojects markers through current T_world_camera
  └─ draws overlay at projected [u, v]
```

## Data flow: one marker from click to screen

```
1. Operator clicks (u0, v0) on cutter camera image
   └─ depth at (u0, v0) from depth_decoder or LiDAR projection
   └─ K⁻¹ @ [u0, v0, 1] * depth = P_camera_optical (3D point in camera frame)

2. Transform to world frame using initial arm pose
   └─ P_world = T_world_arm_initial @ P_camera_optical
   └─ store as world-fixed marker (never changes)

3. Each new frame:
   └─ T_world_arm_new = latest from v1/pose/arm
   └─ T_world_camera = T_world_arm @ T_arm_camera_optical  (static offset)
   └─ P_cam_new = T_world_camera.inverse() @ P_world
   └─ [u_new, v_new] = K @ P_cam_new[:2] / P_cam_new[2]
   └─ draw marker at (u_new, v_new)
```

## Components to build (in execution order)

### 1. Camera intrinsics calibration (prerequisite)

**Status:** `intrinsics_artifact: pending` in `calibration/frames.deployment.template.json`

Run OpenCV `calibrateCamera` with a checkerboard for both OAK cameras. Record `fx, fy, cx, cy, distortion_coefficients` in the calibration JSON. Without `K`, no 2D→3D conversion is possible.

### 2. IMU bridge module

**New file:** `canonical_zmq/canonical_zmq_publisher/imu_bridge.py`

- Read IMU from OAK (`dai.node.IMU`) or MID-360 SDK
- Publish as canonical `v1/imu/arm` with header fields: `accelerometer_m_s2`, `gyroscope_rad_s`, `timestamp_us`, `frame_id: cutting_arm_base_link`
- Rate-limit to ~200 Hz output (sensor is higher)
- Reuse timestamp pattern from `oak_capture.py` (`time_sync.capture_timestamp_us`)

### 3. Tree trunk detection (LiDAR)

**New file:** `lidar/trunk_detector.py`

- Receive leveled point cloud from `mid360_publisher`
- RANSAC cylinder fit or simple radius+height clustering to find trunk center in `mid360_link` frame
- Output: `trunk_center_xyz` + `trunk_direction_yaw` (angle of trunk center in X-Y plane)
- Published as `v1/docking/trunk_estimate` (extends existing JSON channel)

### 4. VIO fusion node

**New file:** `canonical_zmq/canonical_zmq_publisher/arm_pose_estimator.py`

Subscribes to `v1/imu/arm`, `v1/lidar/raw`, `v1/camera/cutter/rgb`.

**Prediction (IMU-driven, runs at IMU rate ~200 Hz):**
```
yaw_rate = gyro[2]  (Z-axis rotation rate)
yaw_integral += yaw_rate * dt
T_world_arm.predicted_yaw = yaw_integral
pitch, roll from accelerometer gravity vector
```

**Correction (runs at sensor rate when data arrives):**

- **LiDAR path:** `scan_matching(scan_t, scan_t-1)` → relative 6-DOF transform → accumulate → drift-correct the IMU integral
- **Camera VO path:** feature detect (ORB/KLT) → essential matrix → relative 6-DOF → accumulate → drift-correct
- **Trunk anchor path:** trunk center direction is fixed in world frame → compute absolute yaw = `atan2(trunk_y, trunk_x)` → hard reset IMU yaw integral

**Publish:** `v1/pose/arm` as JSON with:
```json
{
  "frame_id": "cutting_arm_base_link",
  "parent_frame": "world",
  "translation_m": [x, y, z],
  "rotation_rpy_rad": [roll, pitch, yaw],
  "covariance": [...],
  "source": "vio_fusion",
  "yaw_anchor": "trunk" | "imu_integral"
}
```

### 5. Camera VO module (lightweight)

**New file:** `canonical_zmq/canonical_zmq_publisher/camera_vo.py`

- Decode OAK H.265/H.264 stream via existing Jetson decoder or extract from depthai pipeline directly
- ORB feature detection + matching between consecutive frames
- Essential matrix decomposition → relative R, t
- Scale ambiguity resolved by LiDAR or IMU acceleration
- Output: relative 6-DOF transform + inlier count

**Simpler alternative:** Use the existing Gazebo simulation to prototype VO offline with `oil_palm_harvester_description` before implementing on hardware.

### 6. LiDAR scan matching

**New file:** `lidar/scan_matcher.py`

- Downsample point cloud (already done in `mid360_publisher.py` at 2000 points)
- Use PCL ICP or Open3D `registration_icp` for point-to-plane matching
- The tree trunk at ~1 m provides a strong geometric anchor
- Output: `T_relative` (6-DOF) between consecutive scans

**Dependencies:** Add `open3d` or `pclpy` to `depthai-env` requirements.

### 7. World-fixed marker storage and reprojection (dashboard)

**New file:** `harvester_dashboard/harvester_dashboard/world_markers.py`

```python
class WorldMarker:
    position_world: Tuple[float, float, float]  # fixed
    label: str
    created_timestamp: float

class WorldMarkerStore:
    def add_from_click(self, u, v, depth_m, T_world_camera, K) -> WorldMarker
    def project_all(self, T_world_camera, K) -> List[Tuple[float, float]]  # (u, v) screen coords
    def prune_stale(self, max_age_s) -> None
```

**Integration point:** `harvester_dashboard/model/telemetry_model.py` — extend `TelemetryModel` to track the latest `v1/pose/arm` header and compute `T_world_camera` from it.

**Overlay render loop** (in the viewer widget):
```python
# Each rendered frame:
T_world_camera = compute_current_camera_pose(model)
screen_points = marker_store.project_all(T_world_camera, K)
for (u, v) in screen_points:
    draw_circle_overlay(u, v)
```

### 8. Dynamic pose from joint state (Gazebo / simulation path)

**New file:** `geometry/dynamic_pose.py`

For simulation, build `T_world_camera` from URDF joint state publisher + forward kinematics through the static transform graph in `geometry/transforms.py`. This is the simulation reference that the VIO estimate must track.

```python
def compute_camera_pose_from_joints(joint_state: dict, frame_graph: FrameGraph) -> Transform:
    """Build T_world_cutting_camera_optical from measured joint angles."""
    # Compose dynamic transforms:
    #   T_world_rail = rail_position + heading
    #   T_rail_base = static (surveyed)
    #   T_base_turret = from boom_pivot_angle_rad (PLC)
    #   ... down to cutting_arm_base_link
    #   T_arm_camera = static (surveyed mount offset)
    pass
```

## Calibration prerequisites

| Calibration | Current status | Required for |
|-------------|---------------|--------------|
| OAK camera intrinsics (`fx, fy, cx, cy`) | `pending` | 2D click → 3D ray, VO, marker reprojection |
| OAK camera-to-arm extrinsics (`T_arm_camera_link`) | `pending` | World-frame pose of camera |
| MID-360-to-arm extrinsics (`T_arm_mid360_link`) | `pending` | Trunk detection in world frame |
| Static survey transforms | `null` | Frame graph composition |

All four must be measured and recorded before runtime deployment. The Gazebo simulation uses `frames.nominal.json` for development but it is explicitly marked `simulation_only` and is rejected by deployment validation.

## Failure modes and mitigations

| Failure | Symptom | Mitigation |
|---------|---------|-----------|
| IMU drift (no correction for >1 s) | Marker slides off target | Trunk anchor runs every LiDAR cycle (10 Hz) — drift never accumulates |
| Trunk not visible (occluded) | No absolute yaw reset | IMU integral holds yaw; VO/LiDAR still correct relative drift. Degrade gracefully — show yaw uncertainty on HUD |
| VO failure (textureless bark, motion blur) | No camera correction | LiDAR scan matching continues at 10 Hz. IMU prediction fills gaps |
| Scan matching failure (sparse points) | No LiDAR correction | VO + IMU continue. Trunk anchor recovers on next visible scan |
| Depth missing at click point | Cannot create 3D marker | Fall back to LiDAR-projected depth at same pixel, or require depth before accepting annotation |
| Camera intrinsics not calibrated | Wrong 3D position, marker projects incorrectly | Block annotation creation until `intrinsics_artifact` is `verified` in calibration JSON |

## Validation plan

1. **Gazebo simulation first:** Run `gazebo_harvester_and_tree.launch.py` with `articulation_control_mode:=kinematic`. Drive the arm with `joint_state_publisher_gui`. Verify VIO estimates track the Gazebo ground-truth joint state.
2. **Synthetic data replay:** Replay a recorded `.lvx2` + OAK stream through the fusion node. Compare estimated pose against surveyed markers.
3. **Static trunk test:** Arm moves through full range of motion while trunk is always visible. Verify yaw never drifts more than ±2° between trunk anchor resets.
4. **Occlusion test:** Block trunk from LiDAR for 3 seconds. Verify IMU+VO hold yaw within ±5° and re-anchor when trunk reappears.
5. **Marker projection test:** Place 3 world-fixed markers in Gazebo. Click each on screen. Verify projected marker stays within 3 pixels of true image projection during arm motion.

## Out of scope

- Magnetic rotary encoder (excluded by premise)
- AprilTags or other visual fiducials (tree trunk is the reference)
- 2-axis tilt sensor (replaced by IMU pitch/roll)
- OAK depth channel integration (depth comes from LiDAR projection or OAK depth once pipeline is extended)
- Arm actuation or control commands (observation-only, same safety boundary as existing stack)

## Dependencies to add to depthai-env

```
open3d>=0.18    # LiDAR scan matching (ICP)
opencv-python   # VO feature detection + camera calibration tool
numpy           # already present
scipy           # RANSAC helpers (optional, for trunk cylinder fit)
```

## Files changed

| File | Action |
|------|--------|
| `calibration/frames.deployment.template.json` | Fill intrinsics + extrinsics (survey required) |
| `canonical_zmq/canonical_zmq_publisher/imu_bridge.py` | New — IMU → canonical ZMQ |
| `canonical_zmq/canonical_zmq_publisher/arm_pose_estimator.py` | New — VIO fusion node |
| `canonical_zmq/canonical_zmq_publisher/camera_vo.py` | New — lightweight feature VO |
| `lidar/trunk_detector.py` | New — trunk center from point cloud |
| `lidar/scan_matcher.py` | New — ICP scan-to-scan matching |
| `geometry/dynamic_pose.py` | New — joint-state → camera pose |
| `geometry/transforms.py` | Extend — add `Transform.from_rpy` convenience, velocity integration helpers |
| `harvester_dashboard/harvester_dashboard/world_markers.py` | New — marker store + reprojection |
| `harvester_dashboard/harvester_dashboard/model/telemetry_model.py` | Extend — track `v1/pose/arm` |
| `harvester_dashboard/harvester_dashboard/annotation_publisher.py` | Extend — set `world_fixed: true`, include `world_xyz` when pose is available |
| `docs/orin_canonical_zmq.md` | Update — document new channels and run instructions |
| `run_all.sh` | Update — start imu_bridge + arm_pose_estimator alongside existing adapters |
| `.kilo/plans/orin-oak-adapter-plan.md` | Update — mark depth/IMU scope change |
