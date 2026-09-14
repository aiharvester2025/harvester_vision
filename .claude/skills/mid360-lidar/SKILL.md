---
name: mid360-lidar
description: "Livox Mid-360(S) LiDAR specialist: authoritative specs, downloads, SDK2/ROS integration, point cloud formats, and hardware facts."
disable-model-invocation: true
argument-hint: "the Mid-360(S) integration, hardware, or point-cloud task"
metadata:
  author: harvester
  version: "1.0.0"
  status: stable
---

# Livox Mid-360S LiDAR

Specialist reference for the Livox Mid-360 / Mid-360S LiDAR: authoritative hardware specs,
downloads, SDK2/ROS integration, point cloud formats, and safety facts.

> **Model note:** Livox lists the **Mid-360S** as the current product
> (`https://www.livoxtech.com/mid-360s`). It supersedes the Mid-360; both share the same
> Livox-SDK2 `MID360` config key and communication protocol. Specs below are the authoritative
> Mid-360S values from the official specs page; a "Legacy Mid-360" table is kept for reference.

## Authoritative specs (Mid-360S)

| Spec | Value |
|------|-------|
| Model | Mid-360S |
| Laser | 905 nm, Class 1 (IEC 60825-1:2014); divergence 25.2° (H) × 8° (V) FWHM |
| Range | 40 m @ 10% reflectivity (typical); 100 m cut-off |
| Blind zone | 0.1 m (0.1–0.2 m is reference-only) |
| FOV | 360° H × -7°~52° V (typical) |
| Range precision (1σ) | ≤ 2 cm @ 10 m; ≤ 4 cm @ 0.2 m |
| Angular precision (1σ) | < 0.15° |
| Point rate | 200,000 pts/s (first return) |
| Frame rate | 10 Hz (typical) |
| Data port | 100BASE-TX Ethernet |
| Sync | IEEE 1588-2008 (PTPv2), GPS |
| Anti-interference | Available; false alarm rate < 0.01% @ 100 klx |
| IMU | Built-in, ICM40609 |
| Operating temp | -20 °C to 55 °C |
| IP rating | IP67 |
| Power | 6.5 W typical (14 W peak self-heating at -20~0 °C) |
| Supply voltage | 9 ~ 27 V DC |
| Dimensions | 65 × 65 × 60 mm |
| Weight | 265 g |

Safety/thermal notes:
- Laser divergence 25.2° × 8°; max embedded laser power may exceed 70 W — **do NOT disassemble**.
- Keep shell temp below 80 °C (add heat dissipation in hot environments); a high-temperature
  protection mechanism auto-warns and shuts down.
- Self-heating below 0 °C peaks at 14 W — size the power supply accordingly.

## Downloads (authoritative)

| Asset | URL |
|---|---|
| User Manual (en) | https://terra-1-g.djicdn.com/65c028cd298f4669a7f0e40e50ba1131/Mid-360S/UM/20260601/Livox_Mid-360s_User_Manual_en.pdf |
| Product Information | https://terra-1-g.djicdn.com/65c028cd298f4669a7f0e40e50ba1131/Mid-360S/PI/Livox_Mid-360s_Product_Information.pdf |
| 3D model (.stp) | https://terra-1-g.djicdn.com/65c028cd298f4669a7f0e40e50ba1131/Mid-360S/Mid-360S%203D%20Model/mid-360S.stp |
| FOV model (.stp) | https://terra-1-g.djicdn.com/65c028cd298f4669a7f0e40e50ba1131/Mid-360S/Mid-360S%20FOV%E6%95%B0%E6%A8%A1/mid-360s_fov.stp |
| Livox Viewer 2 (Windows) | https://terra-1-g.djicdn.com/65c028cd298f4669a7f0e40e50ba1131/Mid-360S/Livox%20Viewer%202%20-%20Windows/Viewer2_2.5.9_Windows.zip |
| Livox Viewer 2 (Linux) | https://terra-1-g.djicdn.com/65c028cd298f4669a7f0e40e50ba1131/Mid-360S/Livox%20Viewer%202%20-%20Ubuntu/Viewer2_2.5.9_Linux.zip |
| Release Notes (EN) | https://terra-1-g.djicdn.com/65c028cd298f4669a7f0e40e50ba1131/Mid-360S/RN/Livox_Mid-360S_Release_Notes_EN_V35.1.0112.pdf |
| Firmware 35.01.0112 (.bin) | https://terra-1-g.djicdn.com/65c028cd298f4669a7f0e40e50ba1131/Mid-360S/Mid-360S%20%E5%9B%BA%E4%BB%B6/LIVOX_MID360S_FW_35.01.0112.bin |

Product pages: `/mid-360s` (overview), `/mid-360s/specs`, `/mid-360s/downloads`, `/mid-360s/faq`.

## Software stack

### Livox-SDK2 (supports Mid-360 and Mid-360S)

```bash
git clone https://github.com/Livox-SDK/Livox-SDK2.git
cd Livox-SDK2 && mkdir build && cd build
cmake .. && make -j && sudo make install
```

Installs `liblivox_lidar_sdk_*` to `/usr/local/lib` and `livox_lidar_api.h` to `/usr/local/include`.
Supports Linux (Ubuntu 18.04+), Windows 10/11, x86 + ARM. Samples: `livox_lidar_quick_start`,
`logger`, `multi_lidars_upgrade`.

### livox_ros_driver2

```bash
git clone https://github.com/Livox-SDK/livox_ros_driver2.git ws/src/livox_ros_driver2
source /opt/ros/humble/setup.sh && ./build.sh humble
```

### Communication protocol (authoritative)

Mid-360(S) communication protocol (control commands + data format):
https://livox-wiki-en.readthedocs.io/en/latest/tutorials/new_product/mid360/mid360.html

## SDK2 config (MID360 key — shared by Mid-360 and Mid-360S)

```json
{
  "MID360": {
    "lidar_net_info": {
      "cmd_data_port": 56100, "push_msg_port": 56200,
      "point_data_port": 56300, "imu_data_port": 56400, "log_data_port": 56500
    },
    "host_net_info": [{
      "lidar_ip": ["192.168.1.3"], "host_ip": "192.168.1.5",
      "multicast_ip": "224.1.1.5",
      "cmd_data_port": 56101, "push_msg_port": 56201,
      "point_data_port": 56301, "imu_data_port": 56401, "log_data_port": 56501
    }]
  }
}
```

Optional fields: `master_sdk` (true = master sends control cmds + receives data; false = slave
receives point cloud only; exactly ONE master SDK), `lidar_log_enable`,
`lidar_log_cache_size_MB`, `lidar_log_path`, `multicast_ip`.

## Network ports (SDK2 default)

| Data type | LiDAR port | Host port |
|-----------|-----------|-----------|
| Command/control | 56100 | 56101 |
| Push message | 56200 | 56201 |
| Point cloud | 56300 | 56301 |
| IMU | 56400 | 56401 |
| Firmware log | 56500 | 56501 |

## Point cloud data types

- **Type 1** (default): Cartesian 32-bit — x, y, z in mm, reflectivity, tag
- **Type 2**: Cartesian 16-bit — x, y, z in 10 mm, reflectivity, tag
- **Type 3**: Spherical — depth (mm), theta/phi (0.01°), reflectivity, tag
- **Type 0**: IMU — gyro x/y/z (rad/s), acc x/y/z (g)

Tag bits: bit 0-1 glue-point confidence, bit 2-3 rain/fog/dust confidence, bit 4-5 detection
confidence, bit 6-7 reserved.

## SDK2 key APIs (`include/livox_lidar_api.h`)

```c
bool LivoxLidarSdkInit(const char* path, const char* host_ip, const LivoxLidarLoggerCfgInfo* log_cfg);
bool LivoxLidarSdkStart();
void LivoxLidarSdkUninit();
void SetLivoxLidarPointCloudCallBack(LivoxLidarPointCloudCallBack cb, void* client_data);
void SetLivoxLidarImuDataCallback(LivoxLidarImuDataCallback cb, void* client_data);
livox_status SetLivoxLidarPclDataType(uint32_t handle, LivoxLidarPointDataType data_type, ...);
livox_status SetLivoxLidarScanPattern(uint32_t handle, LivoxLidarScanPattern scan_pattern, ...);
livox_status EnableLivoxLidarPointSend(uint32_t handle, ...);
livox_status EnableLivoxLidarImuDataSend(uint32_t handle, ...);
```

## Time sync

- **PTP**: IEEE 1588v2 (1588-2008) over UDP/IP (NOT 1588v2.1)
- **gPTP**: Automotive Ethernet L2
- **GPS**: PPS + GPRMC (valid 2000–2037)

Do NOT mix PTP and gPTP.

## Safety

- NO PoE on RJ-45.
- Avoid overlapping FOVs between units.
- Shell temp must stay below 80 °C.
- Self-heating at -20 °C to 0 °C (peak 14 W).
- IP67, Class 1 laser — do not disassemble.

## Project context (this repo)

`mid360_publisher.py` publishes leveled points over ZMQ (synthetic/livox-sdk/file modes);
`lidar/livox_source.py` is the **only** file that should know the Livox SDK API (return
`(points, quaternion, valid, source_name)` per scan, converting the MID-360 vendor frame to the
project's +X forward / +Y left / +Z up convention). `lidar/leveling.py` does gravity leveling;
`lidar/boom_kinematics.py` computes LiDAR height above ground. See the `harvester-geometry-time`
skill for those software layers.

## Legacy Mid-360 (superseded by Mid-360S)

| Spec | Value |
|------|-------|
| Model | MID-360 |
| Laser | 905 nm, Class 1 |
| Range | 40 m @ 10%, 70 m @ 80% |
| Blind zone | 0.1 m |
| FOV | 360° H × -7°~52° V |
| Precision | ≤ 2 cm @ 10 m |
| Point rate | 200,000 pts/s |
| Frame rate | 10 Hz |
| Interface | 100BASE-TX Ethernet |
| Sync | IEEE 1588v2 (PTP), GPS |
| IMU | ICM40609 |
| Temp | -20 °C to 55 °C |
| IP | IP67 |
| Power | 6.5 W avg, 9–27 V DC |
| Size/weight | 65×65×60 mm, 265 g |

Note: the Mid-360S drops "70 m @ 80%" in favor of "40 m @ 10% / 100 m cut-off" and adds
anti-interference + explicit false-alarm-rate specs. Verify against the current specs page before
relying on any single figure for a deployment decision.
