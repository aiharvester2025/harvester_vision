#!/usr/bin/env python3
"""OAK camera capture adapter for the canonical telemetry bus.

Encodes one OAK camera (docking or cutter) with the OAK hardware H.264/H.265
encoder and PUSHes canonical three-frame packets into the canonical aggregator's
PULL socket.  It never binds the canonical ``5590`` PUB endpoint — the aggregator
is the sole owner of that endpoint.

Quality is controlled by the encoder ``setQuality(1-100)`` rate control (higher
= better), following the depthai_sdk pattern.  Do NOT use ``setBitrateKbps()``
on H.264/H.265: it pushes the OAK firmware encoder into an invalid state that
crashes with "stack smashing" in ``libdepthaicore``.

Data flow::

    OAK (DepthAI hw encode) -> canonical frames -> PUSH -> aggregator PULL
                                                             -> PUB tcp://*:5590
                                                             -> dashboard SUB

Run one process per camera role (see the Orin handoff doc)::

    PYTHONPATH=canonical_zmq python3 -m canonical_zmq_publisher.oak_capture \\
        --camera-role docking_camera --ingest-endpoint tcp://127.0.0.1:5570

Requires the depthai-env python (``depthai``, ``zmq``, ``msgpack``).
"""

from __future__ import annotations

import argparse
import math
import os
import signal
import subprocess
import sys
import time
from datetime import timedelta
from typing import Optional

import zmq

from camera_config import CAMERAS, camera_choices
from time_sync import (
    TIME_AUTHORITY,
    ChronyStatus,
    capture_timestamp_us,
)
from harvester_telemetry_contract import pack_message

# ---------------------------------------------------------------------------
# Codec helpers
# ---------------------------------------------------------------------------

CODEC_PROFILES = {
    'h264': 'H264_MAIN',
    'h265': 'H265_MAIN',
    'jpeg': 'MJPEG',
}

# Annex-B NAL unit types that mark an H.264/H.265 keyframe (IDR / IRAP).
_H264_IDR_TYPES = frozenset({5})          # 5 = IDR slice
_H265_IRAP_TYPES = frozenset({16, 19, 20, 21, 32, 33, 34})  # BLA/IDR/CRA


def detect_keyframe(codec: str, payload: bytes) -> bool:
    """Best-effort keyframe detection from the Annex-B NAL unit type.

    DepthAI v3 does not expose an explicit keyframe flag on the encoded
    ``ImgFrame``, so we inspect the first NAL unit.  This is reliable for
    H.264/H.265 Annex-B output; JPEG packets are always self-contained
    keyframes.
    """
    if codec == 'jpeg':
        return True
    # A keyframe access unit contains SPS/PPS followed by an IDR/IRAP slice, so
    # scan every NAL unit and return True if any is a keyframe NAL type.
    keyframe_types = _H264_IDR_TYPES if codec == 'h264' else _H265_IRAP_TYPES
    i = 0
    length = len(payload)
    while i < length - 4:
        if payload[i:i + 4] == b'\x00\x00\x00\x01':
            header_index = i + 4
            i += 4
        elif payload[i:i + 3] == b'\x00\x00\x01':
            header_index = i + 3
            i += 3
        else:
            i += 1
            continue
        if header_index >= length:
            break
        nal_type = (payload[header_index] & 0x1F) if codec == 'h264' else (
            (payload[header_index] >> 1) & 0x3F)
        if nal_type in keyframe_types:
            return True
    return False


# ---------------------------------------------------------------------------
# Adapter
# ---------------------------------------------------------------------------

# Canonical channel names are frozen by the contract: the camera role names
# ``docking_camera``/``cutting_camera`` map to ``docking``/``cutter``.
ROLE_TO_CHANNEL = {
    'docking_camera': 'docking',
    'cutting_camera': 'cutter',
}


def gravity_to_rpy(accel_ms2):
    """Return ``(roll_rad, pitch_rad, accel_norm_ms2)`` from an accelerometer
    reading in the camera optical frame (+X right, +Y down, +Z forward).

    At rest the accelerometer measures the reaction to gravity: the specific
    force is ``-g`` along the vertical, so the gravity direction in the body
    frame is ``-a / |a|``.  Roll and pitch are the small-angle tilts of that
    gravity direction about the optical Z/X axes.  This mirrors
    ``harvester_dashboard/decoders/imustab.py:gravity_to_rpy`` (the dashboard
    imports its own copy so the adapter never depends on the dashboard module).
    """
    ax, ay, az = accel_ms2[0], accel_ms2[1], accel_ms2[2]
    norm = math.sqrt(ax * ax + ay * ay + az * az)
    if norm <= 0.0 or not math.isfinite(norm):
        return 0.0, 0.0, 0.0
    # Gravity vector in the body frame (unit length).
    gx = -ax / norm
    gy = -ay / norm
    gz = -az / norm
    # roll: rotation about +Z (optical axis); pitch: rotation about +X.
    roll = math.atan2(gy, gz)
    pitch = math.atan2(-gx, math.sqrt(gy * gy + gz * gz))
    return roll, pitch, norm


def _read_imu_to_cam_rotation(device, dai):
    """Return the 3x3 IMU -> camera rotation (list of 9 floats), or identity.

    RAW accelerometer/gyroscope samples are in the IMU sensor frame; this
    rotation maps them into the camera optical frame (+X right, +Y down, +Z
    forward) using the factory ``imuExtrinsics``.  Falls back to identity when
    the device does not expose the transform (the IMU is physically mounted
    close to the optical frame, so identity is a reasonable default).
    """
    identity = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
    try:
        extrinsics = device.readCalibration().getImuToCameraExtrinsics(
            dai.CameraBoardSocket.CAM_A, False)
        rotation = extrinsics[:3, :3]
        return [float(v) for row in rotation for v in row]
    except Exception:
        return identity


def _rotate_vec3(matrix, vec):
    """Apply a row-major 3x3 rotation (list of 9) to a length-3 vector."""
    (m00, m01, m02, m10, m11, m12, m20, m21, m22) = matrix
    x, y, z = vec[0], vec[1], vec[2]
    return (
        m00 * x + m01 * y + m02 * z,
        m10 * x + m11 * y + m12 * z,
        m20 * x + m21 * y + m22 * z,
    )



def canonical_channel(camera_role):
    """Return the canonical ``v1/camera/<name>/rgb`` channel for a camera role."""
    name = ROLE_TO_CHANNEL.get(camera_role, camera_role)
    return 'v1/camera/{}/rgb'.format(name)


def canonical_depth_channel(camera_role):
    """Return the canonical ``v1/camera/<name>/depth`` channel for a role."""
    name = ROLE_TO_CHANNEL.get(camera_role, camera_role)
    return 'v1/camera/{}/depth'.format(name)


def canonical_camera_info_channel(camera_role):
    """Return the canonical ``v1/camera/<name>/camera_info`` channel for a role."""
    name = ROLE_TO_CHANNEL.get(camera_role, camera_role)
    return 'v1/camera/{}/camera_info'.format(name)


def canonical_imu_channel(camera_role):
    """Return the canonical ``v1/camera/<name>/imu`` channel for a role."""
    name = ROLE_TO_CHANNEL.get(camera_role, camera_role)
    return 'v1/camera/{}/imu'.format(name)


def build_capabilities(camera_role, depth=False, camera_info=False, imu=False):
    """Return the per-camera ``capabilities`` dict shared by all headers.

    Depth, camera_info, and imu start disabled and flip to ``True`` once the
    adapter actually publishes those channels, so a consumer can tell
    whether the source really delivers them before relying on them.
    """
    name = ROLE_TO_CHANNEL.get(camera_role, camera_role)
    return {
        'camera.{}.rgb'.format(name): True,
        'camera.{}.depth'.format(name): bool(depth),
        'camera.{}.camera_info'.format(name): bool(camera_info),
        'camera.{}.imu'.format(name): bool(imu),
        'target.world_fixed': False,
    }


def build_rgb_header(camera_role, codec, width, height, keyframe,
                     acquisition_timestamp_ns, frame_id):
    """Return a canonical header for a ``v1/camera/<name>/rgb`` packet."""
    name = ROLE_TO_CHANNEL.get(camera_role, camera_role)
    channel = canonical_channel(camera_role)
    return {
        'schema_version': 1,
        'source_mode': 'hardware',
        'source_id': 'orin',
        'sequence': 0,  # owned by the aggregator
        'frame_id': frame_id,
        'acquisition_timestamp_ns': acquisition_timestamp_ns,
        'clock_domain': 'plc_rtc_utc',
        'gateway_monotonic_ns': 0,  # owned by the aggregator
        'calibration_id': 'oak_{}_v0'.format(name),
        'capabilities': build_capabilities(camera_role),
        'codec': codec,
        'pixel_encoding': {'h264': 'H264', 'h265': 'H265', 'jpeg': 'RGB8'}[codec],
        'width': int(width),
        'height': int(height),
        'keyframe': bool(keyframe),
    }


def build_depth_header(camera_role, width, height, acquisition_timestamp_ns,
                       frame_id, publish_camera_info=False):
    """Return a canonical header for a ``v1/camera/<name>/depth`` packet.

    Depth is a per-frame independently-decodable uint16 millimetre plane
    (codec ``depth_uint16_le``); there is no ``pixel_encoding`` and no
    inter-frame state, so ``keyframe`` is always True.
    """
    name = ROLE_TO_CHANNEL.get(camera_role, camera_role)
    return {
        'schema_version': 1,
        'source_mode': 'hardware',
        'source_id': 'orin',
        'sequence': 0,  # owned by the aggregator
        'frame_id': frame_id,
        'acquisition_timestamp_ns': acquisition_timestamp_ns,
        'clock_domain': 'plc_rtc_utc',
        'gateway_monotonic_ns': 0,  # owned by the aggregator
        'calibration_id': 'oak_{}_v0'.format(name),
        'capabilities': build_capabilities(
            camera_role, depth=True, camera_info=publish_camera_info),
        'codec': 'depth_uint16_le',
        'width': int(width),
        'height': int(height),
        'keyframe': True,
    }


def build_camera_info_header(camera_role, width, height, acquisition_timestamp_ns,
                             frame_id, publish_depth=False):
    """Return a canonical header for a ``v1/camera/<name>/camera_info`` packet."""
    name = ROLE_TO_CHANNEL.get(camera_role, camera_role)
    return {
        'schema_version': 1,
        'source_mode': 'hardware',
        'source_id': 'orin',
        'sequence': 0,  # owned by the aggregator
        'frame_id': frame_id,
        'acquisition_timestamp_ns': acquisition_timestamp_ns,
        'clock_domain': 'plc_rtc_utc',
        'gateway_monotonic_ns': 0,  # owned by the aggregator
        'calibration_id': 'oak_{}_v0'.format(name),
        'capabilities': build_capabilities(
            camera_role, depth=publish_depth, camera_info=True),
        'codec': 'json',
        'width': int(width),
        'height': int(height),
    }


def build_imu_header(camera_role, acquisition_timestamp_ns, frame_id):
    """Return a canonical header for a ``v1/camera/<name>/imu`` packet.

    IMU has no image geometry, so there is no ``width``/``height`` and the
    payload is JSON (``codec='json'``).  The ``camera.<name>.imu`` capability
    is flipped on to advertise that this adapter publishes IMU samples.
    """
    name = ROLE_TO_CHANNEL.get(camera_role, camera_role)
    return {
        'schema_version': 1,
        'source_mode': 'hardware',
        'source_id': 'orin',
        'sequence': 0,  # owned by the aggregator
        'frame_id': frame_id,
        'acquisition_timestamp_ns': acquisition_timestamp_ns,
        'clock_domain': 'plc_rtc_utc',
        'gateway_monotonic_ns': 0,  # owned by the aggregator
        'calibration_id': 'oak_{}_v0'.format(name),
        'capabilities': build_capabilities(camera_role, imu=True),
        'codec': 'json',
    }


class OakCapture:
    """DepthAI pipeline that PUSHes canonical RGB frames to an aggregator."""

    # v3 StereoDepth presets (RVC2).  FAST_DENSITY/FAST_ACCURACY are the v3
    # closest to the legacy "high density"/"high accuracy" presets; see
    # https://docs.luxonis.com/software-v3/depthai/depthai-components/nodes/stereo_depth.md
    STEREO_PROFILES = ('fast_density', 'fast_accuracy', 'high_detail',
                       'robotics', 'default')

    def __init__(self, camera_role, ingest_endpoint='tcp://127.0.0.1:5570',
                 codec='jpeg', width=1920, height=1080, fps=15.0,
                 quality=90, keyframe_frequency=30, ev_compensation=7,
                 brightness=0, contrast=0, saturation=0, sharpness=0,
                 max_exposure_us=0, depth=True, stereo_profile='fast_density',
                 stereo_lr_check=True, depth_fps=5.0, imu=True,
                 imu_rate=50.0, imu_fps=5.0):
        if camera_role not in CAMERAS:
            raise ValueError('unknown camera role {!r}'.format(camera_role))
        if codec not in CODEC_PROFILES:
            raise ValueError('unsupported codec {!r}'.format(codec))
        if stereo_profile not in self.STEREO_PROFILES:
            raise ValueError('unsupported stereo profile {!r}'.format(stereo_profile))
        self.camera_role = camera_role
        self.camera = CAMERAS[camera_role]
        self.codec = codec
        self.width = int(width)
        self.height = int(height)
        self.fps = float(fps)
        # Depth (stereo) publishing.  Additive: RGB is never affected by
        # these flags; ``depth=False`` keeps the adapter RGB-only, byte-for-
        # byte identical to the pre-depth behaviour.
        self.depth_enabled = bool(depth)
        self.stereo_profile = stereo_profile
        self.stereo_lr_check = bool(stereo_lr_check)
        self.depth_fps = max(0.5, float(depth_fps))
        # IMU publishing.  Additive like depth: RGB is never affected.  The
        # IMU stream feeds vibration compensation of the camera point cloud.
        self.imu_enabled = bool(imu)
        self.imu_rate = max(1.0, float(imu_rate))
        self.imu_fps = max(1.0, float(imu_fps))
        # H.265/H.264 use quality-based rate control (1-100, higher = better),
        # matching the depthai_sdk pattern.  MJPEG also uses setQuality().
        self.quality = max(1, min(100, int(quality)))
        self.keyframe_frequency = int(keyframe_frequency)
        # Auto-exposure tuning: positive EV compensation brightens dark
        # interiors that would otherwise be dragged down by a bright window.
        # ISP brightness/contrast add a static lift so the operator sees a
        # usable image even before 3A has converged.
        self.ev_compensation = int(ev_compensation)
        self.brightness = int(brightness)
        self.contrast = int(contrast)
        self.saturation = int(saturation)
        self.sharpness = int(sharpness)
        self.max_exposure_us = int(max_exposure_us)
        self.frame_id = {
            'docking_camera': 'docking_camera_optical_frame',
            'cutting_camera': 'cutter_camera_optical_frame',
        }[camera_role]
        # Canonical channel this adapter produces.
        self.channel = canonical_channel(camera_role)
        self.depth_channel = canonical_depth_channel(camera_role)
        self.camera_info_channel = canonical_camera_info_channel(camera_role)
        self.imu_channel = canonical_imu_channel(camera_role)

        context = zmq.Context.instance()
        self.push_socket = context.socket(zmq.PUSH)
        self.push_socket.setsockopt(zmq.LINGER, 0)
        self.push_socket.setsockopt(zmq.SNDHWM, 8)
        self.push_socket.connect(ingest_endpoint)

        self.chrony = ChronyStatus()
        self._dai = None
        # Depth-side state: the depth output queue (created only when
        # ``depth_enabled``) and a monotonic-ordered depth frame stream.
        self._depth_queue = None
        self._last_depth_emit = 0.0
        # Depth map dimensions (set in _build_depth_path to the mono size).
        self.depth_width = int(width)
        self.depth_height = int(height)
        # IMU-side state: the output queue (created only when imu_enabled) and
        # per-window accumulation for batched attitude derivation.
        self._imu_queue = None
        self._imu_available = False
        self._imu_to_cam = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        self._last_imu_emit = 0.0
        self._imu_accel_accum = [0.0, 0.0, 0.0]
        self._imu_gyro_accum = [0.0, 0.0, 0.0]
        self._imu_n_accum = 0

    def _open_pipeline(self):
        import depthai as dai
        self._dai = dai
        device_address = self.camera.address
        # Retry connecting: right after the previous process releases the
        # device (e.g. run_all.sh restarting the stack) the OAK can briefly be
        # in an undiscoverable state, so a single-shot connect races and fails.
        # After a hard XLink drop the device needs ~10-15s to finish its own
        # reconnection window before it is rediscoverable, so retry for a
        # generous 60s and log progress rather than bailing after 20s (which
        # made the supervisor churn: child exits, restarts after 3s, and the
        # new child hits a still-undiscoverable device and exits again).
        deadline = time.monotonic() + 60.0
        last_error = None
        next_log = 0.0
        while True:
            try:
                if device_address:
                    device = dai.Device(dai.DeviceInfo(device_address))
                else:
                    device = dai.Device()
                break
            except RuntimeError as error:
                last_error = error
                now = time.monotonic()
                if now >= deadline:
                    raise RuntimeError(
                        '[{}] could not connect to OAK {} after retries: {}. '
                        'Is the camera powered and discoverable (check '
                        '"python3 -c \\"import depthai as dai; '
                        'print(dai.Device.getAllAvailableDevices())\\"")?'.format(
                            self.channel, device_address or '<first>', error)) from error
                if now >= next_log:
                    print('[{}] OAK {} not yet discoverable ({}); retrying '
                          '...'.format(self.channel, device_address or '<first>',
                                       error))
                    next_log = now + 10.0
                time.sleep(1.0)
        pipeline = dai.Pipeline(device)

        pipeline.getDefaultDevice().setTimesync(timedelta(seconds=5), 10, True)
        cam = pipeline.create(dai.node.ColorCamera)
        cam.setBoardSocket(dai.CameraBoardSocket.CAM_A)
        cam.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
        cam.setVideoSize(self.width, self.height)
        cam.setFps(self.fps)
        # Brighten dark interiors: enable auto-exposure and add positive EV
        # compensation so a bright window doesn't drag the scene down.  Do
        # NOT cap the max exposure time — the sensor default lets
        # auto-exposure collect enough light in a dark room; a 33 ms or
        # 66 ms cap leaves the scene near-black.  The --max-exposure-us
        # flag (default 0 = sensor default) overrides this.
        control = cam.initialControl
        control.setAutoExposureEnable()
        if self.ev_compensation != 0:
            control.setAutoExposureCompensation(self.ev_compensation)
        if self.brightness != 0:
            control.setBrightness(self.brightness)
        if self.contrast != 0:
            control.setContrast(self.contrast)
        if self.max_exposure_us > 0:
            control.setAutoExposureLimit(self.max_exposure_us)

        encoder = pipeline.create(dai.node.VideoEncoder)
        profile_name = CODEC_PROFILES[self.codec]
        profile = getattr(dai.VideoEncoderProperties.Profile, profile_name)
        # Follow the depthai_sdk pattern exactly: preset (fps+profile) then
        # quality-based rate control.  Using setBitrateKbps() on H.265 pushes
        # the encoder into an invalid state that crashes the OAK firmware
        # ("stack smashing" in libdepthaicore).
        encoder.setDefaultProfilePreset(self.fps, profile)
        encoder.setQuality(self.quality)
        if self.codec != 'jpeg':
            try:
                encoder.setKeyframeFrequency(self.keyframe_frequency)
            except Exception as error:
                print('[{}] keyframe tuning skipped: {}'.format(self.channel, error))

        cam.video.link(encoder.input)
        # depthai 3.1.0 has no XLinkOut node: create the output queue directly
        # on the encoder bitstream output (same as the legacy MJPEG publisher).
        # The queue is drained with a blocking get() in run() so the tight
        # non-blocking tryGet() loop never races the XLink send.
        self.queue = encoder.bitstream.createOutputQueue(maxSize=8, blocking=True)

        # ---- additive stereo depth path (never touches the RGB path above) ----
        self._depth_queue = None
        self._depth_intrinsics = None
        if self.depth_enabled:
            try:
                self._depth_queue, self._depth_intrinsics = self._build_depth_path(
                    device, pipeline, dai)
            except Exception as error:
                # A unit without a stereo pair (e.g. OAK-D-Lite has no CAM_B /
                # CAM_C) would otherwise crash the whole adapter (RGB included)
                # into a --supervise restart loop.  Degrade to RGB-only instead.
                print('[{}] depth unavailable ({}); depth publishing disabled'.format(
                    self.channel, error))
                self.depth_enabled = False
                self._depth_queue = None
                self._depth_intrinsics = None

        # ---- additive IMU path (never touches the RGB/depth paths above) ----
        self._imu_queue = None
        self._imu_available = False
        if self.imu_enabled:
            self._imu_queue = self._build_imu_path(device, pipeline, dai)

        # camera_info JSON is derived from the device calibration once per
        # connect; it is emitted in run() before the first depth frame.
        return device, pipeline

    def _build_imu_path(self, device, pipeline, dai):
        """Build the additive v3 ``IMU`` node and return its output queue.

        Uses ``ACCELEROMETER_RAW``/``GYROSCOPE_RAW`` (the universally available
        raw streams on this depthai build) so the samples arrive in the IMU
        sensor frame; the raw vectors are then rotated into the camera optical
        frame on the host (see ``_emit_imu`` / ``gravity_to_rpy``) using the
        ``imuExtrinsics`` rotation, which is the same +X right / +Y down / +Z
        forward frame the point cloud is expressed in.

        Returns the output queue, or ``None`` if the device has no IMU (the
        adapter then publishes no IMU channel and the cloud renders
        uncompensated, matching the pre-IMU behaviour).  The node is only
        created after confirming an IMU is present and the report enums exist,
        so a failed/missing IMU can never leave a report-less node in the
        pipeline (which would make ``pipeline.start()`` abort the whole feed).
        """
        # Guard 1: does this device report an IMU at all?
        try:
            connected_imu = device.getConnectedIMU()
        except Exception as error:
            print('[{}] IMU unavailable ({}); IMU publishing disabled'.format(
                self.channel, error))
            self.imu_enabled = False
            return None
        if not connected_imu:
            print('[{}] no IMU on device; IMU publishing disabled'.format(
                self.channel))
            self.imu_enabled = False
            return None

        # Guard 2: confirm the report enums exist on this depthai build before
        # creating the node (avoids a report-less IMU node in the pipeline).
        try:
            accel_report = dai.IMUSensor.ACCELEROMETER_RAW
            gyro_report = dai.IMUSensor.GYROSCOPE_RAW
        except AttributeError as error:
            print('[{}] IMU report enum unavailable ({}); IMU publishing '
                  'disabled'.format(self.channel, error))
            self.imu_enabled = False
            return None

        imu = pipeline.create(dai.node.IMU)
        try:
            imu.enableIMUSensor([accel_report, gyro_report], int(self.imu_rate))
            imu.setBatchReportThreshold(1)
            imu.setMaxBatchReports(10)
        except Exception as error:
            # A later failure (e.g. unsupported rate) must not leave the
            # report-less node in the pipeline.
            print('[{}] IMU configuration failed ({}); IMU publishing '
                  'disabled'.format(self.channel, error))
            self.imu_enabled = False
            return None

        # Read the IMU -> camera extrinsics so the raw (sensor-native) accel
        # and gyro can be rotated into the camera optical frame on the host.
        # The rotation is the identity when the device does not expose it.
        self._imu_to_cam = _read_imu_to_cam_rotation(device, dai)

        self._imu_available = True
        return imu.out.createOutputQueue(maxSize=32, blocking=False)

    def _build_depth_path(self, device, pipeline, dai):
        """Build the additive StereoDepth pipeline and return its output queue.

        Returns ``(depth_queue, camera_info_dict)``.  Uses the v3 ``Camera``
        node for the mono pair (``ColorCamera``/``MonoCamera`` are deprecated
        in v3) and a v3 ``StereoDepth`` node.  The color sensor on ``CAM_A``
        is left untouched so the RGB H.264/H.265/JPEG path is unchanged.
        """
        # Mono resolution is the OAK-D Pro (RVC2) native mono size: OV9282 at
        # 1280x800 (800P).  Stereo runs at this size on-device.  The depth
        # output is then aligned to the RGB camera and resized to half the RGB
        # resolution (see setOutputSize below), which bounds per-frame memory
        # (a 1080p depth plane contributed to the dashboard OOM) while keeping
        # an exact 2x mapping between RGB and depth pixels.
        mono_width, mono_height = 1280, 800

        mono_left = pipeline.create(dai.node.Camera).build(
            dai.CameraBoardSocket.CAM_B, (mono_width, mono_height), self.fps)
        mono_right = pipeline.create(dai.node.Camera).build(
            dai.CameraBoardSocket.CAM_C, (mono_width, mono_height), self.fps)

        left_out = mono_left.requestOutput(
            (mono_width, mono_height), type=dai.ImgFrame.Type.RAW8, fps=self.fps)
        right_out = mono_right.requestOutput(
            (mono_width, mono_height), type=dai.ImgFrame.Type.RAW8, fps=self.fps)

        stereo = pipeline.create(dai.node.StereoDepth)
        preset = getattr(dai.node.StereoDepth.PresetMode, self.stereo_profile.upper())
        stereo.setDefaultProfilePreset(preset)
        stereo.setLeftRightCheck(self.stereo_lr_check)
        # Align depth to the RGB (color) camera so the depth map is centered on
        # the color view, then resize to EXACTLY half the RGB resolution (same
        # 16:9 aspect ratio).  This bounds memory (a 1080p depth plane was the
        # main OOM contributor) while keeping a clean, exact 2x mapping between
        # RGB pixel (u,v) and depth pixel (u,v): depth(u,v) == rgb(2u, 2v).
        # The intrinsics are therefore the RGB camera intrinsics at this half
        # resolution, read from CAM_A (not the mono CAM_B).
        stereo.setDepthAlign(dai.CameraBoardSocket.CAM_A)
        stereo.setOutputSize(self.width // 2, self.height // 2)
        stereo.setSubpixel(False)  # keep the depth map dense + fast

        left_out.link(stereo.left)
        right_out.link(stereo.right)

        depth_queue = stereo.depth.createOutputQueue(maxSize=2, blocking=False)

        # Depth is aligned to and resized against the RGB camera, so its
        # intrinsics come from CAM_A at the depth output resolution.
        depth_width = self.width // 2
        depth_height = self.height // 2
        self.depth_width = depth_width
        self.depth_height = depth_height

        # Build the camera_info JSON from the device calibration.  Intrinsics
        # are read from the device (never hard-coded), mapped into the frozen
        # camera_info schema consumed by the dashboard back-projection.
        camera_info = self._read_camera_info(device, depth_width, depth_height)
        return depth_queue, camera_info

    def _read_camera_info(self, device, width, height):
        """Read RGB-camera intrinsics at the delivered depth resolution.

        Depth is aligned to CAM_A (the RGB/color camera) and resized to
        ``width x height``, so back-projection must use the RGB camera's
        intrinsics at that size, not the mono camera's.  Falls back to an
        identity-only struct if the device does not expose calibration data
        (which makes back-projection unusable, so the dashboard then refuses
        annotations rather than producing wrong points).
        """
        try:
            calibration = device.getCalibration()
            intrinsics = calibration.getCameraIntrinsics(
                self._dai.CameraBoardSocket.CAM_A, width, height)
            fx, fy = intrinsics[0][0], intrinsics[1][1]
            cx, cy = intrinsics[0][2], intrinsics[1][2]
            # getDistortionCoefficients returns [k1,k2,p1,p2,k3,k4..k14].
            coeffs = calibration.getDistortionCoefficients(
                self._dai.CameraBoardSocket.CAM_A)
            d = list(coeffs[:5])
            d += [0.0] * max(0, 5 - len(d))
            # Rotation from RGB camera to the depth-aligned (rectified) frame.
            rectified_rotation = calibration.getStereoLeftRectificationRotation()
            if isinstance(rectified_rotation, (list, tuple)):
                rectified_rotation = [float(v) for row in rectified_rotation
                                      for v in row]
        except Exception as error:
            print('[{}] camera_info from device calibration failed ({}); '
                  'using identity intrinsics (back-projection disabled)'.format(
                      self.depth_channel, error))
            fx, fy = float(width), float(height)
            cx, cy = width / 2.0, height / 2.0
            d = [0.0, 0.0, 0.0, 0.0, 0.0]
            rectified_rotation = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        return {
            'width': int(width),
            'height': int(height),
            'distortion_model': 'plumb_bob',
            'd': [float(x) for x in d],
            'k': [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0],
            'r': [float(x) for x in rectified_rotation],
            'p': [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0],
            'binning_x': 1,
            'binning_y': 1,
            'roi': {'x_offset': 0, 'y_offset': 0, 'height': int(height),
                    'width': int(width), 'do_rectify': False},
        }

    def run(self):
        import depthai as dai
        device, pipeline = self._open_pipeline()
        self._dai = dai
        pipeline.start()
        depth_note = ' + depth' if self.depth_enabled else ''
        imu_note = ' + imu' if self._imu_available else ''
        print('[{}] OAK {} -> PUSH {} (codec {} {}x{} @ {:.1f}fps q{}{}{})'.format(
            self.channel, self.camera.address, self.push_socket.getsockopt(zmq.LAST_ENDPOINT),
            self.codec, self.width, self.height, self.fps, self.quality,
            depth_note, imu_note))
        # camera_info is emitted periodically (not just once) because ZMQ PUB
        # drops messages to subscribers that connect after the first publish;
        # a dashboard started later would otherwise never receive intrinsics.
        camera_info_period_s = 5.0
        last_camera_info = 0.0
        try:
            while pipeline.isRunning():
                now = time.monotonic()
                if self.depth_enabled and now - last_camera_info >= camera_info_period_s:
                    self._emit_camera_info()
                    last_camera_info = now
                try:
                    frame = self.queue.get()
                except Exception as error:
                    # The OAK device can drop the XLink connection (e.g. when
                    # the host CPU is briefly saturated and the monitor thread
                    # misses a keepalive ping).  ``queue.get()`` then raises
                    # ``MessageQueue.QueueException``/``RuntimeError``.  Treat
                    # this as a clean "device disconnected" exit so the
                    # supervisor restarts a fresh child instead of crashing
                    # with a traceback.
                    print('[{}] OAK device disconnected ({!r}); exiting for '
                          'supervisor restart'.format(self.channel, error))
                    break
                if frame is None:
                    continue
                self._emit(frame)
                # Drain the (non-blocking) depth queue best-effort each RGB
                # frame so depth keeps pace without ever blocking RGB.
                if self._depth_queue is not None:
                    self._drain_depth()
                # Drain the (non-blocking) IMU queue best-effort each RGB
                # frame; never blocks RGB.
                if self._imu_queue is not None:
                    self._drain_imu()
        finally:
            try:
                pipeline.stop()
            except Exception:
                pass
            self.push_socket.close(0)

    def _drain_imu(self):
        """Accumulate IMU batches and emit a throttled canonical packet.

        IMU samples arrive at ``imu_rate`` (batched up to 10 per ``IMUData``).
        We accumulate accel/gyro over the window since the last emit and
        publish one JSON packet at ``imu_fps`` with the mean samples plus the
        gravity-derived roll/pitch.  This keeps the wire tiny while still
        giving the dashboard a fresh attitude several times per second.
        """
        import depthai as dai
        try:
            imu_data = self._imu_queue.tryGet()
        except RuntimeError:
            return
        while imu_data is not None:
            for pkt in imu_data.packets:
                a = pkt.acceleroMeter
                g = pkt.gyroscope
                self._imu_accel_accum[0] += a.x
                self._imu_accel_accum[1] += a.y
                self._imu_accel_accum[2] += a.z
                self._imu_gyro_accum[0] += g.x
                self._imu_gyro_accum[1] += g.y
                self._imu_gyro_accum[2] += g.z
                self._imu_n_accum += 1
            try:
                imu_data = self._imu_queue.tryGet()
            except RuntimeError:
                break

        now = time.monotonic()
        if now - self._last_imu_emit < (1.0 / self.imu_fps):
            return
        if self._imu_n_accum == 0:
            return
        n = float(self._imu_n_accum)
        accel = [v / n for v in self._imu_accel_accum]
        gyro = [v / n for v in self._imu_gyro_accum]
        self._imu_accel_accum = [0.0, 0.0, 0.0]
        self._imu_gyro_accum = [0.0, 0.0, 0.0]
        self._imu_n_accum = 0
        self._last_imu_emit = now
        # Rotate RAW sensor-frame vectors into the camera optical frame so the
        # published accel/gyro and derived attitude share the point cloud's
        # frame convention.
        accel = _rotate_vec3(self._imu_to_cam, accel)
        gyro = _rotate_vec3(self._imu_to_cam, gyro)
        self._emit_imu(accel, gyro, int(n))

    def _emit_imu(self, accel, gyro, n_samples):
        import json
        roll, pitch, accel_norm = gravity_to_rpy(accel)
        acquisition_ns = time.time_ns()
        header = build_imu_header(
            self.camera_role, acquisition_ns, self.frame_id)
        payload = json.dumps({
            'frame_id': self.frame_id,
            'accel_ms2': [float(v) for v in accel],
            'gyro_rad_s': [float(v) for v in gyro],
            'attitude_rpy_rad': [roll, pitch, 0.0],
            'accel_norm_ms2': float(accel_norm),
            'sample_rate_hz': int(self.imu_rate),
            'n_samples': int(n_samples),
        }, separators=(',', ':')).encode('utf-8')
        try:
            frames = pack_message(self.imu_channel, header, payload)
        except Exception as error:
            print('[{}] rejected imu packet: {}'.format(self.imu_channel, error))
            return
        self.push_socket.send_multipart(frames)

    def _drain_depth(self):
        """Best-effort drain of the depth output queue into canonical packets.

        Depth is throttled to ``depth_fps`` (default 5 Hz, independent of the
        RGB rate) so the dashboard is not fed a 15 Hz x 2 camera stream of
        960x540 depth planes — that allocation churn contributed to the
        dashboard OOM.  Click-depth and the point cloud only need a few Hz.
        """
        import depthai as dai
        now = time.monotonic()
        if now - self._last_depth_emit < (1.0 / self.depth_fps):
            # Drop queued frames without draining the queue fully, keeping the
            # latest available for the next tick.
            while self._depth_queue.tryGet() is not None:
                pass
            return
        depth_frame = self._depth_queue.tryGet()
        if depth_frame is None:
            return
        self._last_depth_emit = now
        self._emit_depth(depth_frame)
        # Drain any further stale frames so we always emit the newest only.
        while self._depth_queue.tryGet() is not None:
            pass

    def _emit_camera_info(self):
        if self._depth_intrinsics is None:
            return
        import json
        acquisition_ns = time.time_ns()
        header = build_camera_info_header(
            self.camera_role, self.depth_width, self.depth_height,
            acquisition_ns, self.frame_id, publish_depth=True)
        payload = json.dumps(
            self._depth_intrinsics, separators=(',', ':')).encode('utf-8')
        try:
            frames = pack_message(self.camera_info_channel, header, payload)
        except Exception as error:
            print('[{}] rejected camera_info: {}'.format(
                self.camera_info_channel, error))
            return
        self.push_socket.send_multipart(frames)

    def _emit_depth(self, depth_frame):
        payload = bytes(depth_frame.getData())
        received_monotonic_ns = time.monotonic_ns()
        received_utc_ns = time.time_ns()
        acquisition_ns = capture_timestamp_us(
            depth_frame.getTimestamp(), received_monotonic_ns, received_utc_ns) * 1000
        header = build_depth_header(
            self.camera_role, self.depth_width, self.depth_height,
            acquisition_ns, self.frame_id, publish_camera_info=True)
        try:
            frames = pack_message(self.depth_channel, header, payload)
        except Exception as error:  # ProtocolError -> drop, do not crash
            print('[{}] rejected depth frame: {}'.format(self.depth_channel, error))
            return
        self.push_socket.send_multipart(frames)

    def _emit(self, frame):
        payload = bytes(frame.getData())
        received_monotonic_ns = time.monotonic_ns()
        received_utc_ns = time.time_ns()
        acquisition_ns = capture_timestamp_us(
            frame.getTimestamp(), received_monotonic_ns, received_utc_ns) * 1000
        keyframe = detect_keyframe(self.codec, payload)
        header = build_rgb_header(
            self.camera_role, self.codec, self.width, self.height,
            keyframe, acquisition_ns, self.frame_id)
        try:
            frames = pack_message(self.channel, header, payload)
        except Exception as error:  # ProtocolError -> drop, do not crash
            print('[{}] rejected frame: {}'.format(self.channel, error))
            return
        # PUSH the canonical three-frame packet into the aggregator. The
        # aggregator re-owns sequence/source_id/gateway_monotonic_ns.
        self.push_socket.send_multipart(frames)


def _arguments(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--camera-role', choices=camera_choices(),
                        default='docking_camera')
    parser.add_argument('--ingest-endpoint', default='tcp://127.0.0.1:5570',
                        help='aggregator PULL endpoint to PUSH canonical frames into')
    parser.add_argument('--codec', choices=('h264', 'h265', 'jpeg'), default='jpeg')
    parser.add_argument('--width', type=int, default=1920)
    parser.add_argument('--height', type=int, default=1080)
    parser.add_argument('--fps', type=float, default=15.0)
    parser.add_argument('--quality', type=int, default=90,
                        help='encoder quality (JPEG: 1-100 higher=better; H.264/H.265: same scale)')
    parser.add_argument('--keyframe-frequency', type=int, default=30)
    parser.add_argument('--ev-compensation', type=int, default=7,
                        help='auto-exposure EV compensation (-9..+9, positive = brighter)')
    parser.add_argument('--brightness', type=int, default=0,
                        help='ISP brightness offset (-10..10, positive = brighter; 0 = default)')
    parser.add_argument('--contrast', type=int, default=0,
                        help='ISP contrast offset (-10..10; 0 = default)')
    parser.add_argument('--saturation', type=int, default=0,
                        help='ISP saturation offset (-10..10; 0 = default)')
    parser.add_argument('--sharpness', type=int, default=0,
                        help='ISP sharpness offset (0..4; 0 = default)')
    parser.add_argument('--max-exposure-us', type=int, default=0,
                        help='max auto-exposure time in microseconds (0 = sensor default ~1s for IMX378; lower to cap for motion sharpness)')
    parser.add_argument('--no-depth', action='store_true',
                        help='disable the stereo depth stream (RGB-only, byte-for-byte pre-depth behaviour)')
    parser.add_argument('--stereo-profile',
                        choices=OakCapture.STEREO_PROFILES,
                        default='fast_density',
                        help='v3 StereoDepth profile preset (default fast_density)')
    parser.add_argument('--stereo-lr-check', dest='stereo_lr_check',
                        action='store_true', default=True,
                        help='enable left-right check on StereoDepth (default on)')
    parser.add_argument('--no-stereo-lr-check', dest='stereo_lr_check',
                        action='store_false',
                        help='disable left-right check on StereoDepth')
    parser.add_argument('--depth-fps', type=float, default=5.0,
                        help='depth stream emission rate in Hz (default 5; RGB stays at --fps)')
    parser.add_argument('--no-imu', action='store_true',
                        help='disable the IMU stream (RGB+/-depth only)')
    parser.add_argument('--imu-rate', type=float, default=50.0,
                        help='IMU accel/gyro report rate in Hz (default 50; '
                             'vibration compensation only needs a few Hz; the '
                             'future VIO path will use its own ~200 Hz bridge)')
    parser.add_argument('--imu-fps', type=float, default=5.0,
                        help='IMU packet emission rate in Hz (default 5, '
                             'matching depth)')
    parser.add_argument('--supervise', action='store_true',
                        help='run under a process supervisor that auto-restarts on crash')
    parser.add_argument('--restart-delay-s', type=float, default=3.0,
                        help='cooldown before restarting a crashed child')
    return parser.parse_args(argv)


def main(argv=None):
    args = _arguments(argv)
    capture = OakCapture(
        camera_role=args.camera_role,
        ingest_endpoint=args.ingest_endpoint,
        codec=args.codec,
        width=args.width,
        height=args.height,
        fps=args.fps,
        quality=args.quality,
        keyframe_frequency=args.keyframe_frequency,
        ev_compensation=args.ev_compensation,
        brightness=args.brightness,
        contrast=args.contrast,
        saturation=args.saturation,
        sharpness=args.sharpness,
        max_exposure_us=args.max_exposure_us,
        depth=not args.no_depth,
        stereo_profile=args.stereo_profile,
        stereo_lr_check=args.stereo_lr_check,
        depth_fps=args.depth_fps,
        imu=not args.no_imu,
        imu_rate=args.imu_rate,
        imu_fps=args.imu_fps,
    )
    try:
        capture.run()
    except KeyboardInterrupt:
        pass
    except RuntimeError as error:
        raise SystemExit(str(error))


# ---------------------------------------------------------------------------
# Process supervisor
# ---------------------------------------------------------------------------
#
# The OAK DepthAI firmware intermittently aborts with a native "stack smashing"
# (SIGABRT) in ``libdepthaicore`` — a C-level abort that Python ``except``
# blocks cannot catch.  A supervisor re-runs the capture in a child process and
# restarts it after any exit (including a native crash), so a single camera
# glitch never permanently kills the video feed.

def _run_capture_child(argv):
    """Execute ``main`` in-process (called from the supervisor's child)."""
    sys.exit(main(argv))


def supervise(argv=None):
    """Run the adapter in a child process, restarting it after any exit.

    The child runs ``main(argv)`` which blocks until the camera feed ends or
    crashes.  On child exit the supervisor waits ``restart_delay_s`` (letting
    the OAK device settle back into a discoverable state) and starts a fresh
    child.  SIGINT/SIGTERM stop the supervisor and its current child.
    """
    args = _arguments(argv)
    restart_delay_s = float(getattr(args, 'restart_delay_s', 3.0))

    stop = [False]

    def _handle_signal(signum, _frame):
        stop[0] = True

    signal.signal(signal.SIGINT, _handle_signal)
    signal.signal(signal.SIGTERM, _handle_signal)

    python = sys.executable
    module = [sys.argv[0], '-m', 'canonical_zmq_publisher.oak_capture']
    # Rebuild the exact CLI args, minus --supervise/--restart-delay-s so the
    # child does not recurse into the supervisor.
    child_argv = list(sys.argv[1:])
    child_argv = [a for a in child_argv
                  if not a.startswith('--supervise')
                  and not a.startswith('--restart-delay-s')]

    restart_count = 0
    while not stop[0]:
        print('[supervisor] starting child (restart #{})'.format(restart_count))
        proc = subprocess.Popen([python, '-m', 'canonical_zmq_publisher.oak_capture']
                                + child_argv)
        # Wait until the child exits or a stop is requested.
        while not stop[0]:
            try:
                code = proc.wait(timeout=0.5)
                break
            except subprocess.TimeoutExpired:
                continue
        if stop[0]:
            proc.terminate()
            try:
                proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                proc.kill()
            break
        print('[supervisor] child exited with code {}; restarting in {:.1f}s'.format(
            code, restart_delay_s))
        # Cooldown: give the OAK device time to leave its crashed/discoverable
        # limbo before the next connect attempt.
        slept = 0.0
        while slept < restart_delay_s and not stop[0]:
            time.sleep(0.1)
            slept += 0.1
        restart_count += 1

    print('[supervisor] stopped')


if __name__ == '__main__':
    if '--supervise' in sys.argv:
        supervise()
    else:
        main()
