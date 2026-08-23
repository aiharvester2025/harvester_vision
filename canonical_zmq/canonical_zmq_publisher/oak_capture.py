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


def canonical_channel(camera_role):
    """Return the canonical ``v1/camera/<name>/rgb`` channel for a camera role."""
    name = ROLE_TO_CHANNEL.get(camera_role, camera_role)
    return 'v1/camera/{}/rgb'.format(name)


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
        'capabilities': {
            'camera.{}.rgb'.format(name): True,
            'camera.{}.depth'.format(name): False,
            'camera.{}.camera_info'.format(name): False,
            'target.world_fixed': False,
        },
        'codec': codec,
        'pixel_encoding': {'h264': 'H264', 'h265': 'H265', 'jpeg': 'RGB8'}[codec],
        'width': int(width),
        'height': int(height),
        'keyframe': bool(keyframe),
    }


class OakCapture:
    """DepthAI pipeline that PUSHes canonical RGB frames to an aggregator."""

    def __init__(self, camera_role, ingest_endpoint='tcp://127.0.0.1:5570',
                 codec='jpeg', width=1920, height=1080, fps=15.0,
                 quality=90, keyframe_frequency=30, ev_compensation=7,
                 brightness=0, contrast=0, saturation=0, sharpness=0,
                 max_exposure_us=0):
        if camera_role not in CAMERAS:
            raise ValueError('unknown camera role {!r}'.format(camera_role))
        if codec not in CODEC_PROFILES:
            raise ValueError('unsupported codec {!r}'.format(codec))
        self.camera_role = camera_role
        self.camera = CAMERAS[camera_role]
        self.codec = codec
        self.width = int(width)
        self.height = int(height)
        self.fps = float(fps)
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

        context = zmq.Context.instance()
        self.push_socket = context.socket(zmq.PUSH)
        self.push_socket.setsockopt(zmq.LINGER, 0)
        self.push_socket.setsockopt(zmq.SNDHWM, 8)
        self.push_socket.connect(ingest_endpoint)

        self.chrony = ChronyStatus()
        self._dai = None

    def _open_pipeline(self):
        import depthai as dai
        self._dai = dai
        device_address = self.camera.address
        # Retry connecting: right after the previous process releases the
        # device (e.g. run_all.sh restarting the stack) the OAK can briefly be
        # in an undiscoverable state, so a single-shot connect races and fails.
        deadline = time.monotonic() + 20.0
        last_error = None
        while True:
            try:
                if device_address:
                    device = dai.Device(dai.DeviceInfo(device_address))
                else:
                    device = dai.Device()
                break
            except RuntimeError as error:
                last_error = error
                if time.monotonic() >= deadline:
                    raise RuntimeError(
                        '[{}] could not connect to OAK {} after retries: {}. '
                        'Is the camera powered and discoverable (check '
                        '"python3 -c \\"import depthai as dai; '
                        'print(dai.Device.getAllAvailableDevices())\\"")?'.format(
                            self.channel, device_address or '<first>', error)) from error
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
        return device, pipeline

    def run(self):
        import depthai as dai
        device, pipeline = self._open_pipeline()
        self._dai = dai
        pipeline.start()
        print('[{}] OAK {} -> PUSH {} (codec {} {}x{} @ {:.1f}fps q{})'.format(
            self.channel, self.camera.address, self.push_socket.getsockopt(zmq.LAST_ENDPOINT),
            self.codec, self.width, self.height, self.fps, self.quality))
        try:
            while pipeline.isRunning():
                frame = self.queue.get()
                if frame is None:
                    continue
                self._emit(frame)
        finally:
            self.push_socket.close(0)

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
