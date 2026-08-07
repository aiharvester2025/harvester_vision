#!/usr/bin/env python3
import argparse
import contextlib
import time
from datetime import timedelta
from typing import Optional
import warnings

# Suppress the harmless DepthAI deprecation warning to keep your console clean
warnings.filterwarnings("ignore", category=DeprecationWarning)

from camera_config import CAMERAS, camera_choices
from time_sync import (
    TIME_AUTHORITY,
    ChronyStatus,
    capture_timestamp_us,
    depthai_timestamp_to_monotonic_us,
)

dai = None
msgpack = None
zmq = None


@contextlib.contextmanager
def open_pipeline(device_address: Optional[str]):
    """Open a DepthAI v3 pipeline, optionally pinned to a PoE device IP."""
    if device_address:
        with dai.Device(dai.DeviceInfo(device_address)) as device:
            with dai.Pipeline(device) as pipeline:
                yield pipeline
    else:
        with dai.Pipeline() as pipeline:
            yield pipeline


def main():
    global dai, msgpack, zmq
    parser = argparse.ArgumentParser()
    parser.add_argument('--camera-role', choices=camera_choices(), default='docking_camera')
    parser.add_argument('--topic', help='Override topic for a custom deployment')
    parser.add_argument(
        '--device',
        help='Override DepthAI v3 device IP, MXID, or USB path',
    )
    parser.add_argument('--pub-port', dest='pub_port', type=int)
    parser.add_argument('--ctl-port', dest='ctl_port', type=int)
    parser.add_argument('--width', type=int, default=1280, help='Video width in pixels')
    parser.add_argument('--height', type=int, default=720, help='Video height in pixels')
    parser.add_argument('--fps', type=float, default=15.0)
    parser.add_argument('--quality', type=int, default=65, help='MJPEG quality (1-100)')
    parser.add_argument('--disabled', action='store_true')
    args = parser.parse_args()
    if args.width <= 0 or args.height <= 0:
        parser.error('--width and --height must be positive')
    if args.fps <= 0:
        parser.error('--fps must be positive')
    if not 1 <= args.quality <= 100:
        parser.error('--quality must be in the range 1-100')

    camera = CAMERAS[args.camera_role]
    args.topic = args.topic or camera.role
    args.device = args.device or camera.address
    args.pub_port = args.pub_port or camera.pub_port
    args.ctl_port = args.ctl_port or camera.ctl_port

    import depthai as depthai
    import msgpack as messagepack
    import zmq as zeromq
    dai, msgpack, zmq = depthai, messagepack, zeromq

    print(f"[{args.topic}] Publishing video on port {args.pub_port}")
    print(f"[{args.topic}] Listening for control on port {args.ctl_port}")
    print(f"[{args.topic}] Video: {args.width}x{args.height} @ {args.fps:g} FPS, MJPEG quality {args.quality}")
    if args.device:
        print(f"[{args.topic}] Connecting to OAK device {args.device}")

    # Set up ZMQ context and sockets
    context = zmq.Context()
    pub = context.socket(zmq.PUB)
    pub.bind(f"tcp://*:{args.pub_port}")

    # One viewer sends enable/disable commands with a PUSH socket. PULL is a
    # reliable one-way control channel and avoids PUB/SUB subscription races.
    ctl = context.socket(zmq.PULL)
    ctl.bind(f"tcp://*:{args.ctl_port}")

    is_enabled = not args.disabled
    chrony_status = ChronyStatus()
    sequence_number = 0
    print(f"[{args.topic}] Starting state: {'ENABLED' if is_enabled else 'DISABLED'}")

    # DepthAI v3 Pipeline is a context manager. When --device is supplied,
    # it is attached to that exact OAK rather than the first discovered one.
    with open_pipeline(args.device) as pipeline:
        # These are DepthAI v3's defaults. Set them explicitly to make the
        # synchronization policy part of this application's contract.
        pipeline.getDefaultDevice().setTimesync(timedelta(seconds=5), 10, True)
        cam = pipeline.create(dai.node.ColorCamera)
        cam.setBoardSocket(dai.CameraBoardSocket.CAM_A)
        cam.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
        cam.setVideoSize(args.width, args.height)
        cam.setFps(args.fps)

        encoder = pipeline.create(dai.node.VideoEncoder)
        encoder.setDefaultProfilePreset(args.fps, dai.VideoEncoderProperties.Profile.MJPEG)
        encoder.setQuality(args.quality)

        cam.video.link(encoder.input)

        # V3 API: Create output queue directly on the node's output (no XLinkOut)
        q_rgb = encoder.bitstream.createOutputQueue(maxSize=4, blocking=False)

        # V3 API: Start the pipeline directly
        pipeline.start()

        while pipeline.isRunning():
            # 1. Non-blocking check for control messages
            try:
                msg = ctl.recv_json(flags=zmq.NOBLOCK)
                if 'enabled' in msg:
                    is_enabled = msg['enabled']
                    print(f"[{args.topic}] State changed to: {'ENABLED' if is_enabled else 'DISABLED'}")
            except zmq.Again:
                pass  # No message waiting
            except Exception as e:
                print(f"Control channel error: {e}")

            # 2. Grab frames from the camera
            in_rgb = q_rgb.tryGet()

            # 3. Publish if enabled
            if in_rgb is not None and is_enabled:
                frame_data = in_rgb.getData()
                received_monotonic_ns = time.monotonic_ns()
                received_utc_ns = time.time_ns()
                capture_monotonic_us = depthai_timestamp_to_monotonic_us(in_rgb.getTimestamp())
                time_quality, chrony_offset_us = chrony_status.get()
                sequence_number += 1

                payload = {
                    "topic": args.topic,
                    "camera_role": args.camera_role,
                    "timestamp_us": capture_timestamp_us(
                        in_rgb.getTimestamp(), received_monotonic_ns, received_utc_ns
                    ),
                    "timestamp_monotonic_us": capture_monotonic_us,
                    "received_timestamp_us": received_utc_ns // 1_000,
                    "timestamp_source": "depthai_host_sync",
                    "time_authority": TIME_AUTHORITY,
                    "time_quality": time_quality,
                    "chrony_offset_us": chrony_offset_us,
                    "sequence_number": sequence_number,
                    "fps": args.fps,
                    "frame": frame_data.tobytes()
                }

                # Use msgpack for efficient binary serialization
                pub.send(msgpack.packb(payload, use_bin_type=True))
            elif in_rgb is None:
                # tryGet() is non-blocking. Avoid a full-speed busy-poll when
                # the next camera frame has not arrived yet.
                time.sleep(0.005)

if __name__ == '__main__':
    main()
