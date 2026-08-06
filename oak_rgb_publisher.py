#!/usr/bin/env python3
import argparse
import contextlib
import time
from typing import Optional
import warnings

# Suppress the harmless DepthAI deprecation warning to keep your console clean
warnings.filterwarnings("ignore", category=DeprecationWarning)

import depthai as dai
import msgpack
import zmq


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
    parser = argparse.ArgumentParser()
    parser.add_argument('--topic', default='oak_22')
    parser.add_argument(
        '--device',
        help='DepthAI v3 device IP, MXID, or USB path; required to pin one publisher to a specific OAK',
    )
    parser.add_argument('--pub-port', dest='pub_port', type=int, default=5556)
    parser.add_argument('--ctl-port', dest='ctl_port', type=int, default=5566)
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
    print(f"[{args.topic}] Starting state: {'ENABLED' if is_enabled else 'DISABLED'}")

    # DepthAI v3 Pipeline is a context manager. When --device is supplied,
    # it is attached to that exact OAK rather than the first discovered one.
    with open_pipeline(args.device) as pipeline:
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

                payload = {
                    "topic": args.topic,
                    "timestamp": time.time(),
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
