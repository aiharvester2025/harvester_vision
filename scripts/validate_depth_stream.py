#!/usr/bin/env python3
"""Verify the OAK-D stereo depth stream dimensions and intrinsics on the wire.

Subscribes to the canonical PUB endpoint and reports, for each camera, the
delivered ``width``/``height`` of the ``/rgb``, ``/depth``, and ``/camera_info``
channels, confirming that depth is exactly half the RGB resolution (the clean
2x mapping the dashboard relies on for click accuracy), that depth frames are
actually flowing with valid millimetre values, and that camera_info intrinsics
are non-identity.

Run with the canonical stack up (``./run_all.sh foreground`` in another
terminal) and the ``depthai-env`` python::

    PYTHONPATH=canonical_zmq:. /home/marcop/depthai-env/bin/python3 \\
        scripts/validate_depth_stream.py --duration 10

Exit code 0 means all checks passed.
"""

import argparse
import sys
import time
from pathlib import Path

import numpy as np

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--pub", default="tcp://127.0.0.1:5590")
    parser.add_argument("--duration", type=float, default=10.0)
    args = parser.parse_args()
    if args.duration <= 0:
        parser.error("--duration must be positive")

    import zmq
    from harvester_telemetry_contract import unpack_message

    context = zmq.Context()
    socket = context.socket(zmq.SUB)
    socket.setsockopt(zmq.LINGER, 0)
    socket.setsockopt(zmq.SUBSCRIBE, b"v1/camera/")
    socket.connect(args.pub)

    # Accumulate per-channel facts keyed by (camera, kind).
    facts = {}          # (camera, kind) -> {'width','height', 'n', 'non_zero'}
    camera_info = {}    # camera -> intrinsics dict
    errors = []
    deadline = time.monotonic() + args.duration

    print("Listening on {} for {:.0f}s ...".format(args.pub, args.duration))
    while time.monotonic() < deadline:
        try:
            frames = socket.recv_multipart(flags=zmq.NOBLOCK)
        except zmq.Again:
            time.sleep(0.05)
            continue
        try:
            channel, header, payload = unpack_message(frames)
        except Exception as error:
            errors.append("unpack: {}".format(error))
            continue

        if '/cutter/' in channel:
            camera = 'cutter'
        elif '/docking/' in channel:
            camera = 'docking'
        else:
            continue

        if channel.endswith('/rgb'):
            kind = 'rgb'
        elif channel.endswith('/depth'):
            kind = 'depth'
        elif channel.endswith('/camera_info'):
            kind = 'camera_info'
        else:
            continue

        entry = facts.setdefault((camera, kind), {
            'width': header.get('width'), 'height': header.get('height'),
            'n': 0, 'non_zero': 0, 'codec': header.get('codec'),
        })
        entry['n'] += 1
        entry['width'] = header.get('width')
        entry['height'] = header.get('height')

        if kind == 'depth':
            millimetres = np.frombuffer(payload, dtype='<u2')
            entry['non_zero'] += int((millimetres != 0).sum())
            entry['payload_bytes'] = len(payload)
        elif kind == 'camera_info':
            try:
                import json
                camera_info[camera] = json.loads(payload.decode('utf-8'))
            except Exception:
                pass

    socket.close(0)
    context.term()

    ok = True

    for camera in ('cutter', 'docking'):
        rgb = facts.get((camera, 'rgb'))
        depth = facts.get((camera, 'depth'))
        info = facts.get((camera, 'camera_info'))
        ci = camera_info.get(camera)

        print("\n=== {} ===".format(camera))
        if rgb is None:
            print("  rgb: MISSING")
            ok = False
        else:
            print("  rgb:         {}x{}  ({} frames, codec {})".format(
                rgb['width'], rgb['height'], rgb['n'], rgb['codec']))
        if depth is None:
            print("  depth:       MISSING (no depth frames received)")
            ok = False
        else:
            print("  depth:       {}x{}  ({} frames, {} valid mm px, {} bytes/frame, codec {})".format(
                depth['width'], depth['height'], depth['n'],
                depth['non_zero'], depth.get('payload_bytes'), depth['codec']))
            if rgb:
                expected_w = rgb['width'] // 2
                expected_h = rgb['height'] // 2
                # Two valid pixel-aligned cases: depth is exactly half the RGB
                # resolution (hardware oak_capture) or the same as RGB
                # (synthetic / 1:1).  Both give a clean, integer scale factor.
                half_res = (depth['width'], depth['height']) == (expected_w, expected_h)
                same_res = (depth['width'], depth['height']) == (rgb['width'], rgb['height'])
                if half_res:
                    print("  ok: depth is exactly half RGB resolution (clean 2x mapping)")
                elif same_res:
                    print("  ok: depth is pixel-aligned 1:1 with RGB")
                else:
                    print("  !! depth {}x{} is neither half ({}) nor equal ({}) to RGB — click mapping may be off".format(
                        depth['width'], depth['height'],
                        (expected_w, expected_h), (rgb['width'], rgb['height'])))
                    ok = False
            if depth['non_zero'] == 0:
                print("  !! all depth pixels are zero (invalid) — check stereo scene / dot projector")
                ok = False
            if depth['n'] < 2:
                print("  !! too few depth frames; stream may be stalled")
                ok = False

        if info is None:
            print("  camera_info: MISSING")
            ok = False
        elif ci is None:
            print("  camera_info: present but JSON unparseable")
            ok = False
        else:
            k = ci.get('k')
            identity = (k and k[0] == info['width'] and k[4] == info['height']
                        and k[2] == info['width'] / 2 and k[5] == info['height'] / 2)
            print("  camera_info: {}x{}  fx={:.1f} fy={:.1f} cx={:.1f} cy={:.1f}{}".format(
                ci.get('width'), ci.get('height'),
                k[0] if k else float('nan'), k[4] if k else float('nan'),
                k[2] if k else float('nan'), k[5] if k else float('nan'),
                "  (IDENTITY — intrinsics fallback, back-projection disabled!)" if identity else ""))
            if identity:
                ok = False

    if errors:
        print("\n{} unpack errors (first few):".format(len(errors)))
        for e in errors[:5]:
            print("  -", e)

    print("\n" + ("PASS: depth stream dimensions and intrinsics are correct."
                  if ok else "FAIL: see '!!' lines above."))
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
