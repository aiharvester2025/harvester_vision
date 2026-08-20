"""Replay exact canonical telemetry recordings over a ZeroMQ PUB endpoint."""

from __future__ import annotations

import argparse
import time

import zmq

from .recording import iter_recordings


def _arguments():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('record_dir', help='directory created by PacketRecorder')
    parser.add_argument('--endpoint', default='tcp://*:5591',
                        help='PUB bind endpoint; default avoids the live gateway port')
    parser.add_argument('--speed', type=float, default=1.0,
                        help='timing multiplier; 2 is twice real-time')
    parser.add_argument('--max-gap-s', type=float, default=0.25,
                        help='maximum preserved gap between packets')
    parser.add_argument('--subscriber-wait-s', type=float, default=0.5,
                        help='wait after bind so ZeroMQ subscribers can connect')
    return parser.parse_args()


def main():
    args = _arguments()
    if args.speed <= 0.0 or args.max_gap_s < 0.0 or args.subscriber_wait_s < 0.0:
        raise SystemExit('speed must be positive; gaps and wait must be non-negative')
    records = list(iter_recordings(args.record_dir))
    if not records:
        raise SystemExit('no valid .msgpack recordings found in {}'.format(args.record_dir))

    context = zmq.Context.instance()
    socket = context.socket(zmq.PUB)
    socket.setsockopt(zmq.LINGER, 0)
    socket.bind(args.endpoint)
    print('Replaying {} packets on {}'.format(len(records), args.endpoint))
    time.sleep(args.subscriber_wait_s)
    try:
        previous_time_ns = None
        for recorded_time_ns, frames in records:
            if previous_time_ns is not None:
                elapsed_s = max(0.0, (recorded_time_ns - previous_time_ns) / 1e9)
                time.sleep(min(elapsed_s / args.speed, args.max_gap_s))
            socket.send_multipart(frames)
            previous_time_ns = recorded_time_ns
    finally:
        socket.close(0)


if __name__ == '__main__':
    main()
