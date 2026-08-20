"""Command-line entry point for the Orin canonical ZeroMQ publisher.

Run under the depthai-env python (which provides ``zmq``/``msgpack``) with the
contract on ``PYTHONPATH``::

    PYTHONPATH=canonical_zmq python3 -m canonical_zmq_publisher.main \
        --synthetic --relay tcp://10.108.137.233:5590
"""

from __future__ import annotations

import argparse
import time

from .aggregator import CanonicalAggregator
from .ingest import SyntheticSource


def _arguments(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--pub', default='tcp://*:5590', help='canonical PUB bind endpoint')
    parser.add_argument('--status', default='tcp://*:5600', help='read-only REP bind endpoint')
    parser.add_argument('--source-id', default='orin', help='source identity in headers')
    parser.add_argument('--source-mode', default='hardware',
                        choices=('hardware', 'simulation'))
    parser.add_argument('--queue-depth', type=int, default=2)
    parser.add_argument('--socket-hwm', type=int, default=8)
    parser.add_argument('--record-dir', default='', help='opt-in audit directory')
    parser.add_argument('--ingest', default='', help='PULL bind endpoint for adapters')
    parser.add_argument('--relay', default='', help='remote canonical PUB to forward')
    parser.add_argument('--synthetic', action='store_true',
                        help='emit synthetic hardware packets for validation')
    parser.add_argument('--synthetic-period-s', type=float, default=1.0)
    return parser.parse_args(argv)


def main(argv=None):
    args = _arguments(argv)
    aggregator = CanonicalAggregator(
        pub_endpoint=args.pub,
        status_endpoint=args.status,
        source_id=args.source_id,
        source_mode=args.source_mode,
        queue_depth=args.queue_depth,
        socket_hwm=args.socket_hwm,
        record_dir=args.record_dir,
        relay_endpoint=args.relay,
        ingest_endpoint=args.ingest,
    )
    aggregator.start()
    print('Orin canonical publisher: PUB {} ; status REP {} ; source {}'.format(
        args.pub, args.status, args.source_id))
    if args.relay:
        print('Relaying remote canonical source {}'.format(args.relay))
    if args.synthetic:
        source = SyntheticSource(aggregator, period_s=args.synthetic_period_s)
        print('Synthetic source enabled (period {:.2f}s)'.format(args.synthetic_period_s))

    last_status = 0.0
    last_synthetic = 0.0
    try:
        while True:
            now = time.monotonic()
            # Drain every non-empty channel so no stream is starved, then poll
            # the read-only status endpoint.
            while aggregator.flush_one_packet():
                pass
            aggregator.handle_status_request()
            if now - last_status >= 1.0:
                aggregator.publish_system_status()
                last_status = now
            if args.synthetic and now - last_synthetic >= args.synthetic_period_s:
                source.emit_once()
                last_synthetic = now
            time.sleep(0.002)
    except KeyboardInterrupt:
        pass
    finally:
        aggregator.close()


if __name__ == '__main__':
    main()
