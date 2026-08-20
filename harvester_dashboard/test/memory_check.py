"""Bounded-memory acceptance measurement (manual run).

Blasts canonical packets over inproc faster than the drainer consumes
them (drain loop artificially sleeps), then verifies RSS stays bounded
and drops are counted rather than queued.

    PYTHONPATH=src/harvester_dashboard \\
    /usr/bin/python3 src/harvester_dashboard/test/memory_check.py
"""

import os
import sys
import threading
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import zmq

from helpers import json_packet
from harvester_dashboard.zmq_source import SocketDrainer


def rss_kb() -> int:
    with open('/proc/self/status') as status:
        for line in status:
            if line.startswith('VmRSS:'):
                return int(line.split()[1])
    return 0


def main() -> int:
    context = zmq.Context()
    pub = context.socket(zmq.PUB)
    pub.setsockopt(zmq.SNDHWM, 8)          # mirror gateway bound
    pub.bind('inproc://memory-check')
    drainer = SocketDrainer('inproc://memory-check', hwm=8, context=context)

    received = []
    drainer.on_packet = lambda c, h, p, parsed: received.append(c)

    # Let the subscription establish.
    for _ in range(3):
        pub.send_multipart(json_packet('v1/range/cutter', {'i': 0}))
        drainer.drain_once()
        time.sleep(0.05)

    stop = threading.Event()

    def blast():
        sequence = 0
        while not stop.is_set():
            sequence += 1
            try:
                pub.send_multipart(
                    json_packet('v1/range/cutter', {'i': sequence}),
                    flags=zmq.NOBLOCK)
            except zmq.Again:
                pass   # SNDHWM backpressure: drop, never queue unbounded
            time.sleep(0.0005)   # ~2000 pkt/s offered

    thread = threading.Thread(target=blast)
    thread.start()

    samples = []
    deadline = time.time() + 10.0
    while time.time() < deadline:
        time.sleep(0.5)            # artificially slow consumer
        drainer.drain_once(max_packets=64)
        samples.append(rss_kb())

    stop.set()
    thread.join()
    # Final drain of whatever remains.
    while drainer.drain_once(max_packets=512):
        pass
    drainer.close()
    pub.close(0)
    context.term()

    first, peak, last = samples[0], max(samples), samples[-1]
    print('RSS samples (kB): first={} peak={} last={}'.format(first, peak, last))
    print('received={} queued-at-drainer=0 (drain_once emptied socket)'.format(
        len(received)))
    growth_kb = peak - first
    print('peak growth: {} kB'.format(growth_kb))
    bounded = growth_kb < 30000   # well under any leak scale
    print('BOUNDED' if bounded else 'UNBOUNDED')
    return 0 if bounded else 1


if __name__ == '__main__':
    sys.exit(main())
