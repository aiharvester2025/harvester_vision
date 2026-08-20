"""Source-agnostic operator dashboard for canonical ZeroMQ telemetry v1.

The dashboard consumes the same canonical three-frame packets from the
Xavier simulation gateway, a replay publisher, or the future Orin
aggregator.  It never publishes on the canonical PUB endpoint, never
writes to the status REP endpoint, and performs no actuation of any kind.
"""

__version__ = '0.1.0'
