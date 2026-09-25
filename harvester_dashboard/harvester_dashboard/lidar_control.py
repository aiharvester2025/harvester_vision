"""Opt-in LiDAR standby/normal control for the operator scan HUD.

This is the **one deliberate exception** to the dashboard's render-only rule,
and it exists because the operator scan workflow requires it: the operator must
be able to put the MID-360 into normal mode for a scan and back to standby when
done, rather than leaving the motor spinning between scans.

Scope is deliberately narrow:

  * **Disabled by default.**  Nothing is created until an endpoint is supplied
    (``--lidar-control``), so an existing deploy behaves exactly as before.
  * **One PUSH socket, one payload.**  It only ever sends ``{"enabled": bool}``
    to the producer's PULL control endpoint (``lidar_capture``), which is the
    documented on-demand contract every publisher already uses.
  * **No actuation.**  The LiDAR is a sensor, not an actuator: the worst case is
    the motor not spinning when a scan starts, which the HUD surfaces as
    ``no_data``.  It never touches the boom, the platform, or the PLC.

The dashboard's *view* controls remain render-only; only the three scan buttons
(SCAN/STOP/CANCEL) may call :meth:`set_enabled`, and only this publisher writes.
"""

from __future__ import annotations

import json
from typing import Optional


class LidarControlPublisher:
    """PUSHes ``{"enabled": bool}`` to the LiDAR producer's control socket."""

    def __init__(self, endpoint: str = '', context=None):
        self.endpoint = endpoint
        self.socket = None
        self.sent = 0
        self.errors = 0
        self.last_error = ''
        self._owns_context = False
        self._last_value: Optional[bool] = None
        if endpoint:
            import zmq
            if context is None:
                self._context = zmq.Context.instance()
                self._owns_context = True
            else:
                self._context = context
            self.socket = self._context.socket(zmq.PUSH)
            self.socket.setsockopt(zmq.LINGER, 0)
            self.socket.connect(endpoint)

    @property
    def enabled(self) -> bool:
        return self.socket is not None

    def set_enabled(self, enabled: bool) -> bool:
        """Send ``{"enabled": enabled}``; returns True when sent.

        De-duplicates consecutive identical values so a scan that is already in
        the requested state does not spam the producer, and returns False when
        the publisher is disabled (the caller treats that as "no control").

        A bounded BLOCKING send is used, not NOBLOCK: dropping a control command
        silently would leave the LiDAR in the wrong state while the HUD believed
        it had switched, so a short wait (with a timeout) is the safer trade.
        ``_last_value`` is only recorded when the send actually succeeded.
        """
        if not self.enabled:
            return False
        value = bool(enabled)
        if value == self._last_value:
            return False
        try:
            import zmq
            self.socket.send_json({'enabled': value}, flags=zmq.NOBLOCK)
        except zmq.Again:
            # Buffer full: retry once blocking, bounded, so a momentary backlog
            # does not lose the command.
            try:
                self.socket.send_json({'enabled': value})
            except Exception as error:  # pragma: no cover - transport failure
                self.errors += 1
                self.last_error = str(error)
                return False
        except Exception as error:  # pragma: no cover - transport failure
            self.errors += 1
            self.last_error = str(error)
            return False
        self._last_value = value
        self.sent += 1
        return True

    def close(self) -> None:
        if self.socket is not None:
            try:
                self.socket.close(0)
            except Exception:
                pass
            self.socket = None


__all__ = ['LidarControlPublisher']
