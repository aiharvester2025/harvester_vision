#!/usr/bin/env python3
"""Set/read the MID-360 LiDAR IP address via Livox-SDK2.

The MID-360 does NOT persist its IP in firmware: it comes up in DHCP/auto-IP
mode on every power cycle. This tool sends ``SetLivoxLidarIp`` to move it onto
the deployment sensor LAN, and can be re-run (or invoked at Orin startup) to
re-assert ``192.168.50.30`` after the LiDAR reboots.

It must run on a host that can currently reach the LiDAR at its *present*
address. The very first run therefore targets the factory/default address
(typically ``192.168.1.111``) from the machine already streaming it.

    # From the laptop on 192.168.1.5, move the LiDAR to the sensor LAN:
    sudo python3 set_lidar_ip.py --host-ip 192.168.1.5 \
        --current-ip 192.168.1.111 --new-ip 192.168.50.30

    # From the Orin (192.168.50.10) re-assert after a power cycle:
    sudo python3 set_lidar_ip.py --host-ip 192.168.50.10
"""

from __future__ import annotations

import argparse
import ctypes
import json
import os
import tempfile
import time
from typing import Optional, Tuple

from lidar.livox_source import (
    DEFAULT_HOST_IP,
    DEFAULT_LIDAR_IP,
    DEFAULT_SDK_LIBRARY,
    _SdkBindings,
    default_sdk_config,
)

BROADCAST_IP = "255.255.255.255"

# LivoxLidarInfo (livox_lidar_def.h): dev_type, 16-byte serial, 16-byte IP.
# The SDK API is ``SetLivoxLidarInfoChangeCallback`` with
# ``LivoxLidarInfoChangeCallback(handle, const LivoxLidarInfo*, void*)``.
class _LidarInfo(ctypes.Structure):
    _pack_ = 1
    _fields_ = [
        ("dev_type", ctypes.c_uint8),
        ("sn", ctypes.c_char * 16),
        ("lidar_ip", ctypes.c_char * 16),
    ]


_INFO_CB = ctypes.CFUNCTYPE(
    None,
    ctypes.c_uint32,
    ctypes.POINTER(_LidarInfo),
    ctypes.c_void_p,
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--host-ip", default=DEFAULT_HOST_IP,
                        help=f"this host's IP on the LiDAR link (default {DEFAULT_HOST_IP})")
    parser.add_argument("--current-ip", default=None,
                        help="LiDAR's present IP; omit to broadcast-discover it")
    parser.add_argument("--target-serial", default=None,
                        help="only re-IP the device currently at this address "
                             "(required when discovery finds more than one)")
    parser.add_argument("--new-ip", default=DEFAULT_LIDAR_IP,
                        help=f"IP to assign (default {DEFAULT_LIDAR_IP})")
    parser.add_argument("--netmask", default="255.255.255.0")
    parser.add_argument("--gateway", default="0.0.0.0")
    parser.add_argument("--sdk-library", default=None,
                        help=f"SDK shared library (default {DEFAULT_SDK_LIBRARY})")
    parser.add_argument("--timeout", type=float, default=5.0,
                        help="seconds to wait for device discovery")
    parser.add_argument("--yes", action="store_true",
                        help="confirm the change (required)")
    args = parser.parse_args()
    if not args.yes:
        parser.error("refusing to change the LiDAR IP without --yes")
    return args


def _write_discovery_config(host_ip: str, broadcast: str) -> str:
    """Write an SDK config whose target is a unicast or broadcast address."""
    config = default_sdk_config(lidar_ip=broadcast, host_ip=host_ip)
    handle = tempfile.NamedTemporaryFile("w", suffix=".json", delete=False, encoding="utf-8")
    try:
        json.dump(config, handle)
        handle.write("\n")
    finally:
        handle.close()
    return handle.name


def discover(host_ip: str, current_ip: Optional[str], timeout: float,
             library: Optional[str]) -> Tuple[_SdkBindings, list, list, object]:
    """Init the SDK and collect the device list within ``timeout``.

    Returns ``(bindings, handles, addresses, callback)``. The callback is
    returned so the caller keeps a strong reference for the whole SDK lifetime:
    the SDK stores a raw function pointer, and a garbage-collected ctypes
    trampoline would be called after free.
    """
    # Probing a single address is faster and avoids a broadcast storm; discovery
    # still needs the SDK, so the config target is the address we expect.
    target = current_ip or BROADCAST_IP
    config_path = _write_discovery_config(host_ip, target)
    bindings = _SdkBindings(library)

    devices: list = []
    seen = {}

    def on_info_change(handle, info_ptr, client_data):
        info = info_ptr.contents
        lidar_ip = info.lidar_ip.split(b"\x00", 1)[0].decode("ascii", "replace")
        serial = info.sn.split(b"\x00", 1)[0].decode("ascii", "replace")
        seen[handle] = (lidar_ip, serial)
        if handle not in devices:
            devices.append(handle)

    info_cb = _INFO_CB(on_info_change)
    bindings.lib.SetLivoxLidarInfoChangeCallback.argtypes = [_INFO_CB, ctypes.c_void_p]
    bindings.lib.SetLivoxLidarInfoChangeCallback.restype = None
    bindings.lib.SetLivoxLidarInfoChangeCallback(info_cb, None)

    if not bindings.init(config_path, host_ip):
        os.unlink(config_path)
        raise RuntimeError(
            f"LivoxLidarSdkInit failed for host {host_ip}; is that IP assigned to "
            "the NIC on the LiDAR link?"
        )
    try:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline and not devices:
            time.sleep(0.1)
    finally:
        # The config file is only needed for discovery; do not leave it behind.
        os.unlink(config_path)
    addresses = [seen[h][0] for h in devices]
    serials = [seen[h][1] for h in devices]
    return bindings, devices, list(zip(addresses, serials)), info_cb


def main() -> None:
    args = parse_args()
    print(f"[set-lidar-ip] host {args.host_ip} -> LiDAR {args.new_ip}")
    if args.current_ip:
        print(f"[set-lidar-ip] targeting present address {args.current_ip}")
    else:
        print("[set-lidar-ip] no --current-ip; broadcasting for discovery")

    bindings, devices, discovered, _info_cb = discover(
        args.host_ip, args.current_ip, args.timeout, args.sdk_library
    )
    try:
        if not devices:
            raise SystemExit(
                "no LiDAR found; check cabling, power, and that this host is on "
                "the LiDAR's current subnet"
            )
        listing = ", ".join(
            "{} (sn {})".format(address, serial or "unknown")
            for address, serial in discovered
        )
        print(f"[set-lidar-ip] found {len(devices)} device(s): {listing}")
        # Re-IP exactly one device. Broadcasting can discover unrelated Livox
        # units on the same LAN, and writing the same address to all of them
        # would strand the extras; require an explicit choice when ambiguous.
        if len(devices) > 1 and not args.target_serial:
            raise SystemExit(
                "found {} devices ({}); refusing to re-IP them all. Re-run with "
                "--current-ip to target one, or --target-serial to select "
                "by serial.".format(len(devices), listing)
            )
        targeted = 0
        for handle, (address, serial) in zip(devices, discovered):
            # Select by SERIAL, not by address: comparing the serial argument
            # against an IP can never match and would silently re-IP nothing.
            if args.target_serial and serial != args.target_serial:
                continue
            targeted += 1
            status = bindings.set_ip(handle, args.new_ip, args.netmask, args.gateway)
            if status != 0:
                print(f"[set-lidar-ip] handle {handle} ({address}, sn {serial}) "
                      f"set-ip status {status:#x}")
            else:
                print(f"[set-lidar-ip] handle {handle} ({address}, sn {serial}) "
                      f"-> {args.new_ip} OK")
        if args.target_serial and targeted == 0:
            raise SystemExit(
                "no device matched --target-serial {!r}; discovered: {}".format(
                    args.target_serial, listing)
            )
        print("[set-lidar-ip] NOTE: the LiDAR reverts to DHCP/auto-IP on power cycle; "
              "re-run this or call it at Orin startup to make the address stick.")
    finally:
        bindings.uninit()


if __name__ == "__main__":
    main()
