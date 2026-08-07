#!/usr/bin/env python3
"""Persist an OAK PoE IP address. Connect one bootloader device at a time."""

import argparse

from camera_config import CAMERAS, camera_choices

dai = None


def check_str(value: str):
    pieces = value.split(".")
    if len(pieces) != 4:
        raise ValueError(f"{value!r} is not an IPv4 address")
    if any(not piece.isdigit() or not 0 <= int(piece) <= 255 for piece in pieces):
        raise ValueError(f"{value!r} is not an IPv4 address")
    return value


def main():
    global dai
    parser = argparse.ArgumentParser(description="Persist OAK PoE network configuration")
    parser.add_argument("--camera-role", choices=camera_choices())
    parser.add_argument("--static-ip", help="Static OAK IP for a custom deployment")
    parser.add_argument("--mask", default="255.255.255.0")
    parser.add_argument("--gateway", default="0.0.0.0")
    parser.add_argument("--yes", action="store_true", help="Confirm the physical-device flash")
    args = parser.parse_args()
    if args.camera_role and args.static_ip:
        parser.error("use either --camera-role or --static-ip")
    target_ip = args.static_ip or (CAMERAS[args.camera_role].address if args.camera_role else None)

    import depthai as depthai
    dai = depthai

    found, info = dai.DeviceBootloader.getFirstAvailableDevice()
    if not found:
        raise SystemExit("No OAK bootloader device found")
    print(f"Found device with name: {info.name}")

    if target_ip:
        if not args.yes:
            raise SystemExit(f"Refusing to flash {target_ip} without --yes")
        key, ipv4, mask, gateway = "1", target_ip, args.mask, args.gateway
    else:
        print('"1" to set a static IPv4 address')
        print('"2" to set a dynamic IPv4 address')
        print('"3" to clear the config')
        key = input("Enter the number: ").strip()
        if key not in ("1", "2", "3"):
            raise ValueError("Enter 1, 2, or 3")
        if key in ("1", "2"):
            ipv4 = input("Enter IPv4: ").strip()
            mask = input("Enter IPv4 Mask: ").strip()
            gateway = input("Enter IPv4 Gateway: ").strip()

    with dai.DeviceBootloader(info) as bootloader:
        if key in ("1", "2"):
            ipv4, mask, gateway = check_str(ipv4), check_str(mask), check_str(gateway)
            mode = "static" if key == "1" else "dynamic"
            if not target_ip:
                confirmation = input(
                    f"Flashing {mode} IPv4 {ipv4}, mask {mask}, gateway {gateway}. Enter y to confirm: "
                ).strip()
                if confirmation != "y":
                    raise SystemExit("Flashing aborted")
            config = dai.DeviceBootloader.Config()
            if key == "1":
                config.setStaticIPv4(ipv4, mask, gateway)
            else:
                config.setDynamicIPv4(ipv4, mask, gateway)
            success, error = bootloader.flashConfig(config)
        else:
            success, error = bootloader.flashConfigClear()
    if not success:
        raise SystemExit(f"Flashing failed: {error}")
    print("Flashing successful")


if __name__ == "__main__":
    main()
