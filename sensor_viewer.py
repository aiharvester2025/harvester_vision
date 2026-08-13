#!/usr/bin/env python3
"""Live viewer for docking-sensor telemetry from sensor_plc_publisher.py."""

import argparse
import json
import time
from typing import Optional


SENSOR_LAYOUT = (
    
    ("diagonal_left_45deg", "S_L1", "LEFT GATE"),
    ("diagonal_right_45deg", "S_R1", "RIGHT GATE"),
    ("center_line", "S_C", "CENTER LINE"),
    ("c_channel_left", "S_L2", "C-CHANNEL LEFT"),
    ("c_channel_right", "S_R2", "C-CHANNEL RIGHT"),
)


def parse_telemetry(message: bytes) -> dict:
    """Decode and minimally validate a publisher JSON payload."""
    payload = json.loads(message.decode("utf-8"))
    if payload.get("schema") != "harvester.sensor-telemetry.v2":
        raise ValueError("unsupported or missing telemetry schema")
    if not isinstance(payload.get("sensors"), dict):
        raise ValueError("telemetry payload does not contain sensors")
    return payload


def distance_text(reading: dict) -> str:
    if not reading or not reading.get("valid", False):
        return "NO TARGET"
    value = reading.get("distance_m")
    if not isinstance(value, (int, float)):
        return "INVALID"
    return f"{value:.3f} m"


def format_optional_m(value) -> str:
    return f"{value:.3f} m" if isinstance(value, (int, float)) else "--"


def draw_sensor_card(frame, cv2, short_name: str, label: str, rect, reading: dict):
    """Draw one consistent telemetry card for a distance sensor."""
    x, y, width, height = rect
    valid = bool(reading and reading.get("valid", False))
    accent = (57, 181, 74) if valid else (66, 108, 231)
    value_color = (235, 235, 235) if valid else (180, 180, 210)

    cv2.rectangle(frame, (x, y), (x + width, y + height), (44, 44, 44), -1)
    cv2.rectangle(frame, (x, y), (x + width, y + height), (86, 86, 86), 1)
    cv2.rectangle(frame, (x, y), (x + 6, y + height), accent, -1)
    cv2.putText(frame, short_name, (x + 22, y + 38), cv2.FONT_HERSHEY_SIMPLEX,
                0.73, (255, 255, 255), 2, cv2.LINE_AA)
    cv2.putText(frame, label, (x + 22, y + 67), cv2.FONT_HERSHEY_SIMPLEX,
                0.44, (175, 175, 175), 1, cv2.LINE_AA)
    cv2.putText(frame, distance_text(reading), (x + 22, y + 124),
                cv2.FONT_HERSHEY_SIMPLEX, 0.75, value_color, 2, cv2.LINE_AA)
    state = "VALID RANGE" if valid else "NO RETURN"
    cv2.putText(frame, state, (x + 22, y + 153), cv2.FONT_HERSHEY_SIMPLEX,
                0.42, accent, 1, cv2.LINE_AA)


def draw_panel(cv2, np, payload: Optional[dict], last_seen: float, timeout: float, endpoint: str):
    frame = np.full((720, 1280, 3), (24, 24, 24), dtype=np.uint8)
    live = payload is not None and time.monotonic() - last_seen <= timeout
    status = "LIVE" if live else "WAITING FOR TELEMETRY"
    status_color = (40, 200, 40) if live else (0, 170, 255)

    cv2.rectangle(frame, (0, 0), (1280, 110), (34, 34, 34), -1)
    cv2.putText(frame, "DOCKING SENSORS", (38, 49), cv2.FONT_HERSHEY_SIMPLEX,
                0.94, (245, 245, 245), 2, cv2.LINE_AA)
    cv2.putText(frame, "Live distance telemetry", (40, 79), cv2.FONT_HERSHEY_SIMPLEX,
                0.48, (165, 165, 165), 1, cv2.LINE_AA)
    cv2.rectangle(frame, (942, 28), (1242, 72), (48, 48, 48), -1)
    cv2.rectangle(frame, (942, 28), (1242, 72), status_color, 1)
    cv2.putText(frame, status, (960, 56), cv2.FONT_HERSHEY_SIMPLEX,
                0.56, status_color, 1, cv2.LINE_AA)
    cv2.putText(frame, endpoint, (942, 94), cv2.FONT_HERSHEY_SIMPLEX,
                0.42, (150, 150, 150), 1, cv2.LINE_AA)

    sensors = payload.get("sensors", {}) if payload else {}
    card_width, card_height, card_gap = 226, 176, 12
    for index, (key, short_name, label) in enumerate(SENSOR_LAYOUT):
        card_x = 38 + index * (card_width + card_gap)
        draw_sensor_card(frame, cv2, short_name, label,
                         (card_x, 145, card_width, card_height), sensors.get(key))

    phase = payload.get("simulation", {}).get("phase", "--") if payload else "--"
    seq = payload.get("sequence", "--") if payload else "--"
    derived = payload.get("derived", {}) if payload else {}
    info_cards = (
        ("DOCKING PHASE", phase),
        ("SEQUENCE", str(seq)),
        ("ENTRY ALIGNMENT", format_optional_m(derived.get("entry_alignment_error_m"))),
        ("LATERAL OFFSET", format_optional_m(derived.get("lateral_offset_estimate_m"))),
        ("EQUIV. DIAMETER", format_optional_m(derived.get("equivalent_diameter_estimate_m"))),
    )
    for index, (heading, value) in enumerate(info_cards):
        x = 38 + index * (card_width + card_gap)
        cv2.rectangle(frame, (x, 365), (x + card_width, 462), (38, 38, 38), -1)
        cv2.rectangle(frame, (x, 365), (x + card_width, 462), (75, 75, 75), 1)
        cv2.putText(frame, heading, (x + 16, 395), cv2.FONT_HERSHEY_SIMPLEX,
                    0.42, (160, 160, 160), 1, cv2.LINE_AA)
        cv2.putText(frame, value, (x + 16, 432), cv2.FONT_HERSHEY_SIMPLEX,
                    0.58, (240, 240, 240), 1, cv2.LINE_AA)

    cv2.putText(frame, "Press q or Esc to quit", (1012, 690), cv2.FONT_HERSHEY_SIMPLEX,
                0.43, (160, 160, 160), 1, cv2.LINE_AA)
    return frame


def main():
    parser = argparse.ArgumentParser(description="Display docking sensor telemetry from the Raspberry Pi.")
    parser.add_argument("--host", default="192.168.50.40", help="Raspberry Pi publisher address")
    parser.add_argument("--port", type=int, default=5555, help="Raspberry Pi publisher port")
    parser.add_argument("--topic", default="harvester.sensors.v1", help="ZeroMQ telemetry topic")
    parser.add_argument("--timeout", type=float, default=2.0, help="Seconds before data is shown as stale")
    parser.add_argument("--display-fps", type=float, default=15.0, help="Maximum GUI refresh rate")
    args = parser.parse_args()
    if args.timeout <= 0 or args.display_fps <= 0:
        parser.error("--timeout and --display-fps must be positive")

    import cv2
    import numpy as np
    import zmq

    endpoint = f"tcp://{args.host}:{args.port}"
    context = zmq.Context()
    subscriber = context.socket(zmq.SUB)
    subscriber.setsockopt(zmq.SUBSCRIBE, args.topic.encode("utf-8"))
    # Publisher messages are multipart (topic + JSON). ZMQ_CONFLATE is unsafe
    # for multipart streams because it can retain only part of a message.
    subscriber.setsockopt(zmq.RCVHWM, 10)
    subscriber.connect(endpoint)
    print(f"Subscribed to {args.topic!r} at {endpoint}")

    payload = None
    last_seen = 0.0
    try:
        while True:
            while True:
                try:
                    topic, message = subscriber.recv_multipart(flags=zmq.NOBLOCK)
                    if topic == args.topic.encode("utf-8"):
                        payload = parse_telemetry(message)
                        last_seen = time.monotonic()
                except zmq.Again:
                    break
                except (UnicodeDecodeError, json.JSONDecodeError, ValueError) as error:
                    print(f"Ignoring invalid telemetry: {error}")

            cv2.imshow("Docking Sensor Viewer", draw_panel(cv2, np, payload, last_seen, args.timeout, endpoint))
            key = cv2.waitKey(max(1, round(1000 / args.display_fps))) & 0xFF
            if key in (27, ord("q")):
                break
    finally:
        cv2.destroyAllWindows()
        subscriber.close()
        context.term()


if __name__ == "__main__":
    main()
