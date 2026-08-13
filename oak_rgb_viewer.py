#!/usr/bin/env python3
import argparse
import json
import os
import time

from sensor_viewer import SENSOR_LAYOUT, distance_text, parse_telemetry

# Imported after argument parsing.  This lets ``--help`` work even on hosts
# whose OpenCV GUI backend is not available.
cv2 = None
msgpack = None
np = None
zmq = None

def create_offline_frame(
    topic: str, width: int = 1280, height: int = 720, reason: str = "CAMERA OFFLINE"
):
    """Generates a visual fallback frame when a camera drops, times out, or starts offline."""
    frame = np.zeros((height, width, 3), dtype=np.uint8)
    frame[:] = (35, 35, 35)  # Dark gray canvas

    # Red warning header
    cv2.putText(
        frame,
        f"[{topic.upper()}] - {reason}",
        (50, height // 2 - 30),
        cv2.FONT_HERSHEY_SIMPLEX,
        1.0,
        (0, 0, 255),
        2,
        cv2.LINE_AA,
    )
    # Informational subtitle
    cv2.putText(
        frame,
        "No frames received. Check device connection or publisher status.",
        (50, height // 2 + 20),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.6,
        (180, 180, 180),
        1,
        cv2.LINE_AA,
    )
    cv2.putText(
        frame,
        f"Last check: {time.strftime('%H:%M:%S')}",
        (50, height // 2 + 60),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.5,
        (120, 120, 120),
        1,
        cv2.LINE_AA,
    )
    return frame

def send_control_cmd(control_socket, topic: str, enabled: bool):
    """Sends a control command only after the remote PULL endpoint is connected."""
    cmd = {"topic": topic, "enabled": enabled}
    payload = json.dumps(cmd).encode("utf-8")
    try:
        control_socket.send(payload)
        print(f"[CONTROL] Sent to '{topic}': enabled={enabled}")
    except zmq.Again:
        print(
            f"[CONTROL] '{topic}' control endpoint is not connected; "
            "restart the matching publisher using the updated script"
        )
    except Exception as err:
        print(f"[CONTROL] Error sending command to '{topic}': {err}")


def draw_sensor_overlay(frame, payload, last_seen, timeout):
    """Draw a compact five-sensor dashboard over the bottom-right of a frame."""
    height, width = frame.shape[:2]
    panel_width, panel_height = min(440, width - 30), min(278, height - 30)
    x0, y0 = width - panel_width - 18, height - panel_height - 18
    live = payload is not None and time.monotonic() - last_seen <= timeout
    status = "LIVE" if live else "STALE"
    status_color = (55, 190, 75) if live else (0, 170, 255)
    sensors = payload.get("sensors", {}) if payload else {}

    overlay = frame.copy()
    cv2.rectangle(overlay, (x0, y0), (x0 + panel_width, y0 + panel_height), (22, 22, 22), -1)
    cv2.rectangle(overlay, (x0, y0), (x0 + panel_width, y0 + panel_height), (190, 190, 190), 1)
    cv2.rectangle(overlay, (x0, y0), (x0 + panel_width, y0 + 44), (38, 38, 38), -1)
    cv2.putText(overlay, "DOCKING SENSORS", (x0 + 14, y0 + 29), cv2.FONT_HERSHEY_SIMPLEX,
                0.58, (245, 245, 245), 1, cv2.LINE_AA)
    cv2.rectangle(overlay, (x0 + panel_width - 84, y0 + 10), (x0 + panel_width - 12, y0 + 34), status_color, -1)
    cv2.putText(overlay, status, (x0 + panel_width - 76, y0 + 28), cv2.FONT_HERSHEY_SIMPLEX,
                0.42, (25, 25, 25), 1, cv2.LINE_AA)

    for index, (key, short_name, label) in enumerate(SENSOR_LAYOUT):
        row_y = y0 + 56 + index * 33
        reading = sensors.get(key)
        valid = bool(reading and reading.get("valid", False))
        accent = (55, 190, 75) if valid else (66, 108, 231)
        cv2.rectangle(overlay, (x0 + 12, row_y), (x0 + 18, row_y + 23), accent, -1)
        cv2.putText(overlay, f"{short_name}  {label}", (x0 + 30, row_y + 17),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.42, (215, 215, 215), 1, cv2.LINE_AA)
        value_size = cv2.getTextSize(distance_text(reading), cv2.FONT_HERSHEY_SIMPLEX, 0.46, 1)[0]
        cv2.putText(overlay, distance_text(reading),
                    (x0 + panel_width - 14 - value_size[0], row_y + 17),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.46, (245, 245, 245), 1, cv2.LINE_AA)

    phase = payload.get("simulation", {}).get("phase", "--") if payload else "--"
    cv2.line(overlay, (x0 + 12, y0 + panel_height - 50),
             (x0 + panel_width - 12, y0 + panel_height - 50), (85, 85, 85), 1)
    cv2.putText(overlay, f"PHASE: {phase}", (x0 + 14, y0 + panel_height - 28),
                cv2.FONT_HERSHEY_SIMPLEX, 0.43, (210, 210, 210), 1, cv2.LINE_AA)
    cv2.putText(overlay, "[3] Hide sensors", (x0 + 14, y0 + panel_height - 9),
                cv2.FONT_HERSHEY_SIMPLEX, 0.35, (150, 150, 150), 1, cv2.LINE_AA)
    cv2.addWeighted(overlay, 0.92, frame, 0.08, 0, frame)

if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Multi-Camera ZeroMQ Viewer with Topic Pre-registration and Timeout Detection."
    )
    # Usage: --camera TOPIC SUB_ADDR CTL_ADDR
    parser.add_argument(
        "--camera",
        nargs=3,
        action="append",
        metavar=("TOPIC", "SUB_ADDR", "CTL_ADDR"),
        help="Register camera as: TOPIC SUB_ADDR CTL_ADDR (can be specified multiple times)",
    )
    # Compatibility with the previous one-host/many-port invocation.  New
    # deployments should prefer --camera because it also states the control
    # endpoint explicitly.
    parser.add_argument("--host", help="Legacy stream host (use with --ports and --topics)")
    parser.add_argument("--ports", nargs="+", type=int, help="Legacy publisher ports")
    parser.add_argument("--topics", nargs="+", help="Legacy camera topics")
    parser.add_argument(
        "--active-topic",
        help="Topic to show at startup; all other configured streams are disabled",
    )
    parser.add_argument(
        "--timeout",
        type=float,
        default=2.0,
        help="Seconds of missing frames before triggering timeout offline state",
    )
    parser.add_argument(
        "--display-fps",
        type=float,
        default=30.0,
        help="Maximum GUI redraw rate; does not change camera capture FPS",
    )
    parser.add_argument("--sensor-host", default="192.168.50.40", help="Raspberry Pi sensor publisher address")
    parser.add_argument("--sensor-port", type=int, default=5555, help="Raspberry Pi sensor publisher port")
    parser.add_argument("--sensor-topic", default="harvester.sensors.v1", help="Sensor telemetry topic")
    parser.add_argument("--sensor-timeout", type=float, default=2.0, help="Seconds before sensor data is stale")
    args = parser.parse_args()
    if args.display_fps <= 0 or args.sensor_timeout <= 0:
        parser.error("--display-fps and --sensor-timeout must be positive")

    if args.camera and any(value is not None for value in (args.host, args.ports, args.topics)):
        parser.error("use either --camera or --host --ports --topics, not both")

    if args.host is not None or args.ports is not None or args.topics is not None:
        if not (args.host and args.ports and args.topics):
            parser.error("--host, --ports, and --topics must be supplied together")
        if len(args.ports) != len(args.topics):
            parser.error("--ports and --topics must have the same number of values")
        cameras_config = [
            (topic, f"tcp://{args.host}:{port}", f"tcp://{args.host}:{port + 10}")
            for topic, port in zip(args.topics, args.ports)
        ]
    else:
        cameras_config = args.camera or [
            ("docking_camera", "tcp://localhost:5556", "tcp://localhost:5566"),
            ("cutting_camera", "tcp://localhost:5557", "tcp://localhost:5567"),
        ]

    configured_topics = [topic for topic, _, _ in cameras_config]
    active_topic = args.active_topic or configured_topics[0]
    if active_topic not in configured_topics:
        parser.error(f"--active-topic must be one of: {', '.join(configured_topics)}")

    # OpenCV's bundled Qt plugin looks for fonts inside its wheel, although
    # this host provides them through fontconfig.  Point Qt at a real system
    # font directory before OpenCV loads the GUI backend.
    font_dir = "/usr/share/fonts/truetype/dejavu"
    if os.path.isdir(font_dir):
        os.environ.setdefault("QT_QPA_FONTDIR", font_dir)

    import cv2
    import msgpack
    import numpy as np
    import zmq

    context = zmq.Context()

    # Subscriber Socket (Receives video frames)
    sub_socket = context.socket(zmq.SUB)
    sub_socket.setsockopt(zmq.SUBSCRIBE, b"")
    sub_socket.setsockopt(zmq.CONFLATE, 1)

    sensor_endpoint = f"tcp://{args.sensor_host}:{args.sensor_port}"
    sensor_socket = context.socket(zmq.SUB)
    sensor_socket.setsockopt(zmq.SUBSCRIBE, args.sensor_topic.encode("utf-8"))
    # Sensor publisher messages are multipart; do not use ZMQ_CONFLATE here.
    sensor_socket.setsockopt(zmq.RCVHWM, 10)
    sensor_socket.connect(sensor_endpoint)

    camera_state = {}
    control_sockets = {}
    viewer_state = {"active_topic": active_topic, "sensor_overlay_enabled": False}
    sensor_state = {"payload": None, "last_seen": 0.0}

    # Pre-register all expected topics at startup
    for topic, sub_addr, ctl_addr in cameras_config:
        camera_state[topic] = {
            "frame": None,
            "last_seen": 0.0,
            "enabled": topic == active_topic,
            "metadata": {},
        }

        sub_socket.connect(sub_addr)
        print(f"[{topic}] Subscribed to stream at {sub_addr}")

        # Publisher control uses PULL. IMMEDIATE makes a missing/incompatible
        # publisher visible instead of silently queueing a command forever.
        ctl_sock = context.socket(zmq.PUSH)
        ctl_sock.setsockopt(zmq.IMMEDIATE, 1)
        ctl_sock.setsockopt(zmq.SNDTIMEO, 1000)
        ctl_sock.connect(ctl_addr)
        control_sockets[topic] = ctl_sock
        print(f"[{topic}] Connected control channel to {ctl_addr}")

    print("\n--- Viewer Controls ---")
    print(" Press '1'..'9' : Select a camera (stops all other configured streams)")
    print(" Press '3'       : Show/hide docking-sensor overlay")
    print(" Press 'q' or ESC : Quit viewer\n")
    print(f"[sensors] Subscribed to {args.sensor_topic!r} at {sensor_endpoint}")

    def select_camera(target_topic: str, force: bool = False):
        """Enable exactly one camera and pause all other publishers."""
        viewer_state["active_topic"] = target_topic
        for topic, state in camera_state.items():
            enabled = topic == target_topic
            changed = state["enabled"] != enabled
            state["enabled"] = enabled
            if force or changed:
                send_control_cmd(control_sockets[topic], topic, enabled)
        print(f"[SELECT] Active camera: {target_topic}; other streams paused")

    # Force a full state sync so non-selected cameras stop publishing.
    time.sleep(0.25)
    select_camera(active_topic, force=True)

    try:
        while True:
            now = time.time()

            # 1. Drain incoming ZeroMQ frame queue
            while True:
                try:
                    msg = sub_socket.recv(flags=zmq.NOBLOCK)
                    payload = msgpack.unpackb(msg, raw=False)

                    topic = payload.get("topic")
                    # Current publisher uses ``frame`` for MJPEG bytes. Keep
                    # accepting the earlier ``jpeg`` field for compatibility.
                    jpeg_bytes = payload.get("jpeg") or payload.get("frame")

                    if topic in camera_state and jpeg_bytes:
                        np_arr = np.frombuffer(jpeg_bytes, dtype=np.uint8)
                        frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

                        if frame is not None:
                            camera_state[topic]["frame"] = frame
                            camera_state[topic]["last_seen"] = now
                            camera_state[topic]["metadata"] = payload

                except zmq.Again:
                    break
                except Exception as err:
                    print(f"Error processing frame: {err}")
                    break

            # 1b. Drain the latest sensor telemetry without blocking video.
            while True:
                try:
                    sensor_topic, message = sensor_socket.recv_multipart(flags=zmq.NOBLOCK)
                    if sensor_topic == args.sensor_topic.encode("utf-8"):
                        sensor_state["payload"] = parse_telemetry(message)
                        sensor_state["last_seen"] = time.monotonic()
                except zmq.Again:
                    break
                except (UnicodeDecodeError, json.JSONDecodeError, ValueError) as err:
                    print(f"Ignoring invalid sensor telemetry: {err}")

            # 2. Render exactly one window: the currently selected camera.
            topic = viewer_state["active_topic"]
            state = camera_state[topic]
            last_seen = state["last_seen"]
            is_enabled = state["enabled"]
            has_frame = state["frame"] is not None

            if not is_enabled:
                display_frame = create_offline_frame(topic, reason="STREAM PAUSED (DISABLED)")
            elif not has_frame or (now - last_seen > args.timeout):
                reason_msg = (
                    f"DISCONNECTED (Timeout > {args.timeout}s)"
                    if has_frame
                    else "OFFLINE AT STARTUP"
                )
                display_frame = create_offline_frame(topic, reason=reason_msg)
            else:
                # Make an explicit copy to prevent text blur buildup over iterations.
                display_frame = state["frame"].copy()
                cv2.circle(display_frame, (30, 30), 10, (0, 255, 0), -1)
                cv2.putText(
                    display_frame,
                    f"{topic} | LIVE",
                    (50, 36),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.6,
                    (255, 255, 255),
                    2,
                    cv2.LINE_AA,
                )
                metadata = state["metadata"]
                capture_us = metadata.get("timestamp_us")
                quality = metadata.get("time_quality", "unknown")
                if capture_us is not None:
                    capture_text = time.strftime("%Y-%m-%d %H:%M:%S", time.gmtime(capture_us / 1_000_000))
                    capture_text += f".{capture_us % 1_000_000:06d} UTC"
                    cv2.putText(display_frame, f"Capture: {capture_text}", (50, 64), cv2.FONT_HERSHEY_SIMPLEX, 0.48, (255, 255, 255), 1, cv2.LINE_AA)
                cv2.putText(display_frame, f"Time sync: {quality}", (50, 86), cv2.FONT_HERSHEY_SIMPLEX, 0.48, (0, 255, 0) if quality == "synchronized" else (0, 180, 255), 1, cv2.LINE_AA)

            if viewer_state["sensor_overlay_enabled"]:
                draw_sensor_overlay(display_frame, sensor_state["payload"],
                                    sensor_state["last_seen"], args.sensor_timeout)

            cv2.imshow("OAK RGB Viewer", display_frame)

            # 3. Process key actions
            # Redrawing an unchanged frame at 100 Hz wastes CPU. The selected
            # camera is 15 FPS by default, so a 30 Hz GUI limit is ample.
            key = cv2.waitKey(max(1, round(1000 / args.display_fps))) & 0xFF
            if key == 27 or key == ord("q"):
                break

            if key == ord("3"):
                viewer_state["sensor_overlay_enabled"] = not viewer_state["sensor_overlay_enabled"]
                state_text = "shown" if viewer_state["sensor_overlay_enabled"] else "hidden"
                print(f"[SENSORS] Overlay {state_text}")
                continue

            if ord("1") <= key <= ord("9"):
                cam_idx = key - ord("1")
                topics = list(camera_state.keys())
                if cam_idx < len(topics):
                    target_topic = topics[cam_idx]
                    select_camera(target_topic)

    finally:
        cv2.destroyAllWindows()
        sub_socket.close()
        sensor_socket.close()
        for ctl_sock in control_sockets.values():
            ctl_sock.close()
        context.term()
