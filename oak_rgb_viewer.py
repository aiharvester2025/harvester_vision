#!/usr/bin/env python3
import argparse
import json
import os
import time

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
    args = parser.parse_args()
    if args.display_fps <= 0:
        parser.error("--display-fps must be positive")

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

    camera_state = {}
    control_sockets = {}
    viewer_state = {"active_topic": active_topic}

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
    print(" Press 'q' or ESC : Quit viewer\n")

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

            cv2.imshow("OAK RGB Viewer", display_frame)

            # 3. Process key actions
            # Redrawing an unchanged frame at 100 Hz wastes CPU. The selected
            # camera is 15 FPS by default, so a 30 Hz GUI limit is ample.
            key = cv2.waitKey(max(1, round(1000 / args.display_fps))) & 0xFF
            if key == 27 or key == ord("q"):
                break

            if ord("1") <= key <= ord("9"):
                cam_idx = key - ord("1")
                topics = list(camera_state.keys())
                if cam_idx < len(topics):
                    target_topic = topics[cam_idx]
                    select_camera(target_topic)

    finally:
        cv2.destroyAllWindows()
        sub_socket.close()
        for ctl_sock in control_sockets.values():
            ctl_sock.close()
        context.term()
