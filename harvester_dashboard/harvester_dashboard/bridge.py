"""QObject bridge exposing dashboard state and controls to QML.

Everything QML touches lives here as Qt properties/slots so the views
stay declarative.  The bridge deliberately provides **no** slot that
writes to any telemetry socket: view switching is render-only, and
maintenance controls exist but are hidden unless the status source
reports hardware mode (and even then remain inert until a control
endpoint is defined).
"""

from __future__ import annotations

import math
import time
from typing import Any, Dict, List, Optional

import numpy as np

try:
    from PySide2.QtCore import Property, QObject, QTimer, Signal, Slot
    _QT_AVAILABLE = True
except ImportError:  # pragma: no cover - headless pure-python tests
    _QT_AVAILABLE = False
    QObject = object

from .config import DashboardConfig
from .model.telemetry_model import TelemetryModel
from .model.target_model import AnnotationState
from .status_client import StatusClient


if _QT_AVAILABLE:

    class DashboardBridge(QObject):
        """Single context property ``bridge`` for Dashboard.qml."""

        # --- QML-bound notifications ---------------------------------------
        view_changed = Signal()
        hud_visible_changed = Signal()
        diagnostic_visible_changed = Signal()
        lidar_visible_changed = Signal()
        source_badge_changed = Signal()
        ranges_changed = Signal()
        trunk_changed = Signal()
        boom_changed = Signal()
        calibration_changed = Signal()
        stream_rows_changed = Signal()
        annotation_changed = Signal()
        toast_changed = Signal()
        status_summary_changed = Signal()
        maintenance_changed = Signal()
        lidar_points_changed = Signal()
        lidar_view_changed = Signal()
        pointcloud_visible_changed = Signal()
        pointcloud_changed = Signal()
        imu_active_changed = Signal()
        imu_enabled_changed = Signal()
        frame_tick = Signal()

        STATUS_TIMEOUT_MS = 600

        def __init__(self, config: DashboardConfig, model: TelemetryModel,
                     annotation: AnnotationState, annotation_publisher=None,
                     parent=None):
            super().__init__(parent)
            self.config = config
            self.model = model
            self.annotation = annotation
            self.annotation_publisher = annotation_publisher
            self.status_client = (
                StatusClient(config.status_endpoint,
                             timeout_ms=self.STATUS_TIMEOUT_MS)
                if config.status_enabled else None)
            self._view = 'cutter'
            self._hud_visible = True
            self._diagnostic_visible = True
            self._lidar_visible = True
            self._lidar_view_index = 0
            self._pointcloud_visible = True
            self._imu_enabled = True
            self._imu_reference = {'cutter': None, 'docking': None}
            self._imu_attitude = {'cutter': None, 'docking': None}
            self._frame_counters = {'cutter': 0, 'docking': 0}
            self._depth_counters = {'cutter': 0, 'docking': 0}
            self._pointcloud_cache = None
            # Raw (unstabilized) unprojected cloud, cached per camera so an IMU
            # update re-applies only the cheap rotation instead of re-running
            # the ~3.3 ms depth unprojection on the UI thread.
            self._raw_cloud_cache: Dict[str, Any] = {}
            self._latest_frames: Dict[str, Any] = {}
            self._toast = ''
            self._toast_until = 0.0
            self._last_status_response: Optional[Dict[str, Any]] = None
            self._maintenance_mode = 'unknown'
            self._lidar_points: List[List[float]] = []
            self._refresh = QTimer(self)
            self._refresh.timeout.connect(self.refresh)
            self._refresh.start(200)
            if self.status_client is not None:
                self._status_timer = QTimer(self)
                self._status_timer.timeout.connect(self.poll_status)
                self._status_timer.start(int(config.status_interval_s * 1000))
                QTimer.singleShot(0, self.poll_status)

        # =================================================================
        # View switching — render-only by construction: no socket write.
        # =================================================================
        @Slot(str)
        def set_view(self, view: str) -> None:
            if view in ('cutter', 'docking') and view != self._view:
                self._view = view
                # The flattened cloud cache is for the *previous* camera; drop
                # it so the inset re-reads the new camera's cloud immediately
                # (the raw unprojection cache is keyed by camera, so it is fine).
                self._invalidate_pointcloud_cache()
                self.view_changed.emit()

        def _get_view(self) -> str:
            return self._view

        @Slot()
        def toggle_hud(self) -> None:
            self._hud_visible = not self._hud_visible
            self.hud_visible_changed.emit()

        def _get_hud_visible(self) -> bool:
            return self._hud_visible

        @Slot()
        def toggle_diagnostic(self) -> None:
            """Toggle the developer-diagnostic HUD layer (render-only).

            Bound to the ``777`` + Enter key sequence in QML.  Independent of
            the operator HUD (key ``3``): this only hides/shows the health
            overlays (stream table, status line, timestamp line, stale ring).
            No telemetry socket is written.
            """
            self._diagnostic_visible = not self._diagnostic_visible
            self.diagnostic_visible_changed.emit()

        def _get_diagnostic_visible(self) -> bool:
            return self._diagnostic_visible

        @Slot()
        def toggle_lidar(self) -> None:
            self._lidar_visible = not self._lidar_visible
            self.lidar_visible_changed.emit()

        def _get_lidar_visible(self) -> bool:
            return self._lidar_visible

        @Slot()
        def toggle_pointcloud(self) -> None:
            self._pointcloud_visible = not self._pointcloud_visible
            self.pointcloud_visible_changed.emit()

        def _get_pointcloud_visible(self) -> bool:
            return self._pointcloud_visible

        @Slot()
        def toggle_imu(self) -> None:
            """Toggle IMU vibration stabilization (render-only A/B)."""
            self._imu_enabled = not self._imu_enabled
            self._invalidate_pointcloud_cache()
            self.imu_enabled_changed.emit()
            self.pointcloud_changed.emit()

        def _get_imu_enabled(self) -> bool:
            return self._imu_enabled

        # =================================================================
        # LiDAR view cycling — render-only, cycles on key 5.
        # =================================================================
        # Ordered projection modes: top-down, front, left, right, isometric.
        _LIDAR_VIEWS = ('top', 'front', 'left', 'right', 'iso')

        @Slot()
        def cycle_lidar_view(self) -> None:
            self._lidar_view_index = (
                self._lidar_view_index + 1) % len(self._LIDAR_VIEWS)
            self.lidar_view_changed.emit()

        def _get_lidar_view(self) -> str:
            return self._LIDAR_VIEWS[self._lidar_view_index]

        def _get_lidar_view_label(self) -> str:
            return {
                'top': 'top-down (x-y)',
                'front': 'front (x-z)',
                'left': 'left (y-z)',
                'right': 'right (y-z)',
                'iso': 'isometric',
            }.get(self._LIDAR_VIEWS[self._lidar_view_index], '')

        # =================================================================
        # Frame ingestion hook (UI thread; called from TelemetrySource)
        # =================================================================
        def on_frame_decoded(self, channel: str, decoded) -> None:
            self._latest_frames[channel] = decoded
            if channel.endswith('/rgb'):
                camera = 'cutter' if '/cutter/' in channel else 'docking'
                self._frame_counters[camera] += 1
            elif channel.endswith('/depth'):
                camera = 'cutter' if '/cutter/' in channel else 'docking'
                self._depth_counters[camera] += 1
                # Only refresh the point cloud when it is actually visible:
                # unprojecting a 960x540 depth map per depth frame (even at a
                # throttled 5 Hz x 2 cameras) wastes CPU/memory and contributed
                # to the dashboard OOM when the operator was not viewing it.
                if self._pointcloud_visible and camera == self._view:
                    self._invalidate_raw_cloud_cache(camera)
                    self._invalidate_pointcloud_cache()
                    self.pointcloud_changed.emit()
            elif channel == 'v1/lidar/raw':
                self._set_lidar_points(decoded)
            self.frame_tick.emit()

        def on_json_packet(self, channel: str, header, decoded) -> None:
            """UI-thread hook for JSON channels (wired to ``model.on_json``).

            The model invokes ``on_json(channel, header, decoded)``, so the
            ``header`` argument is required even though only ``decoded`` is
            consumed here.  The IMU stream is JSON and never reaches
            ``on_frame_decoded`` (which only carries decoded image/depth/lidar
            frames), so IMU arrivals are handled here.
            """
            if channel.endswith('/imu'):
                self._on_imu(channel, decoded)

        def _on_imu(self, channel: str, decoded) -> None:
            """Record the latest IMU attitude and refresh the active cloud."""
            camera = 'cutter' if '/cutter/' in channel else 'docking'
            # Drop the inactive camera's IMU entirely: only the active camera's
            # cloud is stabilized, so there is no reason to track the inactive
            # camera's attitude (its reference is re-latched on the first
            # active sample after a view switch).
            if camera != self._view:
                return
            if not isinstance(decoded, dict):
                return
            attitude = decoded.get('attitude_rpy_rad')
            if not isinstance(attitude, (list, tuple)) or len(attitude) < 2:
                return
            try:
                roll = float(attitude[0])
                pitch = float(attitude[1])
            except (TypeError, ValueError, OverflowError):
                # Malformed/non-numeric attitude: treat as "no attitude".
                return
            if not (math.isfinite(roll) and math.isfinite(pitch)):
                return
            new_attitude = (roll, pitch)
            previous = self._imu_attitude.get(camera)
            if previous is not None:
                # Only refresh when the attitude actually moved.  IMU is now
                # published at the same ~5 Hz as depth, and each refresh only
                # re-applies the cheap rotation + flatten (depth unprojection is
                # cached), so this epsilon gate still avoids redundant flatten
                # work when the attitude is quiescent.
                delta = abs(roll - previous[0]) + abs(pitch - previous[1])
                if delta < self._IMU_ATTITUDE_EPSILON:
                    return
            self._imu_attitude[camera] = new_attitude
            # Latch the reference attitude on first sample so stabilization
            # removes vibration *relative to* the current (possibly tilted)
            # resting pose, not to absolute gravity-down.
            if self._imu_reference.get(camera) is None:
                self._imu_reference[camera] = new_attitude
            self.imu_active_changed.emit()
            if self._pointcloud_visible:
                self._invalidate_pointcloud_cache()
                self.pointcloud_changed.emit()

        _LIDAR_HUD_SIZE_M = 8.0  # mirror of LidarInset.qml range_limit_m

        # Minimum attitude change (rad, summed |droll|+|dpitch|) that triggers
        # a point-cloud recompute.  IMU is published at ~5 Hz (matching depth).
        _IMU_ATTITUDE_EPSILON = 1e-3

        def _set_lidar_points(self, points) -> None:
            try:
                from .decoders.lidar_decoder import LidarDecoder
                limited = LidarDecoder().limit(
                    points, self.config.lidar_max_points)
            except Exception:
                limited = points
            self._lidar_points = (
                limited.tolist() if limited is not None else [])
            self.lidar_points_changed.emit()

        @Slot(float, float, float, str, result='QVariantList')
        def project_lidar_point(self, x: float, y: float, z: float,
                                view: str) -> list:
            """Project one LiDAR point to screen coords for the given view.

            Returns ``[screen_x, screen_y]`` with the HUD origin (0, 0)
            in the top-left corner, matching the Canvas draw coordinate
            system used by ``LidarInset.qml``.  The caller supplies ``cx``,
            ``cy``, and ``scale`` so the projection can be reused at any
            HUD size.
            """
            from .projection import project_points
            return [project_points([[x, y, z]], view, 0, 0, 1)[0][:2]]

        @Slot(float, float, float, result='QVariantList')
        def project_lidar_point_current(self, x: float, y: float, z: float) -> list:
            """Project one point using the current LiDAR view."""
            return self.project_lidar_point(x, y, z, self._get_lidar_view())

        def latest_rgb(self, camera: str):
            return self._latest_frames.get(
                'v1/camera/{}/rgb'.format(camera))

        def latest_depth(self, camera: str):
            return self._latest_frames.get(
                'v1/camera/{}/depth'.format(camera))

        # =================================================================
        # Camera-relative point cloud (UI-only, derived from depth + rgb +
        # camera_info).  No new wire channel; unprojection happens here.
        # =================================================================
        def latest_pointcloud(self, camera: str):
            """Back-project the latest depth into a camera-frame cloud.

            Returns ``{'points': Nx3, 'colors': Nx3}`` (empty when depth or
            intrinsics are unavailable).  Capped to ``pointcloud_max_points``.
            The depth unprojection is cached in ``_raw_cloud_cache`` (keyed by
            camera) so an IMU update re-applies only the cheap rotation instead
            of re-running the ~3.3 ms unprojection on the UI thread.
            """
            raw = self._raw_cloud_cache.get(camera)
            if raw is None:
                from .decoders.pointcloud import unproject_depth
                depth = self.latest_depth(camera)
                rgb = self.latest_rgb(camera)
                camera_info = self.model.snapshot_camera_info(camera)
                # Pass the RGB delivered size so colour is sampled at the
                # correct RGB pixel when depth is delivered at a different
                # resolution.
                rgb_header = self.model.state(
                    'v1/camera/{}/rgb'.format(camera)).last_header
                rgb_w = (rgb_header or {}).get('width')
                rgb_h = (rgb_header or {}).get('height')
                raw = unproject_depth(
                    depth, rgb, camera_info,
                    max_points=self.config.pointcloud_max_points,
                    rgb_width=rgb_w, rgb_height=rgb_h)
                self._raw_cloud_cache[camera] = raw
            if not self._imu_enabled:
                return raw
            return self._stabilize_cloud(camera, raw)

        def _stabilize_cloud(self, camera: str, cloud):
            """Apply IMU vibration compensation to a point cloud dict."""
            if not isinstance(cloud, dict):
                return cloud
            points = cloud.get('points')
            if points is None or len(points) == 0:
                return cloud
            attitude = self._imu_attitude.get(camera)
            reference = self._imu_reference.get(camera)
            if attitude is None or reference is None:
                return cloud
            from .decoders.imustab import stabilize_points
            return {'points': stabilize_points(points, attitude, reference),
                    'colors': cloud.get('colors')}

        def _get_camera_pointcloud(self):
            """Flatten the active camera's cloud to ``[x,y,z,r,g,b, ...]``.

            Cached: the flattened list is rebuilt only when the underlying
            depth or IMU attitude changes (``pointcloud_changed``), not on
            every QML property read.
            """
            if getattr(self, '_pointcloud_cache', None) is not None:
                return self._pointcloud_cache
            camera = self._view
            cloud = self.latest_pointcloud(camera)
            points = cloud.get('points') if isinstance(cloud, dict) else None
            colors = cloud.get('colors') if isinstance(cloud, dict) else None
            if points is None or len(points) == 0:
                self._pointcloud_cache = []
                return self._pointcloud_cache
            if colors is None or len(colors) != len(points):
                colors = np.zeros((len(points), 3), dtype=np.uint8)
            # Vectorized flatten: build one Nx6 float32 array (xyz + rgb) and
            # call ``.tolist()`` once.  The old ``zip(points.tolist(),
            # colors.tolist())`` + per-element ``int()`` loop was ~3x slower and
            # ran on the UI thread per recompute.  Color channels are stored as
            # float32 here (0-255 is exact in float32); QML rounds them.
            merged = np.empty((len(points), 6), dtype=np.float32)
            merged[:, :3] = points
            merged[:, 3:] = colors
            self._pointcloud_cache = merged.tolist()
            return self._pointcloud_cache

        def _invalidate_pointcloud_cache(self) -> None:
            self._pointcloud_cache = None

        def _invalidate_raw_cloud_cache(self, camera=None) -> None:
            """Drop the cached unprojection (depth changed)."""
            if camera is None:
                self._raw_cloud_cache.clear()
            else:
                self._raw_cloud_cache.pop(camera, None)

        def _get_pointcloud_count(self) -> int:
            return len(self._get_camera_pointcloud())

        def _get_imu_active(self) -> bool:
            return self._imu_attitude.get(self._view) is not None

        def _get_imu_attitude_line(self) -> str:
            attitude = self._imu_attitude.get(self._view)
            if attitude is None:
                return 'imu: —'
            roll_deg = math.degrees(attitude[0])
            pitch_deg = math.degrees(attitude[1])
            state = 'stab on' if self._imu_enabled else 'stab off'
            return 'imu: roll {:+.1f}° pitch {:+.1f}° ({})'.format(
                roll_deg, pitch_deg, state)

        # =================================================================
        # Annotation
        # =================================================================
        def _depth_pixel_for(self, camera: str, u: int, v: int):
            """Map an RGB pixel (u,v) to the aligned depth-map pixel.

            Depth is aligned to the RGB camera and delivered at half the RGB
            resolution (see oak_capture).  We scale by the ratio of the two
            delivered sizes so the mapping stays correct even if the streams
            are configured at other resolutions.  Returns ``(u_depth,
            v_depth)``, or ``None`` if either header is missing.
            """
            rgb_state = self.model.state('v1/camera/{}/rgb'.format(camera))
            depth_state = self.model.state('v1/camera/{}/depth'.format(camera))
            rgb_h = rgb_state.last_header
            depth_h = depth_state.last_header
            if not rgb_h or not depth_h:
                return None
            try:
                rgb_w = int(rgb_h.get('width', 0))
                rgb_hgt = int(rgb_h.get('height', 0))
                dep_w = int(depth_h.get('width', 0))
                dep_hgt = int(depth_h.get('height', 0))
            except (TypeError, ValueError):
                return None
            if not (rgb_w and rgb_hgt and dep_w and dep_hgt):
                return None
            u_depth = int(round(u * dep_w / rgb_w))
            v_depth = int(round(v * dep_hgt / rgb_hgt))
            return u_depth, v_depth

        @Slot(int, int)
        def annotate_click(self, u: int, v: int) -> None:
            camera = self._view
            depth = self.latest_depth(camera)
            camera_info = self.model.snapshot_camera_info(camera)
            header = self.model.state(
                'v1/camera/{}/rgb'.format(camera)).last_header
            frame_id = (header or {}).get('frame_id', '')
            backproject_u = backproject_v = None
            if depth is None:
                depth_m = None
            else:
                mapped = self._depth_pixel_for(camera, u, v)
                if mapped is None:
                    depth_m = None
                else:
                    du, dv = mapped
                    backproject_u, backproject_v = du, dv
                    from .decoders.depth_decoder import DepthDecoder
                    depth_m = DepthDecoder().depth_at(
                        depth, du, dv, window=self.config.annotation_depth_window_px)
            _accepted, message = self.annotation.build(
                camera, u, v, depth_m, camera_info, frame_id,
                backproject_u=backproject_u, backproject_v=backproject_v)
            self._forward_annotation('created')
            self._toast_message(message)
            self.annotation_changed.emit()

        @Slot()
        def clear_annotation(self) -> None:
            had = self.annotation.active
            self.annotation.clear(reason='operator')
            if had:
                self._forward_annotation('cleared')
            self.annotation_changed.emit()

        def _forward_annotation(self, action: str) -> None:
            if self.annotation_publisher is not None:
                self.annotation_publisher.publish_annotation(
                    self.annotation, action=action)

        def _get_annotation_active(self) -> bool:
            return self.annotation.active

        def _get_annotation_label(self) -> str:
            return self.annotation.label()

        def _get_annotation_camera(self) -> str:
            return self.annotation.camera

        def _get_annotation_u(self) -> int:
            return self.annotation.pixel[0]

        def _get_annotation_v(self) -> int:
            return self.annotation.pixel[1]

        # =================================================================
        # Periodic refresh — recompute derived display state
        # =================================================================
        @Slot()
        def refresh(self) -> None:
            self.source_badge_changed.emit()
            self.ranges_changed.emit()
            self.trunk_changed.emit()
            self.boom_changed.emit()
            self.calibration_changed.emit()
            self.stream_rows_changed.emit()
            self.status_summary_changed.emit()
            self.frame_tick.emit()
            if self._toast and time.monotonic() > self._toast_until:
                self._toast = ''
                self.toast_changed.emit()

        # -- source badge ---------------------------------------------------
        def _get_source_badge(self) -> str:
            mode = self.model.source_mode()
            ids = self.model.source_ids()
            return '{} {}'.format(mode, ids).strip()

        def _get_source_mixed(self) -> bool:
            return self.model.is_mixed()

        # -- freshness of the active camera ---------------------------------
        def _active_rgb_channel(self) -> str:
            return 'v1/camera/{}/rgb'.format(self._view)

        def _get_active_camera_stale(self) -> bool:
            return self.model.state(self._active_rgb_channel()).is_stale(
                time.monotonic(), self.config.stale_after_s)

        def _get_active_timestamp_line(self) -> str:
            state = self.model.state(self._active_rgb_channel())
            header = state.last_header
            if header is None:
                return 'camera {}: no packet yet'.format(self._view)
            return '{}: {} ({}) seq {}'.format(
                self._view,
                header.get('acquisition_timestamp_ns', '?'),
                header.get('clock_domain', '?'),
                header.get('sequence', '?'))

        @Slot(str, result=bool)
        def stream_stale(self, channel: str) -> bool:
            return self.model.state(channel).is_stale(
                time.monotonic(), self.config.stale_after_s)

        @Slot(str, result=str)
        def stream_age_line(self, channel: str) -> str:
            age = self.model.state(channel).age_s(time.monotonic())
            return '—' if age is None else '{:.1f}s'.format(age)

        # -- ranges -----------------------------------------------------------
        def _get_docking_range_rows(self):
            records, _cutter = self.model.snapshot_ranges()
            rows = []
            for record in (records or []):
                if not isinstance(record, dict):
                    continue
                rows.append({
                    'key': str(record.get('telemetry_key', '?')),
                    'distance': (None if record.get('distance_m') is None
                                 else float(record['distance_m'])),
                    'valid': bool(record.get('valid', False)),
                })
            return rows

        def _get_cutter_range_line(self) -> str:
            _records, cutter = self.model.snapshot_ranges()
            if not cutter:
                return 'cutter: —'
            distance = cutter.get('distance_m')
            if distance is None:
                return 'cutter: INVALID'
            return 'cutter: {:.2f} m'.format(float(distance))

        # -- boom / leveling / phase guide -------------------------------------
        def _get_boom(self):
            return self.model.snapshot_boom() or {}

        def _get_boom_angle_line(self) -> str:
            boom = self._get_boom()
            angle = boom.get('boom_angle_deg')
            if angle is None:
                return 'boom angle: —'
            return 'boom angle: {:+.1f}°'.format(float(angle))

        def _get_boom_extension_line(self) -> str:
            boom = self._get_boom()
            ext = boom.get('boom_extension_m')
            if ext is None:
                return 'boom ext: —'
            return 'boom ext: {:.2f} m'.format(float(ext))

        def _get_level_line(self) -> str:
            boom = self._get_boom()
            roll = boom.get('platform_roll_deg')
            pitch = boom.get('platform_pitch_deg')
            if roll is None and pitch is None:
                return 'level: —'
            return 'level: roll {:+.2f}° pitch {:+.2f}°'.format(
                float(roll or 0.0), float(pitch or 0.0))

        def _get_phase_guide_line(self) -> str:
            boom = self._get_boom()
            phase = boom.get('phase') or 'NO DATA'
            docked = bool(boom.get('docked', False))
            return 'PHASE: {}'.format(phase) + ('  ✔ DOCKED' if docked else '')

        # -- trunk / calibration ---------------------------------------------
        def _get_trunk_line(self) -> str:
            trunk = self.model.snapshot_trunk()
            if not isinstance(trunk, dict):
                return 'trunk: —'
            position = ((trunk.get('pose') or {}).get('position') or {})
            return 'trunk: ({:+.2f}, {:+.2f}, {:+.2f}) m'.format(
                float(position.get('x', 0.0)),
                float(position.get('y', 0.0)),
                float(position.get('z', 0.0)))

        def _get_calibration_line(self) -> str:
            payload, valid = self.model.snapshot_calibration()
            if payload is None:
                return 'calibration: —'
            status = payload.get('status', '?') if isinstance(payload, dict) else '?'
            calibration_id = (
                payload.get('calibration_id', '')
                if isinstance(payload, dict) else '')
            flag = 'VALID' if valid else 'UNCONFIRMED'
            return 'calibration: {} [{}] {}'.format(
                status, flag, calibration_id)

        # -- stream rows / errors panel ----------------------------------------
        def _get_stream_rows(self):
            rows = self.model.summary_rows(
                stale_after_s=self.config.stale_after_s)
            rendered = []
            for row in rows:
                age = row['age_s']
                rendered.append({
                    'channel': row['channel'],
                    'age': '—' if age is None else '{:.1f}s'.format(age),
                    'stale': row['stale'],
                    'ever_seen': row['ever_seen'],
                    'gaps': row['sequence_gaps'],
                    'drops': row['drops'],
                    'decode_errors': row['decode_errors'],
                    'error': row['last_error'],
                })
            return rendered

        def _get_received_packets(self) -> int:
            return int(getattr(self.model, 'source_received', 0))

        def _get_dropped_packets(self) -> int:
            local = sum(state.drops for state in self.model.states())
            reported = int(getattr(self.model, 'source_dropped', 0))
            return max(local, reported)

        # -- image counters -----------------------------------------------------
        def _get_frame_counter(self) -> int:
            return self._frame_counters.get(self._view, 0)

        def _get_depth_counter(self) -> int:
            return self._depth_counters.get(self._view, 0)

        # -- LiDAR points ---------------------------------------------------------
        def _get_lidar_points(self):
            return self._lidar_points

        # -- status REP ------------------------------------------------------------
        def poll_status(self) -> None:
            if self.status_client is None:
                return
            response = self.status_client.query()
            self._last_status_response = response
            mode = StatusClient.source_mode_of(response)
            if mode != self._maintenance_mode:
                self._maintenance_mode = mode
                self.maintenance_changed.emit()
            self.status_summary_changed.emit()

        def _get_status_line(self) -> str:
            if self.status_client is None:
                return 'status: disabled (replay)'
            response = self._last_status_response
            if response is None:
                return 'status: unreachable'
            drops = response.get('dropped_packets') or {}
            total_drops = sum(
                value if isinstance(value, int) else 0
                for value in drops.values())
            recording = (response.get('recording') or {}).get('enabled', False)
            return 'status: {} | drops {} | rec {}'.format(
                response.get('active_profile', '?'), total_drops,
                'on' if recording else 'off')

        def _get_maintenance_available(self) -> bool:
            return self._maintenance_mode == 'hardware'

        def _get_maintenance_mode(self) -> str:
            return self._maintenance_mode

        # Maintenance actions are intentionally inert: Phase 1 defines no
        # control endpoint client.  They never touch a socket.
        @Slot(str)
        def request_stream_toggle(self, channel: str) -> None:
            self._toast_message(
                'maintenance control endpoint not configured '
                '(source mode: {})'.format(self._maintenance_mode))

        # -- toast -------------------------------------------------------------------
        def _toast_message(self, message: str) -> None:
            self._toast = str(message)
            self._toast_until = time.monotonic() + 4.0
            self.toast_changed.emit()

        def _get_toast(self) -> str:
            return self._toast

        # =====================================================================
        # QML property plumbing
        # =====================================================================
        view = Property(str, _get_view, set_view, notify=view_changed)
        hudVisible = Property(
            bool, _get_hud_visible, notify=hud_visible_changed)
        diagnosticVisible = Property(
            bool, _get_diagnostic_visible, notify=diagnostic_visible_changed)
        lidarVisible = Property(
            bool, _get_lidar_visible, notify=lidar_visible_changed)
        sourceBadge = Property(
            str, _get_source_badge, notify=source_badge_changed)
        sourceMixed = Property(
            bool, _get_source_mixed, notify=source_badge_changed)
        activeCameraStale = Property(
            bool, _get_active_camera_stale, notify=frame_tick)
        activeTimestampLine = Property(
            str, _get_active_timestamp_line, notify=frame_tick)
        dockingRangeRows = Property(
            'QVariantList', _get_docking_range_rows, notify=ranges_changed)
        cutterRangeLine = Property(
            str, _get_cutter_range_line, notify=ranges_changed)
        trunkLine = Property(str, _get_trunk_line, notify=trunk_changed)
        boomAngleLine = Property(str, _get_boom_angle_line, notify=boom_changed)
        boomExtensionLine = Property(
            str, _get_boom_extension_line, notify=boom_changed)
        levelLine = Property(str, _get_level_line, notify=boom_changed)
        phaseGuideLine = Property(str, _get_phase_guide_line, notify=boom_changed)
        calibrationLine = Property(
            str, _get_calibration_line, notify=calibration_changed)
        streamRows = Property(
            'QVariantList', _get_stream_rows, notify=stream_rows_changed)
        receivedPackets = Property(
            int, _get_received_packets, notify=stream_rows_changed)
        droppedPackets = Property(
            int, _get_dropped_packets, notify=stream_rows_changed)
        frameCounter = Property(int, _get_frame_counter, notify=frame_tick)
        depthCounter = Property(int, _get_depth_counter, notify=frame_tick)
        lidarPoints = Property(
            'QVariantList', _get_lidar_points, notify=lidar_points_changed)
        lidarView = Property(
            str, _get_lidar_view, notify=lidar_view_changed)
        lidarViewLabel = Property(
            str, _get_lidar_view_label, notify=lidar_view_changed)
        statusLine = Property(
            str, _get_status_line, notify=status_summary_changed)
        maintenanceAvailable = Property(
            bool, _get_maintenance_available, notify=maintenance_changed)
        maintenanceMode = Property(
            str, _get_maintenance_mode, notify=maintenance_changed)
        annotationActive = Property(
            bool, _get_annotation_active, notify=annotation_changed)
        annotationLabel = Property(
            str, _get_annotation_label, notify=annotation_changed)
        annotationCamera = Property(
            str, _get_annotation_camera, notify=annotation_changed)
        annotationU = Property(
            int, _get_annotation_u, notify=annotation_changed)
        annotationV = Property(
            int, _get_annotation_v, notify=annotation_changed)
        toast = Property(str, _get_toast, notify=toast_changed)
        pointcloudVisible = Property(
            bool, _get_pointcloud_visible, notify=pointcloud_visible_changed)
        cameraPointcloud = Property(
            'QVariantList', _get_camera_pointcloud, notify=pointcloud_changed)
        pointcloudCount = Property(
            int, _get_pointcloud_count, notify=pointcloud_changed)
        imuEnabled = Property(
            bool, _get_imu_enabled, notify=imu_enabled_changed)
        imuActive = Property(
            bool, _get_imu_active, notify=imu_active_changed)
        imuAttitudeLine = Property(
            str, _get_imu_attitude_line, notify=imu_active_changed)


__all__ = ['DashboardBridge', '_QT_AVAILABLE']
