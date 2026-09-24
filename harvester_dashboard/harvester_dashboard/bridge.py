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
from collections import deque
from typing import Any, Dict, List, Optional

import numpy as np

try:
    from PySide2.QtCore import Property, QObject, QTimer, Signal, Slot
    _QT_AVAILABLE = True
except ImportError:  # pragma: no cover - headless pure-python tests
    _QT_AVAILABLE = False
    QObject = object

from .config import DashboardConfig
from .hud_config import HudLayoutConfig, default_hud_layout
from .model.telemetry_model import TelemetryModel
from .model.target_model import AnnotationState
from .status_client import StatusClient

from . import safety_guidance
from .safety_guidance import DANGER, NO_DATA, SAFE, WARN, SafetyConfig
from . import cutter_safety_guidance
from .cutter_safety_guidance import CutterConfig


if _QT_AVAILABLE:

    class DashboardBridge(QObject):
        """Single context property ``bridge`` for Dashboard.qml."""

        # --- QML-bound notifications ---------------------------------------
        view_changed = Signal()
        hud_visible_changed = Signal()
        operator_huds_visible_changed = Signal()
        cutter_hud_visible_changed = Signal()
        diagnostic_visible_changed = Signal()
        lidar_visible_changed = Signal()
        source_badge_changed = Signal()
        ranges_changed = Signal()
        trunk_changed = Signal()
        boom_changed = Signal()
        dock_safety_changed = Signal()
        cutter_safety_changed = Signal()
        mqtt_sensors_changed = Signal()
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
        lidar_imu_changed = Signal()
        scan_changed = Signal()
        frame_tick = Signal()

        STATUS_TIMEOUT_MS = 600

        def __init__(self, config: DashboardConfig, model: TelemetryModel,
                     annotation: AnnotationState, annotation_publisher=None,
                     hud_config: Optional[HudLayoutConfig] = None,
                     safety_config: Optional[SafetyConfig] = None,
                     cutter_config: Optional[CutterConfig] = None,
                     lidar_control=None, parent=None):
            super().__init__(parent)
            self.config = config
            self.model = model
            self.annotation = annotation
            self.annotation_publisher = annotation_publisher
            # Opt-in LiDAR standby/normal control for the operator scan.  When
            # absent (the default) it is created only if the config supplies an
            # endpoint, so a direct construction without one stays render-only.
            if lidar_control is None and config.lidar_control_enabled:
                from .lidar_control import LidarControlPublisher
                lidar_control = LidarControlPublisher(
                    config.lidar_control_endpoint)
            self.lidar_control = lidar_control
            self.hud_config = hud_config or default_hud_layout()
            # Docking safety-guidance thresholds.  None falls back to the
            # shipped tuning file, then to the built-in defaults (the loader
            # never raises), so a direct construction without a config still
            # works.
            self.safety_config = (safety_config if safety_config is not None
                                  else SafetyConfig.load(
                                      safety_guidance.default_config_path()))
            # Cutter safety-guidance thresholds (same contract).
            self.cutter_config = (cutter_config if cutter_config is not None
                                  else CutterConfig.load(
                                      cutter_safety_guidance.default_config_path()))
            self.status_client = (
                StatusClient(config.status_endpoint,
                             timeout_ms=self.STATUS_TIMEOUT_MS)
                if config.status_enabled else None)
            self._view = 'docking'
            self._hud_visible = True
            # Operator HUDs are shown at startup (key 2 toggles them).  This flag
            # gates the boom and docking panels only; each of those is
            # additionally gated on its own ``visible`` config, so the toggle is
            # on while either panel is enabled by config.  cutter_range is
            # deliberately excluded: it is gated on the cutter view rather than
            # this flag, so counting it would turn the toggle on (and light the
            # "Boom/Dock" toolbar button) while showing neither panel.
            self._operator_huds_visible = (
                self.hud_config.boom.visible
                or self.hud_config.docking.visible
            )
            # The Cutter Range HUD is shown while the cutter camera is active;
            # key 1 toggles it when already on the cutter view.
            self._cutter_hud_visible = self.hud_config.cutter_range.visible
            self._diagnostic_visible = False
            # The LiDAR inset is hidden at startup (key 4 toggles it).
            self._lidar_visible = False
            self._lidar_view_index = 0
            # Scan window precedence: an explicit --lidar-scan-seconds wins, else
            # the admin's hud_config lidar_scan.scan_seconds, else 15 s.
            config_seconds = getattr(config, 'lidar_scan_seconds', None)
            if config_seconds is not None:
                self._scan_seconds = float(config_seconds)
            else:
                self._scan_seconds = float(getattr(
                    self.hud_config.lidar_scan, 'scan_seconds', 15.0))
            self._pointcloud_visible = False
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
            # Raw (unstabilized) LiDAR cloud, cached so an IMU update only
            # re-applies the cheap rotation rather than re-decoding the points.
            self._lidar_raw_points: List[List[float]] = []
            # MID-360 IMU attitude (sensor frame) and its latched reference, kept
            # separate from the camera IMU state so the two never mix.
            self._lidar_imu_attitude: Optional[tuple] = None
            self._lidar_imu_reference: Optional[tuple] = None
            # LiDAR scan state machine.  ``idle`` until the operator presses
            # SCAN; ``scanning`` -> ``complete``/``no_data``.
            self._scan_phase = 'idle'
            self._scan_started_at: Optional[float] = None
            self._scan_tree = None
            self._scan_target = None
            self._scan_reason = ''
            # Cached estimate rows, rebuilt only when a scan completes (QML
            # indexes this list per row, so it must not be rebuilt per read).
            self._scan_estimate_rows_cache: List[dict] = []
            # Points accumulated across the scan window.  The estimate is taken
            # from this buffer, not the latest frame, so a long window actually
            # densifies the measurement instead of only showing the last frame.
            self._scan_accumulated: List[List[float]] = []
            # Display state of the last scan_changed emit, so the periodic tick
            # can skip redundant emissions.
            self._last_scan_notify = None
            # Docking safety-guidance state.
            # (receipt_monotonic_s, distance_m) samples for the least-squares
            # closing-speed slope; pruned to _SPEED_MAX_SAMPLE_AGE_S.
            self._center_samples: deque = deque()
            self._last_center_sample: Optional[tuple] = None
            # Identity of the sample window the cached speed was fitted from,
            # so the 200 ms refresh does not re-fit an unchanged window.
            self._dock_speed_fit_key: Optional[tuple] = None
            self._dock_speed_raw: Optional[float] = None      # cm/s
            self._dock_speed_smoothed: Optional[float] = None  # cm/s
            self._dock_center_distance: Optional[float] = None  # m
            # Initial guidance is NO_DATA (never SAFE) so the HUD never shows a
            # reassuring green before any range has arrived.
            self._guidance = safety_guidance.evaluate(
                None, None, self.safety_config)
            self._guidance_state = NO_DATA
            self._guidance_state_since = time.monotonic()
            # Cutter safety-guidance state.  Kept fully independent of the
            # docking section (separate samples/state/phase): the two channels
            # are unrelated and must not share a fit window or a debounce clock.
            self._cutter_samples: deque = deque()
            self._last_cutter_sample: Optional[tuple] = None
            self._cutter_speed_fit_key: Optional[tuple] = None
            self._cutter_speed_raw: Optional[float] = None      # cm/s
            self._cutter_speed_smoothed: Optional[float] = None  # cm/s
            self._cutter_raw_range: Optional[float] = None       # m (raw)
            self._cutter_clearance: Optional[float] = None       # m (offset)
            # Cut-sequence phase (ADVISORY prompt only; never commands motion).
            self._cutter_phase = cutter_safety_guidance.PHASE_IDLE
            # Initial state is NO_DATA / IDLE (never SAFE) so the HUD never
            # shows a reassuring green before any cutter range has arrived.
            self._cutter_guidance = cutter_safety_guidance.evaluate(
                None, None, self.cutter_config)
            self._cutter_state = NO_DATA
            self._cutter_state_since = time.monotonic()
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
        def toggle_operator_huds(self) -> None:
            """Handle key ``2``: the Boom + Docking HUDs.

            While the cutter camera is active, key 2 always returns to the
            docking camera, hides the Cutter Range HUD, and shows the Boom +
            Docking HUDs.  Otherwise it toggles the Boom + Docking HUDs.
            Render-only: this only flips view state, it never writes to a
            telemetry socket.
            """
            if self._view == 'cutter':
                # Returning to docking: cutter HUD hides, boom/docking show.
                self.set_view('docking')
                self._set_cutter_hud_visible(True)
                self._set_operator_huds_visible(True)
                return
            self._set_operator_huds_visible(not self._operator_huds_visible)

        def _set_operator_huds_visible(self, visible: bool) -> None:
            if self._operator_huds_visible == visible:
                return
            self._operator_huds_visible = visible
            self.operator_huds_visible_changed.emit()

        def _get_operator_huds_visible(self) -> bool:
            return self._operator_huds_visible

        @Slot()
        def select_cutter_view(self) -> None:
            """Handle key ``1``: the cutter camera and its range HUD.

            From another view this switches to the cutter camera and shows the
            Cutter Range HUD (the Boom/Docking HUDs hide because they gate on
            the docking view).  Pressed again on the cutter view it hides/shows
            the Cutter Range HUD.  Render-only.
            """
            if self._view != 'cutter':
                self.set_view('cutter')
                self._set_cutter_hud_visible(True)
                return
            self._set_cutter_hud_visible(not self._cutter_hud_visible)

        def _set_cutter_hud_visible(self, visible: bool) -> None:
            if self._cutter_hud_visible == visible:
                return
            self._cutter_hud_visible = visible
            self.cutter_hud_visible_changed.emit()

        def _get_cutter_hud_visible(self) -> bool:
            return self._cutter_hud_visible

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
            if not self._lidar_visible and self._scan_phase == 'scanning':
                # Leaving the overlay while scanning stops the scan rather than
                # leaving the LiDAR running with nothing on screen: keep the
                # data and estimate, and stand the sensor down.
                self.stop_scan()
            self.lidar_visible_changed.emit()

        def _get_lidar_visible(self) -> bool:
            return self._lidar_visible

        def _get_lidar_scan_active(self) -> bool:
            """True while the full-screen LiDAR scan overlay is showing.

            The overlay is the only surface the operator needs during a tree
            scan, so the unrelated HUD layers (operator sensor panels, the
            trunk/calibration column, the camera point-cloud inset) hide while
            it is up.  Derived from the one ``lidarVisible`` flag so the rule
            cannot drift between the QML sites that consume it.
            """
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
        # Ordered projection modes: top-down, front, left, right, isometric,
        # then the camera-optical overlay view (full-screen scan overlay).
        _LIDAR_VIEWS = ('top', 'front', 'left', 'right', 'iso', 'camera')

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
                'camera': 'camera overlay',
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
            elif channel == 'v1/imu/lidar':
                self._on_lidar_imu(decoded)

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

        def _on_lidar_imu(self, decoded) -> None:
            """Record the MID-360 IMU attitude and refresh the LiDAR cloud.

            Mirrors :meth:`_on_imu` for the camera but on **separate** state, so
            the two IMUs never mix.  The sensor-frame roll/pitch is converted to
            the optical convention ``imustab`` expects (see
            ``decoders/livox_imu.py``) before it is stored, and a bad sample is
            treated as "no attitude" rather than rotating the cloud by garbage.
            """
            if not isinstance(decoded, dict):
                return
            from .decoders.livox_imu import sensor_rpy_to_optical
            attitude = sensor_rpy_to_optical(decoded.get('attitude_rpy_rad'))
            if attitude is None:
                return
            previous = self._lidar_imu_attitude
            if previous is not None:
                # Only refresh when the attitude actually moved; re-stabilizing a
                # quiescent 2000-point cloud every IMU sample is wasted work.
                delta = abs(attitude[0] - previous[0]) + abs(attitude[1] - previous[1])
                if delta < self._IMU_ATTITUDE_EPSILON:
                    return
            self._lidar_imu_attitude = attitude
            # Latch the reference on first sample so stabilization removes
            # vibration *relative to* the current (possibly tilted) pose.
            if self._lidar_imu_reference is None:
                self._lidar_imu_reference = attitude
            self.lidar_imu_changed.emit()
            self._apply_lidar_stabilization()

        _LIDAR_HUD_SIZE_M = 8.0  # mirror of the overlay's default range window

        # Upper bound on the accumulated scan buffer (points).  A 15 s window at
        # ~5 frames/s * 2000 capped points is ~150k; 400k leaves headroom while
        # keeping the estimate and its sort/percentile work bounded.
        _SCAN_ACCUM_MAX_POINTS = 400000

        # Grace period (s) after SCAN before a missing cloud counts as NO DATA.
        # The MID-360 needs time to spin up and stream after being enabled, so
        # without this a scan would abort on the first 200 ms tick.
        _SCAN_CLOUD_GRACE_S = 5.0

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
            self._lidar_raw_points = (
                limited.tolist() if limited is not None else [])
            # Accumulate across the scan window so the estimate uses the whole
            # scan, not just the last frame.  Capped so a 15 s window on a
            # fast stream cannot grow without bound.
            if self._scan_phase == 'scanning' and self._lidar_raw_points:
                self._scan_accumulated.extend(self._lidar_raw_points)
                if len(self._scan_accumulated) > self._SCAN_ACCUM_MAX_POINTS:
                    del self._scan_accumulated[:-self._SCAN_ACCUM_MAX_POINTS]
            self._apply_lidar_stabilization()

        def _apply_lidar_stabilization(self) -> None:
            """Re-derive the displayed LiDAR cloud from the cached raw points.

            Applies the same IMU vibration compensation the camera cloud uses
            (key 7 toggles both) so the scan is not smeared by machine
            vibration.  With stabilization off, or no IMU attitude yet, the raw
            (producer-leveled) cloud is shown unchanged.
            """
            raw = self._lidar_raw_points
            attitude = self._lidar_imu_attitude
            reference = self._lidar_imu_reference
            if (self._imu_enabled and raw and attitude is not None
                    and reference is not None):
                try:
                    from .decoders.imustab import stabilize_points
                    stabilized = stabilize_points(
                        np.asarray(raw, dtype=np.float32), attitude, reference)
                    self._lidar_points = stabilized.tolist()
                except Exception:
                    self._lidar_points = list(raw)
            else:
                self._lidar_points = list(raw)
            self.lidar_points_changed.emit()
            # Note: no scan_changed emit here.  This runs per point frame while
            # scanning; the estimate rows only change when a scan completes, and
            # the periodic tick already drives the countdown, so emitting here
            # would re-evaluate every scan-bound QML property for no change.

        @Slot(float, float, float, str, result='QVariantList')
        def project_lidar_point(self, x: float, y: float, z: float,
                                view: str) -> list:
            """Project one LiDAR point to screen coords for the given view.

            Returns ``[screen_x, screen_y]`` with the HUD origin (0, 0)
            in the top-left corner, matching the Canvas draw coordinate
            system used by the LiDAR scan overlay.  The caller supplies ``cx``,
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
            self._update_dock_speed()
            self._recompute_safety_guidance()
            self._update_cutter_speed()
            self._recompute_cutter_guidance()
            # Advance the operator LiDAR scan (render-only state machine).
            self.advance_scan()
            self.source_badge_changed.emit()
            self.ranges_changed.emit()
            self.trunk_changed.emit()
            self.boom_changed.emit()
            self.mqtt_sensors_changed.emit()
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
        # Docking range rows in the same {key, label, value, valid} shape the
        # Boom HUD uses, so the shared SensorHudPanel can render both.  Values
        # come from v1/range/docking (the PLC MQTT laser distances).
        _DOCKING_RANGE_LABELS = {
            'center_line': 'Center',
            'diagonal_left_45deg': '45° Left',
            'diagonal_right_45deg': '45° Right',
            'c_channel_left': 'Left',
            'c_channel_right': 'Right',
        }

        def _get_docking_range_rows(self):
            records, _cutter = self.model.snapshot_ranges()
            rows = []
            for record in (records or []):
                if not isinstance(record, dict):
                    continue
                key = str(record.get('telemetry_key', '?'))
                distance = record.get('distance_m')
                valid = bool(record.get('valid', False)) and distance is not None
                if distance is None:
                    value = '—'
                else:
                    try:
                        value = '{:.3f} m'.format(float(distance))
                    except (TypeError, ValueError):
                        value = '—'
                        valid = False
                rows.append({
                    'key': key,
                    'label': self._DOCKING_RANGE_LABELS.get(key, key),
                    'value': value,
                    'valid': valid,
                })
            return rows

        # -- docking safety guidance -------------------------------------------
        # The guidance is advisory/operator-facing only: it derives a closing
        # speed from the ``center_line`` gap and maps (speed, gap) onto
        # SAFE/WARN/DANGER/NO_DATA.  No socket is ever written from this path.
        #
        # Least-squares window over which the closing speed is estimated.  A
        # shrinking gap over ~1.5 s is the platform's forward closing speed.
        _SPEED_MAX_SAMPLE_AGE_S = 1.5

        def _center_line_distance(self, records=None) -> Optional[float]:
            """Return the forward ``center_line`` gap (m) or None when absent.

            ``records`` may be supplied by the caller so a single
            ``snapshot_ranges()`` per refresh serves both this lookup and the
            docking-range rows.
            """
            if records is None:
                records, _cutter = self.model.snapshot_ranges()
            for record in (records or []):
                if not isinstance(record, dict):
                    continue
                if record.get('telemetry_key') != 'center_line':
                    continue
                if not record.get('valid', False):
                    return None
                distance = record.get('distance_m')
                if distance is None:
                    return None
                try:
                    value = float(distance)
                except (TypeError, ValueError, OverflowError):
                    return None
                # Reject non-finite readings (the wire JSON may carry NaN/Inf):
                # a non-finite gap is not a usable measurement and would also
                # disable the fit cache (NaN != NaN).
                if not math.isfinite(value):
                    return None
                return value
            return None

        def _center_range_receipt_age_s(self) -> Optional[float]:
            """Local receipt age (s) of the last ``v1/range/docking`` packet.

            Staleness must track when the *wire* delivered a range record, not
            when the bridge last read the (unchanged) retained JSON.  Using the
            model's local receipt monotonic time makes a silent range stream age
            out to NO_DATA.
            """
            state = self.model.state('v1/range/docking')
            if state.last_recv_monotonic_s is None:
                return None
            return max(0.0, time.monotonic() - state.last_recv_monotonic_s)

        def _update_dock_speed(self) -> None:
            """Derive the platform closing speed from the center-range gap.

            Samples the gap once per *new* ``v1/range/docking`` wire packet
            (keyed on the model's local receipt time, so a repeated read of the
            retained JSON does not add duplicate samples) and fits the
            least-squares slope over the window.  The closing speed is the
            negated slope (a shrinking gap is a positive closing speed), in
            cm/s.  A missing/invalid/absent center range clears the derived
            values so staleness drives NO_DATA.
            """
            now = time.monotonic()
            state = self.model.state('v1/range/docking')
            receipt = state.last_recv_monotonic_s
            distance = self._center_line_distance()
            stale = (receipt is None
                     or (now - receipt) > self.safety_config.stale_s)
            if distance is None or stale:
                # No valid/fresh reading: age the window out and clear the
                # derived values so the guidance reports NO_DATA.
                self._dock_center_distance = None
                self._dock_speed_raw = None
                self._dock_speed_smoothed = None
                self._dock_speed_fit_key = None
                while (self._center_samples
                       and now - self._center_samples[0][0]
                       > self._SPEED_MAX_SAMPLE_AGE_S):
                    self._center_samples.popleft()
                return

            self._dock_center_distance = distance
            # Sample once per new reading.  Dedup on the (receipt, distance)
            # pair, not the receipt alone: two packets can share a monotonic
            # receipt stamp (coarse clock or a caller-supplied recv time) while
            # carrying different gaps, and dropping the second would freeze the
            # fitted speed at a stale value while the displayed gap moved.
            if ((receipt, distance) != self._last_center_sample
                    or not self._center_samples):
                self._center_samples.append((receipt, distance))
                self._last_center_sample = (receipt, distance)
            while (self._center_samples
                   and now - self._center_samples[0][0]
                   > self._SPEED_MAX_SAMPLE_AGE_S):
                self._center_samples.popleft()

            if len(self._center_samples) < 2:
                # Not enough samples to fit a slope.  Report NO speed (None),
                # never 0.0: a zeroed speed would let a stalled stream evaluate
                # as SAFE (v=0 is "not approaching") while the gap is still
                # fresh, showing a reassuring green on a DANGER approach.
                self._dock_speed_raw = None
                self._dock_speed_fit_key = None
                return
            # Reuse the previous fit when the window has not changed since the
            # last tick (no new sample and nothing pruned), so a 200 ms refresh
            # over a silent-but-not-yet-stale stream does no repeated work.
            fit_key = (len(self._center_samples), self._center_samples[0],
                       self._center_samples[-1])
            if fit_key == self._dock_speed_fit_key:
                return
            times = np.array([s[0] for s in self._center_samples], dtype=np.float64)
            distances = np.array([s[1] for s in self._center_samples],
                                 dtype=np.float64)
            # Least-squares slope of distance vs time: centre BOTH axes
            # (slope = cov(t, d) / var(t)), so samples at t=0,0.1,... are
            # handled correctly.
            times = times - times.mean()
            denominator = float(np.dot(times, times))
            if denominator <= 0.0:
                # All samples share a timestamp (zero variance): no slope is
                # defined.  Cache the key so an unchanged window is not re-fit
                # on every 200 ms tick, and report no speed rather than a
                # fabricated 0.0 (which would read as "not approaching").
                self._dock_speed_fit_key = fit_key
                self._dock_speed_raw = None
                return
            slope = float(np.dot(times, distances - distances.mean())
                          / denominator)
            self._dock_speed_raw = -slope * 100.0
            self._dock_speed_fit_key = fit_key

        def _recompute_safety_guidance(self) -> None:
            """EMA-smooth the speed, evaluate, and apply hysteresis debounce.

            ``danger`` and ``no_data`` are adopted immediately; only
            ``safe``/``warn`` transitions are debounced so the colour cannot
            flicker at a boundary.  When a state is held, the displayed message
            matches the *held* state (never a STOP message on an orange banner).
            """
            cfg = self.safety_config
            now = time.monotonic()
            raw = self._dock_speed_raw
            if raw is None:
                self._dock_speed_smoothed = None
            elif self._dock_speed_smoothed is None:
                self._dock_speed_smoothed = raw
            else:
                alpha = cfg.speed_ema_alpha
                self._dock_speed_smoothed = (
                    alpha * raw + (1.0 - alpha) * self._dock_speed_smoothed)

            # Staleness is authoritative on its own: any missing/fresh-less
            # receipt forces NO_DATA even if a distance or speed is retained, so
            # the HUD can never show a reassuring state on aged-out telemetry.
            age = self._center_range_receipt_age_s()
            stale = age is None or age > cfg.stale_s

            # No fittable speed yet (fewer than two samples in the window) but
            # the gap is still fresh: the platform may still be closing or may
            # have stopped, and we cannot tell.  Evaluate with a 0 cm/s speed so
            # the absolute standoff floor still applies, but never let the
            # missing speed *lower* an already-issued WARN/DANGER: a DANGER
            # approach must not flip to SAFE just because the fit window
            # emptied.  Staleness falls through to the normal evaluation.
            speed_for_eval = self._dock_speed_smoothed
            if (speed_for_eval is None
                    and not stale
                    and self._dock_center_distance is not None):
                speed_for_eval = 0.0

            candidate = safety_guidance.evaluate(
                speed_for_eval, self._dock_center_distance, cfg,
                stale=stale)
            if (self._dock_speed_smoothed is None and not stale
                    and self._guidance_state in (WARN, DANGER)
                    and candidate.state == SAFE):
                # Missing speed must not clear a held WARN/DANGER.
                candidate = candidate.__class__(
                    self._guidance_state, candidate.ttc_s, candidate.speed_cm_s,
                    candidate.distance_m, candidate.stop_distance_m,
                    candidate.max_speed_cm_s, candidate.recommended_speed_cm_s,
                    safety_guidance.state_message(
                        self._guidance_state, candidate))

            new_state = candidate.state
            adopt = (new_state in (DANGER, NO_DATA)
                     or new_state == self._guidance_state
                     or (now - self._guidance_state_since) >= cfg.debounce_s)
            if not adopt:
                return

            if new_state != self._guidance_state:
                self._guidance_state = new_state
                self._guidance_state_since = now

            # Keep the message consistent with the shown (held) state: if the
            # candidate state was not adopted it cannot differ here, so only the
            # adopted state's own message is used.  ``state_message`` guards the
            # case where debounce held the *previous* state.
            if candidate.state == self._guidance_state:
                guidance = candidate
            else:
                guidance = candidate.__class__(
                    self._guidance_state, candidate.ttc_s, candidate.speed_cm_s,
                    candidate.distance_m, candidate.stop_distance_m,
                    candidate.max_speed_cm_s, candidate.recommended_speed_cm_s,
                    safety_guidance.state_message(self._guidance_state, candidate))

            if guidance != self._guidance:
                self._guidance = guidance
                self.dock_safety_changed.emit()

        def _get_dock_safety_state(self) -> str:
            return self._guidance_state

        def _get_dock_guidance_text(self) -> str:
            return self._guidance.message

        def _get_dock_speed_cm_s(self) -> float:
            return (float('nan') if self._dock_speed_raw is None
                    else float(self._dock_speed_raw))

        def _get_dock_speed_smoothed_cm_s(self) -> float:
            return (float('nan') if self._dock_speed_smoothed is None
                    else float(self._dock_speed_smoothed))

        def _get_dock_center_distance_m(self) -> float:
            return (float('nan') if self._dock_center_distance is None
                    else float(self._dock_center_distance))

        def _get_dock_recommended_speed_cm_s(self) -> float:
            value = self._guidance.recommended_speed_cm_s
            return float('nan') if value is None else float(value)

        def _get_dock_max_speed_cm_s(self) -> float:
            value = self._guidance.max_speed_cm_s
            return float('nan') if value is None else float(value)

        def _get_dock_stop_distance_m(self) -> float:
            value = self._guidance.stop_distance_m
            return float('nan') if value is None else float(value)

        def _get_dock_ttc_s(self) -> float:
            value = self._guidance.ttc_s
            return float('nan') if value is None else float(value)

        def _get_dock_safety_row(self):
            """Metric rows for the generic/guidance HUD in the shared shape.

            Values are pre-formatted for display with an em dash when the
            underlying measurement is absent, matching the docking/boom rows.
            Captions come from the ``dock_guidance.rows`` config (falling back
            to the built-in label) so the panel and the config agree.
            """
            guidance = self._guidance
            row_labels = self.hud_config.dock_guidance.rows or {}

            def _label(key: str, fallback: str) -> str:
                value = row_labels.get(key)
                return value if isinstance(value, str) and value else fallback

            def _number(value: Optional[float], fmt: str) -> str:
                if value is None:
                    return '—'
                try:
                    return fmt.format(float(value))
                except (TypeError, ValueError, OverflowError):
                    return '—'

            state_label = {
                'safe': 'APPROACH OK',
                'warn': 'SLOW',
                'danger': 'STOP',
                'no_data': 'NO DATA',
            }.get(self._guidance_state, self._guidance_state.upper())

            # Report the closing speed magnitude with an explicit direction so a
            # receding platform is not shown as a negative "closing" speed.
            if guidance.speed_cm_s is None:
                speed_value = '—'
            elif guidance.speed_cm_s > 0.0:
                speed_value = '{:.1f} cm/s'.format(guidance.speed_cm_s)
            elif guidance.speed_cm_s < 0.0:
                speed_value = 'receding {:.1f} cm/s'.format(-guidance.speed_cm_s)
            else:
                speed_value = '0.0 cm/s'

            rows = [
                {'key': 'state', 'label': _label('state', 'State'),
                 'value': state_label,
                 'valid': self._guidance_state not in (NO_DATA,)},
                {'key': 'speed', 'label': _label('speed', 'Closing Speed'),
                 'value': speed_value,
                 'valid': guidance.speed_cm_s is not None},
                {'key': 'gap', 'label': _label('gap', 'Gap'),
                 'value': _number(guidance.distance_m, '{:.2f} m'),
                 'valid': guidance.distance_m is not None},
                {'key': 'ttc', 'label': _label('ttc', 'TTC'),
                 'value': _number(guidance.ttc_s, '{:.1f} s'),
                 'valid': guidance.ttc_s is not None},
                {'key': 'max_speed', 'label': _label('max_speed', 'Max Safe'),
                 'value': _number(guidance.max_speed_cm_s, '{:.0f} cm/s'),
                 'valid': guidance.max_speed_cm_s is not None},
            ]
            return rows

        # -- cutter safety guidance --------------------------------------------
        # Sibling of the docking guidance, for the cutting arm: the single
        # forward range sensor (``v1/range/cutter``, ``telemetry_key``
        # ``cutter_forward``) measures the tip's clearance to the object.  The
        # sensor sits BEHIND the tip, so the raw reading is offset-corrected by
        # the model (``tip_clearance_m``).  On top of the clearance state a
        # small cut-sequence phase machine prompts the operator
        # (approach -> align -> open -> advance -> cut); it is advisory only and
        # NEVER commands motion.  No socket is ever written from this path.
        #
        # Least-squares window over which the closing speed is estimated (kept
        # separate from the docking window so the two channels never mix).
        _CUTTER_SPEED_MAX_SAMPLE_AGE_S = 1.5

        def _cutter_range_m(self, cutter=None) -> Optional[float]:
            """Return the raw cutter range (m) or None when absent/invalid."""
            if cutter is None:
                _records, cutter = self.model.snapshot_ranges()
            if not isinstance(cutter, dict):
                return None
            if not cutter.get('valid', False):
                return None
            distance = cutter.get('distance_m')
            if distance is None:
                return None
            try:
                value = float(distance)
            except (TypeError, ValueError, OverflowError):
                return None
            # Reject non-finite readings (the wire JSON may carry NaN/Inf).
            if not math.isfinite(value):
                return None
            return value

        def _cutter_range_receipt_age_s(self) -> Optional[float]:
            """Local receipt age (s) of the last ``v1/range/cutter`` packet.

            Staleness tracks when the *wire* delivered a cutter range record,
            not when the bridge last read the retained JSON, so a silent cutter
            stream ages out to NO_DATA.
            """
            state = self.model.state('v1/range/cutter')
            if state.last_recv_monotonic_s is None:
                return None
            return max(0.0, time.monotonic() - state.last_recv_monotonic_s)

        def _update_cutter_speed(self) -> None:
            """Derive the cutter tip closing speed from the raw cutter range.

            Samples the raw range once per *new* ``v1/range/cutter`` wire packet
            and fits the least-squares slope over the window.  The closing speed
            is the negated slope (a shrinking range is a positive closing speed),
            in cm/s.  The offset is applied later by the model, so the derivative
            and the clearance stay consistent.  A missing/invalid/absent cutter
            range clears the derived values so staleness drives NO_DATA.
            """
            now = time.monotonic()
            state = self.model.state('v1/range/cutter')
            receipt = state.last_recv_monotonic_s
            range_m = self._cutter_range_m()
            stale = (receipt is None
                     or (now - receipt) > self.cutter_config.stale_s)
            if range_m is None or stale:
                self._cutter_raw_range = None
                self._cutter_clearance = None
                self._cutter_speed_raw = None
                self._cutter_speed_smoothed = None
                self._cutter_speed_fit_key = None
                while (self._cutter_samples
                       and now - self._cutter_samples[0][0]
                       > self._CUTTER_SPEED_MAX_SAMPLE_AGE_S):
                    self._cutter_samples.popleft()
                return

            self._cutter_raw_range = range_m
            self._cutter_clearance = cutter_safety_guidance.tip_clearance_m(
                range_m, self.cutter_config)
            # Dedup on the (receipt, range) pair: two packets can share a
            # monotonic receipt stamp while carrying different ranges.
            if ((receipt, range_m) != self._last_cutter_sample
                    or not self._cutter_samples):
                self._cutter_samples.append((receipt, range_m))
                self._last_cutter_sample = (receipt, range_m)
            while (self._cutter_samples
                   and now - self._cutter_samples[0][0]
                   > self._CUTTER_SPEED_MAX_SAMPLE_AGE_S):
                self._cutter_samples.popleft()

            if len(self._cutter_samples) < 2:
                # Not enough samples to fit a slope: report NO speed (None),
                # never 0.0 (a zeroed speed would read as "not approaching").
                self._cutter_speed_raw = None
                self._cutter_speed_fit_key = None
                return
            fit_key = (len(self._cutter_samples), self._cutter_samples[0],
                       self._cutter_samples[-1])
            if fit_key == self._cutter_speed_fit_key:
                return
            times = np.array([s[0] for s in self._cutter_samples],
                             dtype=np.float64)
            ranges = np.array([s[1] for s in self._cutter_samples],
                              dtype=np.float64)
            times = times - times.mean()
            denominator = float(np.dot(times, times))
            if denominator <= 0.0:
                # All samples share a timestamp: no slope is defined.
                self._cutter_speed_fit_key = fit_key
                self._cutter_speed_raw = None
                return
            slope = float(np.dot(times, ranges - ranges.mean()) / denominator)
            self._cutter_speed_raw = -slope * 100.0
            self._cutter_speed_fit_key = fit_key

        def _recompute_cutter_guidance(self) -> None:
            """EMA-smooth the speed, advance the phase, evaluate, and debounce.

            ``danger`` and ``no_data`` are adopted immediately; only
            ``safe``/``warn`` transitions are debounced.  When a state is held,
            the displayed message matches the *held* state (never a STOP message
            on an orange banner).
            """
            cfg = self.cutter_config
            now = time.monotonic()
            raw = self._cutter_speed_raw
            if raw is None:
                self._cutter_speed_smoothed = None
            elif self._cutter_speed_smoothed is None:
                self._cutter_speed_smoothed = raw
            else:
                alpha = cfg.speed_ema_alpha
                self._cutter_speed_smoothed = (
                    alpha * raw + (1.0 - alpha) * self._cutter_speed_smoothed)

            # Staleness is authoritative on its own: any missing/aged-out
            # receipt forces NO_DATA even if a range or speed is retained.
            age = self._cutter_range_receipt_age_s()
            stale = age is None or age > cfg.stale_s

            # Missing speed but a fresh clearance: evaluate with 0 cm/s so the
            # absolute clearance floor still applies, without letting the absent
            # speed clear an already-issued WARN/DANGER.
            speed_for_eval = self._cutter_speed_smoothed
            if (speed_for_eval is None and not stale
                    and self._cutter_clearance is not None):
                speed_for_eval = 0.0

            # Advance the measured phase step (approach -> align) from the
            # settled tip.  The phase uses the *fitted* speed, and an unknown
            # speed is treated as "not settled" (inf), never as 0: a tip whose
            # closing speed cannot be measured must not be promoted to
            # "ready to cut".  Operator-confirmed phases hold here.
            phase_speed = (self._cutter_speed_smoothed
                           if self._cutter_speed_smoothed is not None
                           else float('inf'))
            self._cutter_phase = cutter_safety_guidance.next_phase(
                self._cutter_phase, self._cutter_clearance, phase_speed,
                cfg, stale=stale)

            candidate = cutter_safety_guidance.evaluate(
                speed_for_eval, self._cutter_clearance, cfg,
                stale=stale, phase=self._cutter_phase)
            if (self._cutter_speed_smoothed is None and not stale
                    and self._cutter_state in (WARN, DANGER)
                    and candidate.state == SAFE):
                # Missing speed must not clear a held WARN/DANGER.
                candidate = candidate.__class__(
                    self._cutter_state, candidate.phase, candidate.clearance_m,
                    candidate.speed_cm_s, candidate.ttc_s,
                    candidate.stop_distance_m, candidate.max_speed_cm_s,
                    candidate.recommended_speed_cm_s,
                    cutter_safety_guidance.state_message(
                        self._cutter_state, candidate),
                    candidate.phase_message)

            new_state = candidate.state
            adopt = (new_state in (DANGER, NO_DATA)
                     or new_state == self._cutter_state
                     or (now - self._cutter_state_since) >= cfg.debounce_s)
            if not adopt:
                return

            if new_state != self._cutter_state:
                self._cutter_state = new_state
                self._cutter_state_since = now

            if candidate.state == self._cutter_state:
                guidance = candidate
            else:
                guidance = candidate.__class__(
                    self._cutter_state, candidate.phase, candidate.clearance_m,
                    candidate.speed_cm_s, candidate.ttc_s,
                    candidate.stop_distance_m, candidate.max_speed_cm_s,
                    candidate.recommended_speed_cm_s,
                    cutter_safety_guidance.state_message(
                        self._cutter_state, candidate),
                    candidate.phase_message)

            if guidance != self._cutter_guidance:
                self._cutter_guidance = guidance
                self.cutter_safety_changed.emit()

        @Slot()
        def cutter_confirm_phase(self) -> None:
            """Advance the operator-confirmed cut-sequence prompt by one step.

            The scissors/gripper/advance actions have no sensors, so the operator
            confirms them on the HUD.  This is a **no-op while DANGER or
            NO_DATA** (the tip is too close, or there is no range), so the cut
            sequence can never be advanced into a collision.  It only advances a
            prompt; it never commands the arm.
            """
            if self._cutter_state in (DANGER, NO_DATA):
                return
            new_phase = cutter_safety_guidance.advance_phase(self._cutter_phase)
            if new_phase == self._cutter_phase:
                return
            self._cutter_phase = new_phase
            self._recompute_cutter_guidance()
            self.cutter_safety_changed.emit()

        def _get_cutter_safety_state(self) -> str:
            return self._cutter_state

        def _get_cutter_phase(self) -> str:
            return self._cutter_phase

        def _get_cutter_guidance_text(self) -> str:
            return self._cutter_guidance.message

        def _get_cutter_phase_text(self) -> str:
            # Derived live from the current phase (not the debounce-held
            # guidance object), so the banner prompt always matches the phase
            # the CONFIRM STEP button acts on.  The phase advances immediately
            # while the clearance *state* may still be debouncing.
            return cutter_safety_guidance.phase_message(
                self._cutter_phase, self.cutter_config)

        def _get_cutter_clearance_m(self) -> float:
            value = self._cutter_guidance.clearance_m
            if value is None:
                value = self._cutter_clearance
            return float('nan') if value is None else float(value)

        def _get_cutter_raw_range_m(self) -> float:
            return (float('nan') if self._cutter_raw_range is None
                    else float(self._cutter_raw_range))

        def _get_cutter_speed_smoothed_cm_s(self) -> float:
            return (float('nan') if self._cutter_speed_smoothed is None
                    else float(self._cutter_speed_smoothed))

        def _get_cutter_max_speed_cm_s(self) -> float:
            value = self._cutter_guidance.max_speed_cm_s
            return float('nan') if value is None else float(value)

        def _get_cutter_stop_distance_m(self) -> float:
            value = self._cutter_guidance.stop_distance_m
            return float('nan') if value is None else float(value)

        def _get_cutter_ttc_s(self) -> float:
            value = self._cutter_guidance.ttc_s
            return float('nan') if value is None else float(value)

        def _get_cutter_can_confirm(self) -> bool:
            """True when the CONFIRM STEP prompt may advance (not DANGER/NO_DATA)."""
            return (self._cutter_state not in (DANGER, NO_DATA)
                    and self._cutter_phase not in (
                        cutter_safety_guidance.PHASE_IDLE,
                        cutter_safety_guidance.PHASE_APPROACH))

        def _get_cutter_safety_row(self):
            """Metric rows for the cutter guidance HUD in the shared shape.

            Values are pre-formatted with an em dash when absent.  Captions come
            from the ``cutter_guidance.rows`` config (falling back to the
            built-in label) so the panel and the config agree.
            """
            guidance = self._cutter_guidance
            row_labels = self.hud_config.cutter_guidance.rows or {}

            def _label(key: str, fallback: str) -> str:
                value = row_labels.get(key)
                return value if isinstance(value, str) and value else fallback

            def _number(value: Optional[float], fmt: str) -> str:
                if value is None:
                    return '—'
                try:
                    return fmt.format(float(value))
                except (TypeError, ValueError, OverflowError):
                    return '—'

            state_label = {
                'safe': 'TIP CLEAR',
                'warn': 'SLOW',
                'danger': 'STOP',
                'no_data': 'NO DATA',
            }.get(self._cutter_state, self._cutter_state.upper())

            # Report the closing speed magnitude with an explicit direction so a
            # receding tip is not shown as a negative "closing" speed.
            speed = guidance.speed_cm_s
            if speed is None:
                speed_value = '—'
            elif speed > 0.0:
                speed_value = '{:.1f} cm/s'.format(speed)
            elif speed < 0.0:
                speed_value = 'receding {:.1f} cm/s'.format(-speed)
            else:
                speed_value = '0.0 cm/s'

            phase_label = {
                cutter_safety_guidance.PHASE_IDLE: 'Idle',
                cutter_safety_guidance.PHASE_APPROACH: 'Approach',
                cutter_safety_guidance.PHASE_ALIGN: 'Align',
                cutter_safety_guidance.PHASE_OPEN: 'Open',
                cutter_safety_guidance.PHASE_ADVANCE: 'Advance',
                cutter_safety_guidance.PHASE_CUT: 'Cut',
            }.get(self._cutter_phase, self._cutter_phase)

            return [
                {'key': 'state', 'label': _label('state', 'State'),
                 'value': state_label,
                 'valid': self._cutter_state not in (NO_DATA,)},
                {'key': 'phase', 'label': _label('phase', 'Phase'),
                 'value': phase_label,
                 'valid': self._cutter_phase != cutter_safety_guidance.PHASE_IDLE},
                {'key': 'clearance', 'label': _label('clearance', 'Clearance'),
                 'value': _number(guidance.clearance_m, '{:.2f} m'),
                 'valid': guidance.clearance_m is not None},
                {'key': 'speed', 'label': _label('speed', 'Closing Speed'),
                 'value': speed_value,
                 'valid': guidance.speed_cm_s is not None},
                {'key': 'ttc', 'label': _label('ttc', 'TTC'),
                 'value': _number(guidance.ttc_s, '{:.1f} s'),
                 'valid': guidance.ttc_s is not None},
                {'key': 'max_speed', 'label': _label('max_speed', 'Max Safe'),
                 'value': _number(guidance.max_speed_cm_s, '{:.0f} cm/s'),
                 'valid': guidance.max_speed_cm_s is not None},
            ]

        # -- boom / leveling / phase guide -------------------------------------
        def _get_boom(self):
            return self.model.snapshot_boom() or {}

        def _get_boom_rows(self):
            """Structured boom rows {key, label, value, valid} for the Boom HUD.

            Values come from the PLC MQTT subscriber, carried on
            ``v1/boom/state`` (see ``mqtt_ingest.map_boom_state``).
            """
            boom = self._get_boom()
            rows = []

            def _angle(key: str, label: str):
                value = boom.get(key)
                if value is None:
                    rows.append({'key': key, 'label': label,
                                 'value': '—', 'valid': False})
                    return
                try:
                    rows.append({'key': key, 'label': label,
                                 'value': '{:+.2f}°'.format(float(value)),
                                 'valid': True})
                except (TypeError, ValueError):
                    rows.append({'key': key, 'label': label,
                                 'value': '—', 'valid': False})

            # Boom angle, boom length, slew angle.
            _angle('boom_angle_deg', 'Boom Angle')

            length = boom.get('boom_extension_m')
            if length is None:
                rows.append({'key': 'boom_extension_m', 'label': 'Boom Length',
                             'value': '—', 'valid': False})
            else:
                try:
                    rows.append({'key': 'boom_extension_m', 'label': 'Boom Length',
                                 'value': '{:.3f} m'.format(float(length)),
                                 'valid': True})
                except (TypeError, ValueError):
                    rows.append({'key': 'boom_extension_m', 'label': 'Boom Length',
                                 'value': '—', 'valid': False})

            _angle('slew_angle_deg', 'Slew Angle')

            # Platform tilt X1/Y1 and prime mover tilt X2/Y2.
            _angle('platform_tilt_x1_deg', 'Platform Tilt X1')
            _angle('platform_tilt_y1_deg', 'Platform Tilt Y1')
            _angle('primemover_tilt_x2_deg', 'Prime Mover Tilt X2')
            _angle('primemover_tilt_y2_deg', 'Prime Mover Tilt Y2')
            return rows

        def _get_cutter_range_row(self):
            """Single-row cutter range data for the Cutter Range HUD."""
            _records, cutter = self.model.snapshot_ranges()
            if not cutter:
                return [{'key': 'cutter_range', 'label': 'Cutter',
                         'value': '—', 'valid': False}]
            distance = cutter.get('distance_m')
            return [{
                'key': 'cutter_range',
                'label': 'Cutter',
                'value': ('INVALID' if distance is None
                          else '{:.2f} m'.format(float(distance))),
                'valid': distance is not None,
            }]

        def _get_hud_layout(self):
            """QVariantMap of the admin HUD layout for QML."""
            return self.hud_config.to_qml()

        def _get_phase_guide_line(self) -> str:
            boom = self._get_boom()
            phase = boom.get('phase') or 'NO DATA'
            docked = bool(boom.get('docked', False))
            return 'PHASE: {}'.format(phase) + ('  ✔ DOCKED' if docked else '')

        # -- MQTT sensor values (maintenance/diagnostic layer) -----------------
        # Renders every sensor value subscribed from the MQTT broker
        # (harvester/sensors/v1), carried on v1/boom/state and
        # v1/range/docking.  Each row is {label, value} with the value already
        # formatted for display (or '—' when absent/invalid).
        _MQTT_BOOM_FIELDS = (
            ('phase', 'Run phase', None, False),
            ('boom_angle_deg', 'Boom angle', '°', True),
            ('boom_extension_m', 'Boom length', ' m', False),
            ('slew_angle_deg', 'Slew angle', '°', True),
            ('platform_tilt_x1_deg', 'Platform tilt X1', '°', True),
            ('platform_tilt_y1_deg', 'Platform tilt Y1', '°', True),
            ('primemover_tilt_x2_deg', 'Prime mover tilt X2', '°', True),
            ('primemover_tilt_y2_deg', 'Prime mover tilt Y2', '°', True),
        )

        def _get_mqtt_sensor_rows(self):
            """Build the raw MQTT sensor value rows for the diagnostic HUD.

            The ``v1/boom/state`` payload carries the platform/primemover tilt,
            boom angle/extension, and slew angle published by the mqtt_ingest
            adapter; ``v1/range/docking`` carries the three laser docking
            distances (45° left, center, 45° right).
            Values are rendered verbatim (no unit conversion) so a developer can
            inspect exactly what the MQTT source reported.
            """
            boom = self._get_boom()
            rows = []
            for key, label, unit, signed in self._MQTT_BOOM_FIELDS:
                value = boom.get(key)
                if key == 'phase':
                    rows.append({'label': label,
                                 'value': '—' if value is None else str(value)})
                    continue
                if value is None:
                    rows.append({'label': label, 'value': '—'})
                    continue
                try:
                    number = float(value)
                except (TypeError, ValueError):
                    rows.append({'label': label, 'value': '—'})
                    continue
                if signed:
                    rows.append({'label': label,
                                 'value': '{:+.3f}{}'.format(number, unit)})
                else:
                    rows.append({'label': label,
                                 'value': '{:.3f}{}'.format(number, unit)})

            # Laser docking ranges from v1/range/docking.
            records, _cutter = self.model.snapshot_ranges()
            distances = {}
            for record in (records or []):
                if isinstance(record, dict):
                    distance = record.get('distance_m')
                    if distance is not None:
                        try:
                            distances[record.get('telemetry_key')] = float(distance)
                        except (TypeError, ValueError):
                            pass
            for key, label in (('diagonal_left_45deg', 'Laser 45° left'),
                               ('center_line', 'Laser center'),
                               ('diagonal_right_45deg', 'Laser 45° right')):
                value = distances.get(key)
                rows.append({
                    'label': label,
                    'value': '—' if value is None else '{:.3f} m'.format(value),
                })
            return rows

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

        # =====================================================================
        # LiDAR scan state machine (render-only; never writes a socket).
        # =====================================================================
        # The operator drives the scan by hand on the Orin; there is no
        # simulation.  The HUD shows guided instructions while scanning and
        # swaps to the estimate rows when the scan completes.  Everything here
        # is advisory: the estimate is geometry, not a measured contact, and the
        # five measured range sensors remain the authoritative docking guard.
        _SCAN_GUIDE_GUARDING = (
            'Aim the camera at the trunk. Keep the crown and the trunk end in '
            'view, then hold the machine steady.')

        def _scan_changed_emit(self) -> None:
            # Record the display state that this emit reflects, so the periodic
            # tick can skip a redundant emit when nothing visible changed.
            self._last_scan_notify = self._scan_display_key()
            self.scan_changed.emit()

        def _scan_display_key(self):
            """A key of everything ``scan_changed``-bound QML can render.

            The countdown is quantised to 0.1 s so the periodic 200 ms tick
            still animates it, while a sub-0.1 s elapsed change never fires a
            redundant repaint.
            """
            return (
                self._scan_phase,
                len(self._scan_estimate_rows_cache),
                round(self._get_scan_countdown_s(), 1),
                round(self._get_scan_progress(), 3),
            )

        def _scan_notify_if_changed(self) -> None:
            key = self._scan_display_key()
            if key == getattr(self, '_last_scan_notify', None):
                return
            self._last_scan_notify = key
            self.scan_changed.emit()

        def _lidar_set_enabled(self, enabled: bool) -> None:
            """Ask the LiDAR producer for standby (False) or normal (True).

            No-op when no control endpoint is configured (the dashboard stays
            render-only), so this is the single guarded place the scan writes a
            socket.  It only ever sends ``{"enabled": bool}``.
            """
            publisher = getattr(self, 'lidar_control', None)
            if publisher is None:
                return
            try:
                publisher.set_enabled(enabled)
            except Exception:
                # A control failure must never break the HUD: the scan simply
                # reports no_data if the sensor does not start.
                pass

        @Slot()
        def begin_scan(self) -> None:
            """Start a fresh guided scan.

            Puts the LiDAR into normal mode (standby -> scanning), zeroes the
            timer, clears the accumulated buffer, and enters ``scanning``.  The
            estimate runs when the operator presses STOP or the window times out
            (see :meth:`advance_scan`).  Pressing SCAN again restarts the window
            from zero.
            """
            if not self._lidar_visible:
                # A scan can only be guided over a visible overlay.
                self._lidar_visible = True
                self.lidar_visible_changed.emit()
            self._scan_phase = 'scanning'
            self._scan_started_at = time.monotonic()
            self._scan_tree = None
            self._scan_target = None
            self._scan_reason = ''
            self._scan_estimate_rows_cache = []
            self._scan_accumulated = []
            self._lidar_set_enabled(True)
            self._scan_changed_emit()

        @Slot()
        def stop_scan(self) -> None:
            """End the scan early: stop the LiDAR, keep the data, estimate now.

            Used by the STOP button and by the operator leaving the overlay
            while a scan is running.  Returns the LiDAR to standby.
            """
            if self._scan_phase != 'scanning':
                return
            self._lidar_set_enabled(False)
            self._evaluate_scan()

        @Slot()
        def cancel_scan(self) -> None:
            """Discard the scan and return the LiDAR to standby.

            Clears the timer and the accumulated buffer without estimating, so
            the operator can start a fresh scan.
            """
            self._scan_phase = 'idle'
            self._scan_started_at = None
            self._scan_tree = None
            self._scan_target = None
            self._scan_reason = ''
            self._scan_estimate_rows_cache = []
            self._scan_accumulated = []
            self._lidar_set_enabled(False)
            self._scan_changed_emit()

        def _scan_elapsed_s(self) -> float:
            if self._scan_started_at is None:
                return 0.0
            return max(0.0, time.monotonic() - self._scan_started_at)

        def _scan_has_cloud(self) -> bool:
            """True when the LiDAR stream is delivering a usable cloud.

            A cloud that is absent/stale/empty is NO DATA, never a completed
            scan: showing an estimate computed from nothing would be a
            reassuring-looking lie (the same rule the docking guidance uses).
            """
            if not self._lidar_points:
                return False
            state = self.model.state('v1/lidar/raw')
            return not state.is_stale(
                time.monotonic(), self.config.stale_after_s)

        def advance_scan(self) -> None:
            """Advance the scan state machine; called from ``refresh``.

            While ``scanning`` this only drives the countdown display.  The scan
            ends either when the operator presses STOP (:meth:`stop_scan`) or
            when ``scan_seconds`` elapses: both return the LiDAR to standby,
            keep the accumulated data, and run the estimate.  A stream that never
            produces a usable cloud is ``no_data``, never a completed scan.
            """
            if self._scan_phase != 'scanning':
                return
            if not self._scan_has_cloud():
                # Give the LiDAR time to spin up and stream: a sensor just
                # enabled by SCAN takes seconds to produce its first cloud, so a
                # one-tick check would abort the scan immediately.  Only after
                # the grace period is a missing cloud a real NO DATA.
                if self._scan_elapsed_s() < self._SCAN_CLOUD_GRACE_S:
                    self._scan_notify_if_changed()
                    return
                # No usable stream: do not pretend to progress, and stand the
                # LiDAR down so a dead/absent sensor is not left enabled.
                self._lidar_set_enabled(False)
                self._scan_phase = 'no_data'
                self._scan_reason = 'no LiDAR cloud (disabled or out of range)'
                self._scan_changed_emit()
                return
            elapsed = self._scan_elapsed_s()
            if elapsed < self._scan_seconds:
                self._scan_notify_if_changed()
                return
            # Window elapsed: stand the LiDAR down, then estimate from the
            # accumulated cloud (same path as an early STOP).
            self._lidar_set_enabled(False)
            self._evaluate_scan()

        def _evaluate_scan(self) -> None:
            """Compute the tree/boom estimate and complete (or fail) the scan.

            Estimates from the **accumulated** cloud (the whole scan window),
            falling back to the latest frame only if nothing accumulated — so a
            long window genuinely densifies the measurement.
            """
            from .model.scan_estimate import ScanEstimateConfig, plan_scan
            cfg = ScanEstimateConfig(
                docking_offset_below_top_m=float(getattr(
                    self.config, 'docking_offset_below_top_m', 2.0)),
                default_horizontal_distance_m=self._trunk_horizontal_distance_m(),
            )
            cloud = self._scan_accumulated or self._lidar_points
            try:
                tree, target = plan_scan(
                    cloud,
                    horizontal_distance_m=cfg.default_horizontal_distance_m,
                    frame='sensor', cfg=cfg)
            except Exception as error:  # pragma: no cover - defensive
                self._scan_phase = 'no_data'
                self._scan_reason = 'estimate failed: {}'.format(error)
                self._scan_tree = None
                self._scan_target = None
                self._scan_changed_emit()
                return
            self._scan_tree = tree
            self._scan_target = target
            if tree.valid:
                self._scan_phase = 'complete'
                self._scan_reason = ''
            else:
                self._scan_phase = 'no_data'
                self._scan_reason = tree.reason
            # Build the estimate rows once, here, and cache them: the QML card
            # indexes this list per row per repaint, so rebuilding per read was
            # a repeated allocation on the UI thread.
            rows = list(tree.to_rows())
            if target is not None:
                rows.extend(target.to_rows())
            self._scan_estimate_rows_cache = rows
            self._scan_changed_emit()

        def _trunk_horizontal_distance_m(self) -> Optional[float]:
            """Best-effort trunk horizontal distance from the trunk estimate.

            Uses the ``v1/docking/trunk_estimate`` pose when present (the same
            source the trunk line reads), else ``None`` so the boom IK reports
            NO DATA for the distance rather than inventing one.
            """
            trunk = self.model.snapshot_trunk()
            if not isinstance(trunk, dict):
                return None
            position = ((trunk.get('pose') or {}).get('position') or {})
            try:
                x = float(position.get('x'))
                y = float(position.get('y'))
            except (TypeError, ValueError, OverflowError):
                return None
            distance = math.hypot(x, y)
            return distance if math.isfinite(distance) and distance > 0.0 else None

        def _get_scan_phase(self) -> str:
            return self._scan_phase

        def _get_lidar_control_enabled(self) -> bool:
            """True when the HUD can put the LiDAR into normal/standby mode.

            False means the dashboard is render-only (no ``--lidar-control``):
            the scan still runs and estimates from whatever cloud arrives, but
            the SCAN/STOP/CANCEL buttons do not change the sensor state.
            """
            publisher = getattr(self, 'lidar_control', None)
            return bool(publisher is not None and publisher.enabled)

        def _get_scan_progress(self) -> float:
            if self._scan_phase == 'complete':
                return 1.0
            if self._scan_phase in ('idle', 'no_data'):
                return 0.0
            if self._scan_seconds <= 0.0:
                return 1.0
            return min(1.0, max(0.0, self._scan_elapsed_s() / self._scan_seconds))

        def _get_scan_countdown_s(self) -> float:
            if self._scan_phase in ('idle', 'no_data', 'complete'):
                return 0.0
            return max(0.0, self._scan_seconds - self._scan_elapsed_s())

        def _get_scan_guide_text(self) -> str:
            if self._scan_phase == 'idle':
                return ('Press SCAN to start a fresh scan. LiDAR is in standby. '
                        + self._SCAN_GUIDE_GUARDING)
            if self._scan_phase == 'scanning':
                return ('SCANNING… {:.1f} s left — press STOP when the cloud '
                        'looks dense enough.'
                        .format(self._get_scan_countdown_s()))
            if self._scan_phase == 'no_data':
                return 'NO USABLE GEOMETRY — {}'.format(
                    self._scan_reason or 'check the LiDAR and aim at the trunk')
            return 'SCAN COMPLETE — LiDAR returned to standby. Press SCAN to rescan.'

        def _get_scan_quality_text(self) -> str:
            # Live while scanning (so the operator can judge when to press
            # STOP), and the final accumulated total once complete.
            if self._scan_phase == 'scanning':
                count = len(self._scan_accumulated)
                if count < 5000:
                    density = 'sparse'
                elif count < 40000:
                    density = 'filling'
                else:
                    density = 'dense — good to STOP'
                return '{} pts accumulated ({})'.format(count, density)
            count = len(self._scan_accumulated) or len(self._lidar_points)
            imu = 'imu on' if self._lidar_imu_active() else 'imu —'
            return '{} pts  •  {}  •  view {}'.format(
                count, imu, self._get_lidar_view_label())

        def _get_scan_estimate_rows(self):
            """Estimate rows for the completed scan (tree + boom targets).

            Hidden while scanning: only ``complete`` yields rows, so the scan
            instructions and the estimate values never render together.  The
            list is built once in :meth:`_evaluate_scan` and cached: QML reads
            this property once per row *per repaint* (the ``Repeater`` delegate
            indexes ``bridge.scanEstimateRows``), so rebuilding it on every read
            was a per-frame allocation spike.
            """
            if self._scan_phase != 'complete' or self._scan_tree is None:
                return []
            return self._scan_estimate_rows_cache

        def _get_scan_status_text(self) -> str:
            """One-line advisory status for the estimate card."""
            if self._scan_phase != 'complete':
                return ''
            if self._scan_target is None:
                return 'ADVISORY ONLY — verify before moving'
            return ('ADVISORY ONLY — {} — verify before moving'
                    .format(self._scan_target.status))

        # -- LiDAR IMU -------------------------------------------------------
        def _lidar_imu_active(self) -> bool:
            return self._lidar_imu_attitude is not None

        def _get_lidar_imu_active(self) -> bool:
            return self._lidar_imu_active()

        def _get_lidar_imu_attitude_line(self) -> str:
            attitude = self._lidar_imu_attitude
            if attitude is None:
                return 'lidar imu: —'
            roll_deg = math.degrees(attitude[0])
            pitch_deg = math.degrees(attitude[1])
            state = 'stab on' if self._imu_enabled else 'stab off'
            return 'lidar imu: roll {:+.1f}° pitch {:+.1f}° ({})'.format(
                roll_deg, pitch_deg, state)

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
        operatorHudsVisible = Property(
            bool, _get_operator_huds_visible,
            notify=operator_huds_visible_changed)
        cutterHudVisible = Property(
            bool, _get_cutter_hud_visible,
            notify=cutter_hud_visible_changed)
        hudLayout = Property(
            'QVariantMap', _get_hud_layout, constant=True)
        diagnosticVisible = Property(
            bool, _get_diagnostic_visible, notify=diagnostic_visible_changed)
        lidarVisible = Property(
            bool, _get_lidar_visible, notify=lidar_visible_changed)
        # True while the full-screen LiDAR scan overlay is showing.  When it is,
        # the unrelated HUD layers hide so the cloud reads cleanly; this is a
        # single derived flag so the rule has one definition to test.
        lidarScanActive = Property(
            bool, _get_lidar_scan_active, notify=lidar_visible_changed)
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
        trunkLine = Property(str, _get_trunk_line, notify=trunk_changed)
        boomRows = Property(
            'QVariantList', _get_boom_rows, notify=boom_changed)
        cutterRangeRow = Property(
            'QVariantList', _get_cutter_range_row, notify=ranges_changed)
        phaseGuideLine = Property(str, _get_phase_guide_line, notify=boom_changed)
        dockSafetyRow = Property(
            'QVariantList', _get_dock_safety_row, notify=dock_safety_changed)
        dockSafetyState = Property(
            str, _get_dock_safety_state, notify=dock_safety_changed)
        dockGuidanceText = Property(
            str, _get_dock_guidance_text, notify=dock_safety_changed)
        dockSpeedCmS = Property(
            float, _get_dock_speed_cm_s, notify=dock_safety_changed)
        dockSpeedSmoothedCmS = Property(
            float, _get_dock_speed_smoothed_cm_s, notify=dock_safety_changed)
        dockCenterDistanceM = Property(
            float, _get_dock_center_distance_m, notify=dock_safety_changed)
        dockRecommendedSpeedCmS = Property(
            float, _get_dock_recommended_speed_cm_s, notify=dock_safety_changed)
        dockMaxSpeedCmS = Property(
            float, _get_dock_max_speed_cm_s, notify=dock_safety_changed)
        dockStopDistanceM = Property(
            float, _get_dock_stop_distance_m, notify=dock_safety_changed)
        dockTtcS = Property(
            float, _get_dock_ttc_s, notify=dock_safety_changed)
        cutterSafetyState = Property(
            str, _get_cutter_safety_state, notify=cutter_safety_changed)
        cutterPhase = Property(
            str, _get_cutter_phase, notify=cutter_safety_changed)
        cutterGuidanceText = Property(
            str, _get_cutter_guidance_text, notify=cutter_safety_changed)
        cutterPhaseText = Property(
            str, _get_cutter_phase_text, notify=cutter_safety_changed)
        cutterClearanceM = Property(
            float, _get_cutter_clearance_m, notify=cutter_safety_changed)
        cutterRawRangeM = Property(
            float, _get_cutter_raw_range_m, notify=cutter_safety_changed)
        cutterSpeedSmoothedCmS = Property(
            float, _get_cutter_speed_smoothed_cm_s, notify=cutter_safety_changed)
        cutterMaxSpeedCmS = Property(
            float, _get_cutter_max_speed_cm_s, notify=cutter_safety_changed)
        cutterStopDistanceM = Property(
            float, _get_cutter_stop_distance_m, notify=cutter_safety_changed)
        cutterTtcS = Property(
            float, _get_cutter_ttc_s, notify=cutter_safety_changed)
        cutterCanConfirm = Property(
            bool, _get_cutter_can_confirm, notify=cutter_safety_changed)
        cutterSafetyRow = Property(
            'QVariantList', _get_cutter_safety_row,
            notify=cutter_safety_changed)
        mqttSensorRows = Property(
            'QVariantList', _get_mqtt_sensor_rows, notify=mqtt_sensors_changed)
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
        lidarImuActive = Property(
            bool, _get_lidar_imu_active, notify=lidar_imu_changed)
        lidarImuAttitudeLine = Property(
            str, _get_lidar_imu_attitude_line, notify=lidar_imu_changed)
        scanPhase = Property(
            str, _get_scan_phase, notify=scan_changed)
        lidarControlEnabled = Property(
            bool, _get_lidar_control_enabled, notify=lidar_visible_changed)
        scanProgress = Property(
            float, _get_scan_progress, notify=scan_changed)
        scanCountdownS = Property(
            float, _get_scan_countdown_s, notify=scan_changed)
        scanGuideText = Property(
            str, _get_scan_guide_text, notify=scan_changed)
        scanQualityText = Property(
            str, _get_scan_quality_text, notify=scan_changed)
        scanEstimateRows = Property(
            'QVariantList', _get_scan_estimate_rows, notify=scan_changed)
        scanStatusText = Property(
            str, _get_scan_status_text, notify=scan_changed)
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
