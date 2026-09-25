// Full-screen operator LiDAR scan overlay.
//
// Draws the MID-360 point cloud over the live camera image, anchored at the
// camera's *displayed image rect* centre (so the object seen through the camera
// is where the cloud draws).  While a scan is running it shows guided
// instructions; when the scan completes it hides the instructions and shows the
// estimate rows (tree height + boom targets).  Advisory only: it never commands
// motion.
//
// Layering: this overlay is created before HudOverlay in Dashboard.qml, so the
// safety panels always paint on top of the cloud.  QtQuick 2 primitives only
// (PySide2 5.14 has no Controls2).
//
// Projection is orthographic and, in the "camera" view, requires the
// camera<->LiDAR extrinsic from the calibration file; until that survey is
// approved the camera overlay is an aid, not a calibrated measurement.
import QtQuick 2.12

Item {
    id: overlay

    // The letterboxed camera image rect, supplied by CameraView so the cloud
    // shares the image's centre.
    property real display_x: 0
    property real display_y: 0
    property real display_w: 0
    property real display_h: 0

    // Half-width of the projected window in metres (zoom).
    property real range_limit_m: 12.0
    readonly property real range_min_m: 3.0
    readonly property real range_max_m: 40.0

    // Layout config (bridge.hudLayout.lidar_scan).
    readonly property var layout: bridge.hudLayout.lidar_scan

    function zoomBy(factor) {
        var value = overlay.range_limit_m * factor;
        overlay.range_limit_m = Math.max(overlay.range_min_m,
                                         Math.min(overlay.range_max_m, value));
    }

    // The centre of the camera image (falls back to the item centre before the
    // first frame has reported its size).
    readonly property real centre_x: display_w > 0
            ? display_x + display_w / 2 : width / 2
    readonly property real centre_y: display_h > 0
            ? display_y + display_h / 2 : height / 2
    readonly property real scale: (Math.min(display_w > 0 ? display_w : width,
                                            display_h > 0 ? display_h : height)
                                   / 2) / Math.max(0.001, range_limit_m)

    // Redraw cap: the overlay is larger than the old inset, so a full-rate
    // repaint of a 2000-point Canvas would be a heavy, needless CPU cost on
    // this host.  Repaint on the configured cadence while visible.
    Timer {
        id: redraw_timer
        interval: Math.max(30, 1000 / Math.max(1, layout ? layout.redrawHz : 12))
        repeat: true
        running: overlay.visible
        onTriggered: canvas.requestPaint()
    }

    // Light scrim: enough to lift the point cloud off a bright camera image
    // while keeping the image readable for aiming.  Deliberately subtle (the
    // range-coloured points are already high contrast) so the camera image the
    // operator aims with is not dimmed away.  Admin-tunable via
    // hud_config.json lidar_scan.scrim_opacity (0 disables it); this is the
    // ONLY scrim and it exists only while the overlay is visible.
    Rectangle {
        anchors.fill: parent
        color: "#05080c"
        opacity: layout ? layout.scrimOpacity : 0.15
        visible: opacity > 0
    }

    Canvas {
        id: canvas
        anchors.fill: parent
        antialiasing: false
        renderStrategy: Canvas.Cooperative

        property string activeView: bridge.lidarView
        property real activeZoom: overlay.range_limit_m
        onActiveViewChanged: requestPaint()
        onActiveZoomChanged: requestPaint()

        onPaint: {
            var ctx = getContext("2d");
            ctx.reset();
            var cx = overlay.centre_x;
            var cy = overlay.centre_y;
            var s = overlay.scale;
            var view = bridge.lidarView;

            // Points.  Small sprites (2 px) as recommended for measurement: big
            // sprites add fill cost without adding information.
            var points = bridge.lidarPoints;
            for (var i = 0; i < points.length; i++) {
                var p = points[i];
                var proj = overlay.projectPoint(p[0], p[1], p[2], view, cx, cy, s);
                var px = proj[0], py = proj[1];
                if (px < 0 || px >= width || py < 0 || py >= height) continue;
                var distance = Math.sqrt(p[0] * p[0] + p[1] * p[1] + p[2] * p[2]);
                var t = Math.min(distance / Math.max(0.001, overlay.range_limit_m), 1.0);
                var r = Math.round(40 + t * 215);
                var g2 = Math.round(180 * (1 - Math.abs(t - 0.5) * 2));
                var b = Math.round(255 * (1 - t));
                ctx.fillStyle = "rgb(" + r + "," + g2 + "," + b + ")";
                ctx.fillRect(px - 1, py - 1, 2.0, 2.0);
            }

            // Sensor origin marker.
            ctx.fillStyle = "#4fc3f7";
            ctx.fillRect(cx - 3, cy - 3, 6, 6);

            // Crosshair at the image centre: the operator centres the trunk on
            // it while scanning.
            if (bridge.scanPhase === "scanning") {
                ctx.strokeStyle = "#e8eef4";
                ctx.lineWidth = 1.2;
                ctx.beginPath();
                ctx.moveTo(cx - 22, cy); ctx.lineTo(cx - 8, cy);
                ctx.moveTo(cx + 8, cy); ctx.lineTo(cx + 22, cy);
                ctx.moveTo(cx, cy - 22); ctx.lineTo(cx, cy - 8);
                ctx.moveTo(cx, cy + 8); ctx.lineTo(cx, cy + 22);
                ctx.stroke();
            }
        }
    }

    // Projection wrapper.  Mirrors harvester_dashboard/projection.py so the
    // overlay and the portable Python projection agree: the vehicle views are
    // orthographic in the project frame (+X fwd / +Y left / +Z up), and the
    // camera view is the optical frame (+X right / +Y down / +Z forward).
    // Identity camera<->LiDAR extrinsic until the commissioning survey.
    function projectPoint(x, y, z, view, cx, cy, s) {
        if (view === "camera") {
            // Optical +X -> screen right, +Y (down) -> screen down.  +Z is depth.
            return [cx + x * s, cy + y * s];
        }
        if (view === "front") {
            // Observer in front looking toward -x: +y (left) is to the right.
            return [cx + y * s, cy - z * s];
        }
        if (view === "left") {
            // Observer on the left looking toward -y: +x (forward) to the right.
            return [cx + x * s, cy - z * s];
        }
        if (view === "right") {
            // Observer on the right looking toward +y: +x to the left.
            return [cx - x * s, cy - z * s];
        }
        if (view === "iso") {
            var iso = 0.5;
            var lift = 0.866;
            return [cx + (x - y) * iso * s,
                    cy - ((x + y) * iso * 0.5 + z * lift) * s];
        }
        // top (and any unknown view, which must never blank the cloud):
        // +x -> right, +y -> up.
        return [cx + x * s, cy - y * s];
    }

    // ---- Three short cards along the bottom ----
    //
    // One tall card blocked the trunk in the cloud, so the information is split
    // into three short cards pinned to the bottom EDGE: scan state (left), tree
    // estimate (centre), boom/dock estimate (right).  Each stays short so the
    // cloud above the bottom strip stays fully visible.
    readonly property real card_bottom_margin: layout ? layout.marginPx : 12
    readonly property real card_gap: 8
    readonly property real card_width: Math.max(
        180, (width - 2 * card_bottom_margin - 2 * card_gap) / 3)

    // (1) Scan state / guidance, bottom-left.
    Rectangle {
        id: scan_card
        anchors.left: parent.left
        anchors.bottom: parent.bottom
        anchors.leftMargin: overlay.card_bottom_margin
        anchors.bottomMargin: overlay.card_bottom_margin
        width: overlay.card_width
        height: scan_content.height + 20
        radius: 10
        color: "#0b0f14"
        opacity: layout ? layout.opacity : 0.85
        border.color: bridge.scanPhase === "complete" ? "#a8d08d"
                    : (bridge.scanPhase === "no_data" ? "#e25c5c" : "#4fc3f7")
        border.width: 2

        Column {
            id: scan_content
            anchors.left: parent.left
            anchors.right: parent.right
            anchors.top: parent.top
            anchors.margins: 12
            spacing: 6

            Text {
                width: parent.width
                text: (layout ? layout.caption : "LIDAR SCAN")
                      + "  ·  " + bridge.lidarViewLabel
                color: "#bfe3ff"
                font.pixelSize: Math.max(13, (layout ? layout.captionFontPx : 30) - 8)
                font.bold: true
                elide: Text.ElideRight
            }

            Text {
                width: parent.width
                text: bridge.scanGuideText
                color: bridge.scanPhase === "complete" ? "#a8d08d"
                     : (bridge.scanPhase === "no_data" ? "#e25c5c" : "#e8eef4")
                font.pixelSize: Math.max(13, (layout ? layout.guideFontPx : 34) - 12)
                font.bold: true
                wrapMode: Text.WordWrap
            }

            // Progress bar while scanning.
            Rectangle {
                width: parent.width
                height: 8
                radius: 4
                color: "#1c2833"
                visible: bridge.scanPhase === "scanning"
                Rectangle {
                    height: parent.height
                    radius: 4
                    color: "#4fc3f7"
                    width: parent.width * Math.max(0, Math.min(1, bridge.scanProgress))
                }
            }

            Text {
                width: parent.width
                text: bridge.scanQualityText
                color: "#6b7a8c"
                font.pixelSize: Math.max(10, (layout ? layout.captionFontPx : 30) - 16)
                elide: Text.ElideRight
            }
            Text {
                width: parent.width
                text: bridge.lidarImuAttitudeLine
                color: "#6b7a8c"
                font.pixelSize: Math.max(10, (layout ? layout.captionFontPx : 30) - 16)
                elide: Text.ElideRight
            }
            Text {
                width: parent.width
                visible: !bridge.lidarControlEnabled
                text: "LiDAR control disabled — SCAN/STOP/CANCEL do not change the sensor"
                color: "#e2a63c"
                font.pixelSize: Math.max(10, (layout ? layout.captionFontPx : 30) - 16)
                wrapMode: Text.WordWrap
            }
        }
    }

    // (2) Tree estimate, bottom-centre.  Hidden while scanning so the
    // instructions and the estimates never render together.
    Rectangle {
        id: tree_card
        anchors.horizontalCenter: parent.horizontalCenter
        anchors.bottom: parent.bottom
        anchors.bottomMargin: overlay.card_bottom_margin
        width: overlay.card_width
        height: tree_content.height + 20
        radius: 10
        color: "#0b0f14"
        opacity: layout ? layout.opacity : 0.85
        border.color: "#a8d08d"
        border.width: 2
        visible: bridge.scanPhase === "complete"

        Column {
            id: tree_content
            anchors.left: parent.left
            anchors.right: parent.right
            anchors.top: parent.top
            anchors.margins: 12
            spacing: 4

            Text {
                width: parent.width
                text: "TREE ESTIMATE"
                color: "#bfe3ff"
                font.pixelSize: Math.max(13, (layout ? layout.captionFontPx : 30) - 8)
                font.bold: true
            }
            Repeater {
                model: overlay.rowsInGroup("tree")
                delegate: Item {
                    width: tree_content.width
                    height: tree_val.height
                    property var rowData: modelData
                    Text {
                        anchors.left: parent.left
                        anchors.verticalCenter: parent.verticalCenter
                        width: parent.width * 0.58
                        text: rowData ? String(rowData.label) : ""
                        color: "#9fb4c7"
                        font.pixelSize: Math.max(12, (layout ? layout.estimateFontPx : 34) - 12)
                        elide: Text.ElideRight
                    }
                    Text {
                        id: tree_val
                        anchors.right: parent.right
                        anchors.verticalCenter: parent.verticalCenter
                        width: parent.width * 0.42
                        horizontalAlignment: Text.AlignRight
                        text: (rowData && rowData.value !== undefined)
                              ? String(rowData.value) : ""
                        color: (rowData && rowData.valid === false)
                               ? "#e25c5c" : "#e8eef4"
                        font.pixelSize: Math.max(14, (layout ? layout.valueFontPx : 46) - 14)
                        font.bold: true
                        elide: Text.ElideLeft
                    }
                }
            }
        }
    }

    // (3) Boom / dock estimate, bottom-right.
    Rectangle {
        id: boom_card
        anchors.right: parent.right
        anchors.bottom: parent.bottom
        anchors.rightMargin: overlay.card_bottom_margin
        anchors.bottomMargin: overlay.card_bottom_margin
        width: overlay.card_width
        height: boom_content.height + 20
        radius: 10
        color: "#0b0f14"
        opacity: layout ? layout.opacity : 0.85
        border.color: "#a8d08d"
        border.width: 2
        visible: bridge.scanPhase === "complete"

        Column {
            id: boom_content
            anchors.left: parent.left
            anchors.right: parent.right
            anchors.top: parent.top
            anchors.margins: 12
            spacing: 4

            Text {
                width: parent.width
                text: "BOOM / DOCK ESTIMATE"
                color: "#bfe3ff"
                font.pixelSize: Math.max(13, (layout ? layout.captionFontPx : 30) - 8)
                font.bold: true
            }
            Repeater {
                model: overlay.rowsInGroup("boom")
                delegate: Item {
                    width: boom_content.width
                    height: boom_val.height
                    property var rowData: modelData
                    Text {
                        anchors.left: parent.left
                        anchors.verticalCenter: parent.verticalCenter
                        width: parent.width * 0.58
                        text: rowData ? String(rowData.label) : ""
                        color: "#9fb4c7"
                        font.pixelSize: Math.max(12, (layout ? layout.estimateFontPx : 34) - 12)
                        elide: Text.ElideRight
                    }
                    Text {
                        id: boom_val
                        anchors.right: parent.right
                        anchors.verticalCenter: parent.verticalCenter
                        width: parent.width * 0.42
                        horizontalAlignment: Text.AlignRight
                        text: (rowData && rowData.value !== undefined)
                              ? String(rowData.value) : ""
                        color: (rowData && rowData.valid === false)
                               ? "#e25c5c" : "#e8eef4"
                        font.pixelSize: Math.max(14, (layout ? layout.valueFontPx : 46) - 14)
                        font.bold: true
                        elide: Text.ElideLeft
                    }
                }
            }
            Text {
                width: parent.width
                visible: bridge.scanStatusText.length > 0
                text: bridge.scanStatusText
                color: "#e2a63c"
                font.pixelSize: Math.max(10, (layout ? layout.captionFontPx : 30) - 16)
                wrapMode: Text.WordWrap
            }
        }
    }

    // Filter the estimate rows to one group (the rows carry a `group` key set by
    // the estimator, so the split is data-driven, not hardcoded here).
    function rowsInGroup(name) {
        var rows = bridge.scanEstimateRows;
        var out = [];
        if (!rows) return out;
        for (var i = 0; i < rows.length; i++) {
            if (rows[i] && rows[i].group === name) out.push(rows[i]);
        }
        return out;
    }

    // ---- On-screen controls (touch/mouse, render-only) ---------------------
    Row {
        anchors.left: parent.left
        anchors.top: parent.top
        anchors.margins: 12
        spacing: 8

        Repeater {
            model: [
                { label: "SCAN", action: "scan" },
                { label: "STOP", action: "stop" },
                { label: "CANCEL", action: "cancel" },
                { label: "VIEW", action: "view" },
                { label: "ZOOM +", action: "zoomin" },
                { label: "ZOOM −", action: "zoomout" }
            ]
            delegate: Rectangle {
                // STOP is only meaningful mid-scan; dim it otherwise rather
                // than hiding it (a stable button row is easier to hit).
                readonly property bool actionEnabled:
                    modelData.action !== "stop"
                    || bridge.scanPhase === "scanning"
                width: 96
                height: 44
                radius: 6
                color: ctl.pressed ? "#3a4a5a" : "#22303f"
                border.color: "#2a3a4a"
                border.width: 2
                opacity: actionEnabled ? 1.0 : 0.45
                Text {
                    anchors.centerIn: parent
                    text: modelData.label
                    color: "#e8eef4"
                    font.pixelSize: 14
                    font.bold: modelData.action === "scan"
                }
                MouseArea {
                    id: ctl
                    anchors.fill: parent
                    onClicked: {
                        if (!actionEnabled) return;
                        if (modelData.action === "scan") bridge.begin_scan();
                        else if (modelData.action === "stop") bridge.stop_scan();
                        else if (modelData.action === "cancel") bridge.cancel_scan();
                        else if (modelData.action === "view") bridge.cycle_lidar_view();
                        else if (modelData.action === "zoomin") overlay.zoomBy(0.8);
                        else if (modelData.action === "zoomout") overlay.zoomBy(1.25);
                    }
                }
            }
        }
    }

    Connections {
        target: bridge
        // Repaints are driven by the cadence timer above, not by every signal:
        // repainting on lidar_points_changed AND scan_changed AND the timer
        // produced up to three full 2000-point JS loops per update, which is
        // what made the overlay stutter.  A view/zoom change still repaints
        // immediately because the Canvas binds those directly.
        onLidar_view_changed: canvas.requestPaint()
    }

    Component.onCompleted: {
        if (layout && layout.zoomDefaultM !== undefined)
            overlay.range_limit_m = layout.zoomDefaultM;
    }
}
