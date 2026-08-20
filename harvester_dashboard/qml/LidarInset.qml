// LiDAR inset: Canvas scatter of the <=2000-pt cloud, range-coloured.
// Projection view cycles via key 5: top, front, left, right, isometric.
// Point convention: x = forward, y = left, z = up (vehicle/sensor frame).
import QtQuick 2.12

Rectangle {
    id: inset
    color: "#0b0f14"
    radius: 6
    border.color: "#22303f"
    border.width: 1

    property real range_limit_m: 8.0   // half-width of the plotted window

    // Map a 3-D point to screen (sx, sy) for the current view.
    // Point convention: x = forward, y = left, z = up (vehicle/sensor frame).
    // Views are observer-centred: the viewer stands at the named side and
    // looks toward the vehicle origin.  Mirrors ``projection.py``.
    function project(x, y, z, view, cx, cy, scale) {
        var sx, sy;
        if (view === "top") {          // observer above, looking down (-z)
            sx = cx + x * scale;       // forward -> screen right
            sy = cy - y * scale;       // left    -> screen up
        } else if (view === "front") { // observer in front, looking in -x
            // When facing -x, the vehicle's +y (left) is to the viewer's right.
            sx = cx + y * scale;       // left    -> screen right
            sy = cy - z * scale;       // up      -> screen up
        } else if (view === "left") {  // observer on the left, looking in -y
            // When facing -y, the vehicle's +x (forward) is to the viewer's right.
            sx = cx + x * scale;       // forward -> screen right
            sy = cy - z * scale;       // up      -> screen up
        } else if (view === "right") { // observer on the right, looking in +y
            // When facing +y, the vehicle's +x (forward) is to the viewer's left.
            sx = cx - x * scale;       // forward -> screen left
            sy = cy - z * scale;       // up      -> screen up
        } else {                        // isometric
            var iso = 0.5;             // ~30 deg plan tilt
            var lift = 0.866;          // vertical axis shortened
            sx = cx + (x - y) * iso * scale;
            sy = cy - ((x + y) * iso * 0.5 + z * lift) * scale;
        }
        return [sx, sy];
    }

    Text {
        id: title
        anchors.top: parent.top
        anchors.left: parent.left
        anchors.margins: 6
        text: "lidar " + bridge.lidarViewLabel + "  ±" + inset.range_limit_m + " m"
        color: "#9fb4c7"
        font.pixelSize: 11
    }

    Text {
        id: axis_label
        anchors.top: title.bottom
        anchors.left: parent.left
        anchors.margins: 6
        text: ({
            "top":   "right→ +x (fwd)    up→ +y (left)",
            "front": "right→ +y (left)  up→ +z (up)",
            "left":  "right→ +x (fwd)   up→ +z (up)",
            "right": "right→ −x (aft)   up→ +z (up)",
            "iso":   "iso: +x (fwd) / +y (left) / +z (up)"
        })[bridge.lidarView] || ""
        color: "#6b7a8c"
        font.pixelSize: 10
    }

    Canvas {
        id: canvas
        anchors.top: axis_label.bottom
        anchors.left: parent.left
        anchors.right: parent.right
        anchors.bottom: parent.bottom
        anchors.margins: 6
        antialiasing: false

        // Bind the canvas to the current view so any change forces a
        // repaint.  Canvas.requestPaint() alone can be coalesced on
        // identical-sized frames and leave the old pixels in place.
        property string activeView: bridge.lidarView
        onActiveViewChanged: requestPaint()

        onPaint: {
            var ctx = getContext("2d");
            ctx.reset();
            ctx.fillStyle = "#0b0f14";
            ctx.fillRect(0, 0, width, height);

            var centre_x = width / 2;
            var centre_y = height / 2;
            var scale = Math.min(width, height) / 2 / inset.range_limit_m;
            var view = bridge.lidarView;

            // Grid + axes: rings for top-down, rectangular grid otherwise.
            ctx.strokeStyle = "#1c2833";
            if (view === "top") {
                for (var ring = 2; ring <= inset.range_limit_m; ring += 2) {
                    ctx.beginPath();
                    ctx.arc(centre_x, centre_y, ring * scale, 0, Math.PI * 2);
                    ctx.stroke();
                }
                ctx.beginPath();
                ctx.moveTo(centre_x, 0); ctx.lineTo(centre_x, height);
                ctx.moveTo(0, centre_y); ctx.lineTo(width, centre_y);
                ctx.stroke();
            } else {
                for (var g = -inset.range_limit_m; g <= inset.range_limit_m; g += 2) {
                    ctx.beginPath();
                    ctx.moveTo(centre_x + g * scale, 0);
                    ctx.lineTo(centre_x + g * scale, height);
                    ctx.moveTo(0, centre_y + g * scale);
                    ctx.lineTo(width, centre_y + g * scale);
                    ctx.stroke();
                }
            }

            var points = bridge.lidarPoints;
            for (var i = 0; i < points.length; i++) {
                var p = points[i];
                var x = p[0], y = p[1], z = p[2];
                var proj = inset.project(x, y, z, view, centre_x, centre_y, scale);
                var px = proj[0], py = proj[1];
                if (px < 0 || px >= width || py < 0 || py >= height) continue;
                var distance = Math.sqrt(x * x + y * y + z * z);
                var t = Math.min(distance / inset.range_limit_m, 1.0);
                // Cool (near) to warm (far) ramp matching the depth view.
                var r = Math.round(40 + t * 215);
                var g2 = Math.round(180 * (1 - Math.abs(t - 0.5) * 2));
                var b = Math.round(255 * (1 - t));
                ctx.fillStyle = "rgb(" + r + "," + g2 + "," + b + ")";
                ctx.fillRect(px - 1, py - 1, 2.4, 2.4);
            }

            // Vehicle marker at the origin.
            ctx.fillStyle = "#4fc3f7";
            ctx.fillRect(centre_x - 3, centre_y - 3, 6, 6);

            // Screen-axis indicator (bottom-right): an L-shape showing
            // which vehicle axis maps to which screen direction.
            var ax = width - 30;
            var ay = height - 30;
            ctx.strokeStyle = "#e8eef4";
            ctx.fillStyle = "#e8eef4";
            ctx.lineWidth = 1.5;
            ctx.beginPath();
            // right-pointing axis
            ctx.moveTo(ax, ay); ctx.lineTo(ax + 18, ay);
            // up-pointing axis
            ctx.moveTo(ax, ay); ctx.lineTo(ax, ay - 18);
            ctx.stroke();
            ctx.font = "10px sans-serif";
            ctx.textBaseline = "middle";
            var right_label, up_label;
            if (view === "top")   { right_label = "+x fwd"; up_label = "+y left"; }
            else if (view === "front") { right_label = "+y left"; up_label = "+z up"; }
            else if (view === "left")  { right_label = "+x fwd"; up_label = "+z up"; }
            else if (view === "right") { right_label = "−x aft"; up_label = "+z up"; }
            else                       { right_label = "iso";   up_label = ""; }
            ctx.textAlign = "left";
            ctx.fillText(right_label, ax + 20, ay);
            ctx.textAlign = "right";
            ctx.fillText(up_label, ax - 3, ay - 22);
        }
    }

    Connections {
        target: bridge
        onLidar_points_changed: canvas.requestPaint()
        onLidar_view_changed: canvas.requestPaint()
    }
}
