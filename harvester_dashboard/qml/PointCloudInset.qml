// Camera-relative point cloud inset: a Canvas scatter of depth-unprojected
// points in the camera optical frame (+X image-right, +Y image-down, +Z
// forward through the lens).  Derived in the dashboard from the live depth +
// RGB + camera_info streams; no new wire channel.
//
// Mirrors LidarInset.qml's Canvas/projection pattern, but with the camera
// optical-frame convention (the screen up-axis is -Y, since image Y grows
// downward).
import QtQuick 2.12

Rectangle {
    id: inset
    color: "#0b0f14"
    radius: 6
    border.color: "#22303f"
    border.width: 1

    property real range_limit_m: 6.0   // half-depth of the plotted window (Z)

    // Project a camera-frame point (x right, y down, z forward) to screen
    // (sx, sy).  Observer looks along +Z (forward); screen up = -Y.
    function project(x, y, z, cx, cy, scale) {
        // top-down (X-Z plane): forward -> up, right -> right
        var sx = cx + x * scale;
        var sy = cy - z * scale;
        return [sx, sy];
    }

    Text {
        id: title
        anchors.top: parent.top
        anchors.left: parent.left
        anchors.margins: 6
        text: "point cloud  ±" + inset.range_limit_m + " m  (" + bridge.pointcloudCount + " pts)"
        color: "#9fb4c7"
        font.pixelSize: 11
    }

    Text {
        id: imuTitle
        anchors.top: title.bottom
        anchors.left: parent.left
        anchors.margins: 6
        text: bridge.imuAttitudeLine
        color: bridge.imuActive ? (bridge.imuEnabled ? "#4fc3f7" : "#8a99a9") : "#5a6b7a"
        font.pixelSize: 10
    }

    Canvas {
        id: canvas
        anchors.top: imuTitle.bottom
        anchors.left: parent.left
        anchors.right: parent.right
        anchors.bottom: parent.bottom
        anchors.margins: 6
        antialiasing: false

        onPaint: {
            var ctx = getContext("2d");
            ctx.reset();
            ctx.fillStyle = "#0b0f14";
            ctx.fillRect(0, 0, width, height);

            var centre_x = width / 2;
            var centre_y = height / 2;
            var scale = Math.min(width, height) / 2 / inset.range_limit_m;

            // Grid (top-down: X horizontal, Z vertical).
            ctx.strokeStyle = "#1c2833";
            for (var g = -inset.range_limit_m; g <= inset.range_limit_m; g += 1) {
                ctx.beginPath();
                ctx.moveTo(centre_x + g * scale, 0);
                ctx.lineTo(centre_x + g * scale, height);
                ctx.stroke();
                ctx.beginPath();
                ctx.moveTo(0, centre_y + g * scale);
                ctx.lineTo(width, centre_y + g * scale);
                ctx.stroke();
            }

            var points = bridge.cameraPointcloud;
            for (var i = 0; i < points.length; i++) {
                var p = points[i];
                var x = p[0], y = p[1], z = p[2];
                var r = Math.round(p[3]), g2 = Math.round(p[4]), b = Math.round(p[5]);
                var proj = inset.project(x, y, z, centre_x, centre_y, scale);
                var px = proj[0], py = proj[1];
                if (px < 0 || px >= width || py < 0 || py >= height) continue;
                ctx.fillStyle = "rgb(" + r + "," + g2 + "," + b + ")";
                ctx.fillRect(px - 1, py - 1, 2.2, 2.2);
            }

            // Camera origin marker (optical center).
            ctx.fillStyle = "#4fc3f7";
            ctx.fillRect(centre_x - 3, centre_y - 3, 6, 6);

            // Screen-axis indicator: +X (image-right) and +Z (forward).
            var ax = width - 30;
            var ay = height - 30;
            ctx.strokeStyle = "#e8eef4";
            ctx.fillStyle = "#e8eef4";
            ctx.lineWidth = 1.5;
            ctx.beginPath();
            ctx.moveTo(ax, ay); ctx.lineTo(ax + 18, ay);       // right -> +X
            ctx.moveTo(ax, ay); ctx.lineTo(ax, ay - 18);       // up -> +Z (forward)
            ctx.stroke();
            ctx.font = "10px sans-serif";
            ctx.textBaseline = "middle";
            ctx.textAlign = "left";
            ctx.fillText("+X", ax + 20, ay);
            ctx.textAlign = "right";
            ctx.fillText("+Z fwd", ax - 3, ay - 22);
        }
    }

    Connections {
        target: bridge
        onPointcloud_changed: canvas.requestPaint()
    }
}
