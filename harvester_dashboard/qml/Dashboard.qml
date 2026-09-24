// Root dashboard: layout, keyboard handling, view/HUD/LiDAR visibility.
// QtQuick 2 primitives only (PySide2 5.14 on Ubuntu 20.04 has no Controls2).
import QtQuick 2.12
import QtQuick.Layouts 1.12

Item {
    id: root
    width: 1280
    height: 800

    // Multi-key sequence buffer for the developer-diagnostic toggle (777 + Enter).
    // Only "7" is buffered (it is ambiguous with the single-key IMU toggle); all
    // other keys act immediately.  A lone "7" still toggles IMU after the 800 ms
    // flush timer empties the buffer.
    property string key_buffer: ""

    Timer {
        id: key_buffer_timer
        interval: 800
        repeat: false
        onTriggered: {
            // Flush: a lone "7" (not committed with Enter) toggles IMU.
            if (root.key_buffer === "7") bridge.toggle_imu();
            root.key_buffer = "";
        }
    }

    // Keyboard: 1/2 view switch (render-only), 3 HUD, 4 LiDAR, 5 cycle LiDAR
    // view, 6 point cloud, 7 IMU stabilization, 0/Esc clear, 777+Enter toggles
    // the developer-diagnostic HUD.
    focus: true
    Keys.onPressed: {
        // Buffer "7" presses; commit "777" on Enter.
        if (event.key === Qt.Key_7) {
            root.key_buffer += "7";
            if (root.key_buffer.length > 8) root.key_buffer = root.key_buffer.slice(-8);
            key_buffer_timer.restart();
            event.accepted = true;
            return;
        }
        if (event.key === Qt.Key_Enter || event.key === Qt.Key_Return) {
            if (root.key_buffer === "777") { bridge.toggle_diagnostic(); }
            root.key_buffer = "";
            key_buffer_timer.stop();
            event.accepted = true;
            return;
        }
        // Any other key flushes the pending "7" buffer, then falls through.
        if (root.key_buffer !== "") {
            root.key_buffer = "";
            key_buffer_timer.stop();
        }

        if (event.key === Qt.Key_1) { bridge.select_cutter_view(); event.accepted = true; }
        else if (event.key === Qt.Key_2) { bridge.toggle_operator_huds(); event.accepted = true; }
        else if (event.key === Qt.Key_3) { bridge.toggle_hud(); event.accepted = true; }
        else if (event.key === Qt.Key_4) { bridge.toggle_lidar(); event.accepted = true; }
        else if (event.key === Qt.Key_5) { bridge.cycle_lidar_view(); event.accepted = true; }
        else if (event.key === Qt.Key_6) { bridge.toggle_pointcloud(); event.accepted = true; }
        else if (event.key === Qt.Key_0 || event.key === Qt.Key_Escape) {
            bridge.clear_annotation(); event.accepted = true;
        }
    }

    // Touch/mouse equivalents of 1/2/3/4/0 built from primitives.
    RowLayout {
        id: toolbar
        anchors.top: parent.top
        anchors.left: parent.left
        anchors.right: parent.right
        height: 48
        spacing: 6

        Repeater {
            model: [
                { label: "1 Cutter", action: "cutter" },
                { label: "2 Boom/Dock", action: "boomdock" },
                { label: "3 HUD", action: "hud" },
                { label: "4 LiDAR", action: "lidar" },
                { label: "5 View", action: "lidarview" },
                { label: "6 Pts", action: "pointcloud" },
                { label: "7 IMU", action: "imu" },
                { label: "0 Clear", action: "clear" }
            ]
            delegate: Rectangle {
                Layout.preferredWidth: modelData.action === "pointcloud" ? 64 : 96
                Layout.fillHeight: true
                radius: 6
                // Stable id for GUI tests to locate each toolbar button.
                objectName: "toolbar_" + modelData.action
                color: touch.pressed ? "#3a4a5a" : "#22303f"
                border.color: {
                    if (modelData.action === "cutter") return bridge.view === "cutter" ? "#4fc3f7" : "#2a3a4a";
                    if (modelData.action === "boomdock") return (bridge.operatorHudsVisible && bridge.view !== "cutter") ? "#4fc3f7" : "#2a3a4a";
                    if (modelData.action === "lidar") return bridge.lidarVisible ? "#4fc3f7" : "#2a3a4a";
                    if (modelData.action === "pointcloud") return bridge.pointcloudVisible ? "#4fc3f7" : "#2a3a4a";
                    if (modelData.action === "imu") return bridge.imuEnabled ? "#4fc3f7" : "#2a3a4a";
                    if (modelData.action === "lidarview") return "#2a3a4a";
                    return "#2a3a4a";
                }
                border.width: 2

                Text {
                    anchors.centerIn: parent
                    text: modelData.label
                    color: "#e8eef4"
                    font.pixelSize: 13
                }
                MouseArea {
                    id: touch
                    anchors.fill: parent
                    onClicked: {
                        if (modelData.action === "cutter") bridge.select_cutter_view();
                        else if (modelData.action === "boomdock") bridge.toggle_operator_huds();
                        else if (modelData.action === "hud") bridge.toggle_hud();
                        else if (modelData.action === "lidar") bridge.toggle_lidar();
                        else if (modelData.action === "lidarview") bridge.cycle_lidar_view();
                        else if (modelData.action === "pointcloud") bridge.toggle_pointcloud();
                        else if (modelData.action === "imu") bridge.toggle_imu();
                        else if (modelData.action === "clear") bridge.clear_annotation();
                    }
                }
            }
        }

        // Source badge (operator-facing) and status line (developer diagnostic,
        // hidden when the diagnostic HUD is toggled off).
        Text {
            Layout.fillWidth: true
            text: "  " + bridge.sourceBadge
                  + (bridge.diagnosticVisible ? "   " + bridge.statusLine : "")
            color: "#9fb4c7"
            font.pixelSize: 13
            elide: Text.ElideRight
        }
    }

    // Main camera area with annotation overlay.  The camera stays full-frame;
    // the LiDAR scan overlay paints over it (so the cloud can be centred on the
    // camera image), rather than shrinking the camera into a side column.
    CameraView {
        id: camera_view
        anchors.top: toolbar.bottom
        anchors.bottom: parent.bottom
        anchors.left: parent.left
        anchors.right: parent.right
        anchors.margins: 6
    }

    // Full-screen LiDAR scan overlay (key 4 toggles it; hidden by default).
    // Created BEFORE HudOverlay so the safety panels always paint on top of the
    // cloud: this is a background layer, not a replacement for the HUD.
    LidarScanOverlay {
        id: lidar_scan_overlay
        visible: bridge.lidarVisible
        anchors.fill: camera_view
        display_x: camera_view.display_x
        display_y: camera_view.display_y
        display_w: camera_view.display_w
        display_h: camera_view.display_h
    }

    // HUD overlay on top of the camera view.  Hidden while the full-screen
    // LiDAR scan overlay is up so the scan view is not cluttered; the safety
    // panels return as soon as the overlay is hidden (key 4).
    HudOverlay {
        id: hud
        visible: bridge.hudVisible && !bridge.lidarScanActive
        anchors.fill: camera_view
    }

    // Camera point-cloud inset (top-right of the camera view, togglable with 6).
    // Also hidden in LiDAR scan mode (it is the depth-cloud view, unrelated to
    // the LiDAR scan).
    PointCloudInset {
        id: pointcloud_inset
        visible: bridge.pointcloudVisible && bridge.pointcloudCount > 0
                 && !bridge.lidarScanActive
        anchors.top: camera_view.top
        anchors.right: camera_view.right
        anchors.margins: 6
        width: 260
        height: 260
    }

    // Transient toast (annotation feedback, maintenance notices).
    Rectangle {
        visible: bridge.toast.length > 0
        anchors.horizontalCenter: parent.horizontalCenter
        anchors.bottom: parent.bottom
        anchors.bottomMargin: 18
        width: Math.min(root.width - 40, toast_text.width + 32)
        height: 40
        radius: 8
        color: "#e23c3c"
        opacity: 0.92
        Text {
            id: toast_text
            anchors.centerIn: parent
            text: bridge.toast
            color: "white"
            font.pixelSize: 14
        }
    }
}
