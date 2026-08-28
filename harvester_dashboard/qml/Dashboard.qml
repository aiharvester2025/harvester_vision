// Root dashboard: layout, keyboard handling, view/HUD/LiDAR visibility.
// QtQuick 2 primitives only (PySide2 5.14 on Ubuntu 20.04 has no Controls2).
import QtQuick 2.12
import QtQuick.Layouts 1.12

Item {
    id: root
    width: 1280
    height: 800

    // Keyboard: 1/2 view switch (render-only), 3 HUD, 4 LiDAR, 5 cycle LiDAR
    // view, 6 point cloud, 7 IMU stabilization, 0/Esc clear.
    focus: true
    Keys.onPressed: {
        if (event.key === Qt.Key_1) { bridge.set_view("cutter"); event.accepted = true; }
        else if (event.key === Qt.Key_2) { bridge.set_view("docking"); event.accepted = true; }
        else if (event.key === Qt.Key_3) { bridge.toggle_hud(); event.accepted = true; }
        else if (event.key === Qt.Key_4) { bridge.toggle_lidar(); event.accepted = true; }
        else if (event.key === Qt.Key_5) { bridge.cycle_lidar_view(); event.accepted = true; }
        else if (event.key === Qt.Key_6) { bridge.toggle_pointcloud(); event.accepted = true; }
        else if (event.key === Qt.Key_7) { bridge.toggle_imu(); event.accepted = true; }
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
                { label: "2 Docking", action: "docking" },
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
                color: touch.pressed ? "#3a4a5a" : "#22303f"
                border.color: {
                    if (modelData.action === "cutter") return bridge.view === "cutter" ? "#4fc3f7" : "#2a3a4a";
                    if (modelData.action === "docking") return bridge.view === "docking" ? "#4fc3f7" : "#2a3a4a";
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
                        if (modelData.action === "cutter") bridge.set_view("cutter");
                        else if (modelData.action === "docking") bridge.set_view("docking");
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

        Text {
            Layout.fillWidth: true
            text: "  " + bridge.sourceBadge + "   " + bridge.statusLine
            color: "#9fb4c7"
            font.pixelSize: 13
            elide: Text.ElideRight
        }
    }

    // Main camera area with annotation overlay.
    CameraView {
        id: camera_view
        anchors.top: toolbar.bottom
        anchors.bottom: parent.bottom
        anchors.left: parent.left
        anchors.right: bridge.lidarVisible ? lidar_inset.left : parent.right
        anchors.margins: 6
    }

    // LiDAR inset (right column, togglable with 4).
    LidarInset {
        id: lidar_inset
        visible: bridge.lidarVisible
        anchors.top: toolbar.bottom
        anchors.bottom: parent.bottom
        anchors.right: parent.right
        width: 300
        anchors.margins: 6
    }

    // HUD overlay on top of the camera view.
    HudOverlay {
        id: hud
        visible: bridge.hudVisible
        anchors.fill: camera_view
    }

    // Camera point-cloud inset (top-right of the camera view, togglable with 6).
    PointCloudInset {
        id: pointcloud_inset
        visible: bridge.pointcloudVisible && bridge.pointcloudCount > 0
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
