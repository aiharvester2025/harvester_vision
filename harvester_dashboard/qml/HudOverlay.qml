// Sensor HUD: source badge, ranges, trunk, calibration, stream errors.
import QtQuick 2.12

Item {
    id: hud
    property real panel_opacity: 0.82

    // MIXED source warning row (top center).
    Rectangle {
        visible: bridge.sourceMixed
        anchors.top: parent.top
        anchors.horizontalCenter: parent.horizontalCenter
        anchors.topMargin: 8
        width: mixed_text.width + 24
        height: 30
        radius: 5
        color: "#b26a00"
        opacity: panel_opacity
        Text {
            id: mixed_text
            anchors.centerIn: parent
            text: "MIXED SOURCES — timestamps not comparable"
            color: "white"
            font.pixelSize: 13
        }
    }

    // Left panel: docking ranges + cutter range.
    SensorPanel {
        id: sensor_panel
        anchors.left: parent.left
        anchors.top: parent.top
        anchors.margins: 8
    }

    // Right panel: trunk + calibration (operator-facing only).
    Column {
        anchors.right: parent.right
        anchors.top: parent.top
        anchors.margins: 8
        spacing: 4

        Rectangle {
            width: trunk_cal_text.width + 16
            height: trunk_cal_text.height + 12
            radius: 5
            color: "#000000"
            opacity: panel_opacity
            Text {
                id: trunk_cal_text
                anchors.centerIn: parent
                text: bridge.trunkLine + "\n" + bridge.calibrationLine
                color: "#cfe3f5"
                font.pixelSize: 12
                horizontalAlignment: Text.AlignRight
            }
        }
    }

    // Bottom: stream errors panel (collapsible rows per channel).
    // Developer-diagnostic: gated on the 777+Enter diagnostic toggle.
    Rectangle {
        id: errors_panel
        visible: bridge.diagnosticVisible
        anchors.bottom: parent.bottom
        anchors.left: parent.left
        anchors.right: parent.right
        anchors.margins: 8
        height: Math.min(210, 24 + bridge.streamRows.length * 22)
        radius: 5
        color: "#000000"
        opacity: 0.78
        clip: true

        Text {
            anchors.top: parent.top
            anchors.left: parent.left
            anchors.margins: 6
            text: "streams  recv " + bridge.receivedPackets
                  + "  drops " + bridge.droppedPackets
            color: "#9fb4c7"
            font.pixelSize: 12
        }

        // Maintenance placeholder: only meaningful in hardware mode.
        Rectangle {
            visible: bridge.maintenanceAvailable
            anchors.top: parent.top
            anchors.right: parent.right
            anchors.margins: 6
            width: maintenance_text.width + 16
            height: 22
            radius: 4
            color: "#23415e"
            border.color: "#4fc3f7"
            Text {
                id: maintenance_text
                anchors.centerIn: parent
                text: "maintenance: hardware controls pending"
                color: "#bfe3ff"
                font.pixelSize: 11
            }
        }

        Column {
            anchors.top: parent.top
            anchors.topMargin: 24
            anchors.left: parent.left
            anchors.right: parent.right
            spacing: 0

            Repeater {
                model: bridge.streamRows
                delegate: Text {
                    anchors.left: parent.left
                    anchors.right: parent.right
                    anchors.leftMargin: 8
                    text: {
                        var base = modelData.channel + "  " + modelData.age;
                        if (modelData.gaps > 0) base += "  gaps " + modelData.gaps;
                        if (modelData.drops > 0) base += "  drops " + modelData.drops;
                        if (modelData.decode_errors > 0)
                            base += "  dec-err " + modelData.decode_errors;
                        if (modelData.error.length > 0) base += "  [" + modelData.error + "]";
                        return base;
                    }
                    color: !modelData.ever_seen ? "#7a8a99"
                         : modelData.stale ? "#e2a63c"
                         : modelData.decode_errors > 0 ? "#e25c5c"
                         : "#a8d08d"
                    font.pixelSize: 11
                    elide: Text.ElideMiddle
                }
            }
        }
    }
}
