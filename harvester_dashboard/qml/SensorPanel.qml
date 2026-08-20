// Five docking ranges + cutter range rows.
// Renders whatever telemetry_key strings arrive (Orin ingest normalizes
// the Raspberry Pi keys; the dashboard does not hard-code them).
import QtQuick 2.12

Column {
    id: panel
    spacing: 2

    Rectangle {
        width: 250
        height: docking_rows.children.length * 20 + 48
        radius: 5
        color: "#000000"
        opacity: 0.82
        clip: true

        Text {
            anchors.top: parent.top
            anchors.left: parent.left
            anchors.margins: 6
            text: "docking ranges"
            color: "#9fb4c7"
            font.pixelSize: 12
        }

        Column {
            id: docking_rows
            anchors.top: parent.top
            anchors.topMargin: 20
            anchors.left: parent.left
            anchors.margins: 4
            spacing: 0

            Repeater {
                model: bridge.dockingRangeRows
                delegate: Text {
                    text: {
                        var d = modelData.distance;
                        var distance = (d === null || d === undefined || isNaN(d))
                                 ? "INVALID" : d.toFixed(2) + " m";
                        var mark = modelData.valid ? "●" : "○";
                        return mark + " " + modelData.key + ": " + distance;
                    }
                    color: modelData.valid ? "#a8d08d" : "#e25c5c"
                    font.pixelSize: 12
                }
            }
        }

        Text {
            anchors.bottom: parent.bottom
            anchors.left: parent.left
            anchors.margins: 6
            text: bridge.cutterRangeLine
            color: "#cfe3f5"
            font.pixelSize: 12
        }
    }
}
