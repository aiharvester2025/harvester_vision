// Crosshair + camera-relative annotation label on the active camera.
import QtQuick 2.12

Item {
    id: overlay
    property real display_x: 0
    property real display_y: 0
    property real display_w: 0
    property real display_h: 0
    property int image_w: 0
    property int image_h: 0

    visible: bridge.annotationActive

    function map_u(u) {
        return image_w > 0 ? display_x + (u / image_w) * display_w : -100;
    }
    function map_v(v) {
        return image_h > 0 ? display_y + (v / image_h) * display_h : -100;
    }

    // Crosshair lines.
    Rectangle {
        x: overlay.map_u(bridge.annotationU) - 1
        y: overlay.map_v(bridge.annotationV) - 12
        width: 2
        height: 24
        color: "#ffeb3b"
    }
    Rectangle {
        x: overlay.map_u(bridge.annotationU) - 12
        y: overlay.map_v(bridge.annotationV) - 1
        width: 24
        height: 2
        color: "#ffeb3b"
    }

    // Label box anchored right of the crosshair.
    Rectangle {
        x: overlay.map_u(bridge.annotationU) + 14
        y: overlay.map_v(bridge.annotationV) - 14
        width: label_text.width + 16
        height: label_text.height + 10
        radius: 4
        color: "#000000"
        opacity: 0.7
        border.color: "#ffeb3b"
        border.width: 1
        Text {
            id: label_text
            anchors.centerIn: parent
            text: bridge.annotationCamera + "  " + bridge.annotationLabel
            color: "#ffeb3b"
            font.pixelSize: 12
        }
    }
}
