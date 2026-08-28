// Camera image with click annotation overlay.
import QtQuick 2.12

Item {
    id: view

    // Image counter makes the provider URL change per frame.
    property string camera_name: bridge.view
    property string image_url: "image://frames/" + camera_name + "?n=" + bridge.frameCounter

    Rectangle {
        anchors.fill: parent
        color: "#101418"
    }

    Image {
        id: frame
        anchors.fill: parent
        source: view.image_url
        fillMode: Image.PreserveAspectFit
        asynchronous: false
        cache: false
        smooth: true
    }

    // The image is letterboxed; compute the displayed rect so the click
    // mapping and the crosshair stay inside the visible pixels.
    property real display_x: 0
    property real display_y: 0
    property real display_w: 0
    property real display_h: 0

    onImage_urlChanged: {
        var src_w = frame.sourceSize.width;
        var src_h = frame.sourceSize.height;
        if (src_w <= 0 || src_h <= 0) return;
        var scale = Math.min(view.width / src_w, view.height / src_h);
        view.display_w = src_w * scale;
        view.display_h = src_h * scale;
        view.display_x = (view.width - view.display_w) / 2;
        view.display_y = (view.height - view.display_h) / 2;
    }

    // Stale indicator ring around the whole view (developer diagnostic).
    Rectangle {
        visible: bridge.activeCameraStale && bridge.diagnosticVisible
        anchors.fill: parent
        color: "transparent"
        border.color: "#e23c3c"
        border.width: 4
        opacity: 0.85
    }

    MouseArea {
        anchors.fill: parent
        onClicked: {
            // Map view coordinates to source pixel coordinates.
            if (view.display_w <= 0 || view.display_h <= 0) return;
            var rel_x = (mouse.x - view.display_x) / view.display_w;
            var rel_y = (mouse.y - view.display_y) / view.display_h;
            if (rel_x < 0 || rel_x > 1 || rel_y < 0 || rel_y > 1) return;
            var u = Math.round(rel_x * frame.sourceSize.width);
            var v = Math.round(rel_y * frame.sourceSize.height);
            bridge.annotate_click(u, v);
        }
    }

    // Active annotation crosshair + label.
    Annotation {
        anchors.fill: parent
        display_x: view.display_x
        display_y: view.display_y
        display_w: view.display_w
        display_h: view.display_h
        image_w: frame.sourceSize.width
        image_h: frame.sourceSize.height
    }

    Text {
        visible: bridge.diagnosticVisible
        anchors.bottom: parent.bottom
        anchors.left: parent.left
        anchors.margins: 8
        text: bridge.activeTimestampLine
        color: "#9fb4c7"
        font.pixelSize: 12
    }
}
