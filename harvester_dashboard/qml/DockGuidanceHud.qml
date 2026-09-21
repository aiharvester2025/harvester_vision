// Docking safety-guidance HUD: state banner, stop-bar and metrics.
//
// The panel's anchor, caption, size, fonts, opacity and stop-bar scale come
// from the SAME admin config as the other sensor HUDs
// (bridge.hudLayout.dock_guidance, loaded from hud_config.json via
// --hud-config).  QtQuick 2 primitives only (PySide2 5.14, no Controls2).
import QtQuick 2.12

Rectangle {
    id: panel

    // Config (bridge.hudLayout.dock_guidance).
    property var layout
    // Injected by the host so the pulse animation can be enabled/disabled.
    property var bridgeRef: bridge

    readonly property int valueFontPx: layout ? layout.valueFontPx : 40
    readonly property int captionFontPx: layout ? layout.captionFontPx : 26
    readonly property int bannerFontPx: layout ? layout.bannerFontPx : 38
    readonly property int panelWidth: layout ? layout.width : 620
    readonly property int panelHeight: layout ? layout.height : 0
    readonly property real stopbarRangeM: (layout && layout.stopbarRangeM > 0)
                                          ? layout.stopbarRangeM : 1.5
    readonly property bool pulseDanger: layout ? layout.pulseDanger : true

    readonly property string state: bridgeRef ? bridgeRef.dockSafetyState : "no_data"

    // State -> colour map (must match the reference HUD).
    readonly property color stateColor: {
        if (state === "danger") return "#e23c3c";
        if (state === "warn") return "#f0a030";
        if (state === "no_data") return "#9fb4c7";
        return "#40c040";
    }
    readonly property color stateBackground: {
        if (state === "danger") return "#3a1414";
        if (state === "warn") return "#3a2a10";
        if (state === "no_data") return "#1a1f24";
        return "#102a18";
    }

    // Numeric bridge values (NaN when absent).
    readonly property real gapM: bridgeRef ? bridgeRef.dockCenterDistanceM : NaN
    readonly property real stopDistanceM: bridgeRef ? bridgeRef.dockStopDistanceM : NaN
    readonly property real ttcS: bridgeRef ? bridgeRef.dockTtcS : NaN
    readonly property real recSpeedCmS: bridgeRef ? bridgeRef.dockRecommendedSpeedCmS : NaN

    readonly property bool barRed: isFinite(stopDistanceM) && isFinite(gapM)
                                   && stopDistanceM >= gapM

    // The host (SensorPanel) sizes the loader to the gap between the docking
    // and boom panels so the bottom row cannot overlap; fill that width.
    width: parent ? parent.width : panelWidth
    height: panelHeight > 0 ? panelHeight : content.height + 28
    radius: 8
    color: stateBackground
    opacity: layout ? layout.opacity : 0.85
    border.color: stateColor
    border.width: state === "danger" ? 3 : 2
    clip: true

    // Pulse the panel border while DANGER.
    SequentialAnimation on border.color {
        running: panel.pulseDanger && panel.state === "danger"
        loops: Animation.Infinite
        ColorAnimation { to: "#7a1f1f"; duration: 420; easing.type: Easing.InOutSine }
        ColorAnimation { to: "#e23c3c"; duration: 420; easing.type: Easing.InOutSine }
    }

    Column {
        id: content
        anchors.left: parent.left
        anchors.right: parent.right
        anchors.top: parent.top
        anchors.margins: 12
        spacing: 8

        // Panel caption.
        Text {
            width: parent.width
            text: panel.layout ? panel.layout.caption : ""
            color: "#bfe3ff"
            font.pixelSize: panel.captionFontPx
            font.bold: true
            elide: Text.ElideRight
        }

        // State banner (colour-coded, one-line action).
        Text {
            width: parent.width
            text: panel.bridgeRef ? panel.bridgeRef.dockGuidanceText : ""
            color: panel.stateColor
            font.pixelSize: panel.bannerFontPx
            font.bold: true
            elide: Text.ElideRight
        }

        // Stop-bar: current gap (fill) vs required stopping distance (marker).
        Item {
            id: stopbar
            width: parent.width
            height: 22

            Rectangle {
                id: track
                anchors.left: parent.left
                anchors.right: parent.right
                anchors.verticalCenter: parent.verticalCenter
                height: 14
                radius: 4
                color: "#2a3a4a"

                // Fill = current gap, red when it cannot stop in time.
                Rectangle {
                    height: parent.height
                    radius: 4
                    width: {
                        if (!isFinite(panel.gapM)) return 0;
                        var fraction = panel.gapM / panel.stopbarRangeM;
                        fraction = Math.max(0, Math.min(1, fraction));
                        return track.width * fraction;
                    }
                    color: panel.barRed ? "#e23c3c" : panel.stateColor
                }

                // Marker = required stopping distance d_stop(v).
                Rectangle {
                    visible: isFinite(panel.stopDistanceM)
                    width: 4
                    height: parent.height + 6
                    anchors.verticalCenter: parent.verticalCenter
                    x: {
                        if (!isFinite(panel.stopDistanceM)) return 0;
                        var fraction = panel.stopDistanceM / panel.stopbarRangeM;
                        fraction = Math.max(0, Math.min(1, fraction));
                        return Math.max(0, Math.min(
                            track.width - width, track.width * fraction - width / 2));
                    }
                    color: "white"
                    radius: 1
                }
            }
        }

        // Metrics row: speed · gap · TTC · max safe · recommended.  The metric
        // captions and formatting come from the bridge's ``dockSafetyRow``
        // (the single source of truth), so the panel never re-implements the
        // number formatting.  ``recommended`` is rendered here because it is
        // only advisory in WARN/DANGER states.
        Text {
            width: parent.width
            text: {
                var rows = panel.bridgeRef ? panel.bridgeRef.dockSafetyRow : []
                var parts = [];
                for (var i = 0; i < rows.length; ++i) {
                    var row = rows[i];
                    if (row.key === "state") continue;
                    parts.push(row.label + " " + row.value);
                }
                if (isFinite(panel.recSpeedCmS)) {
                    parts.push("Rec " + panel.recSpeedCmS.toFixed(0) + " cm/s");
                }
                return parts.join(" · ");
            }
            color: "#cfe3f5"
            font.pixelSize: panel.captionFontPx
            elide: Text.ElideRight
        }
    }
}
