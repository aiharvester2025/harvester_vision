// Cutter safety-guide HUD (cutter camera view): cut-step banner, clearance
// alert, clearance bar, metrics and the CONFIRM STEP prompt.
//
// The panel's anchor, caption, size, fonts, opacity, bar scale and the
// CONFIRM-button visibility all come from the SAME admin config as the other
// sensor HUDs (bridge.hudLayout.cutter_guidance, loaded from hud_config.json
// via --hud-config).  QtQuick 2 primitives only (PySide2 5.14, no Controls2).
//
// Advisory only: the CONFIRM STEP button advances the operator *prompt*; it
// never commands the arm.  It is disabled (relabelled "WAIT — tip too close")
// while the clearance state is DANGER/NO_DATA.
import QtQuick 2.12

Rectangle {
    id: panel

    // Config (bridge.hudLayout.cutter_guidance).
    property var layout
    // Injected by the host so the pulse animation can be enabled/disabled.
    property var bridgeRef: bridge

    readonly property int valueFontPx: layout ? layout.valueFontPx : 40
    readonly property int captionFontPx: layout ? layout.captionFontPx : 26
    readonly property int bannerFontPx: layout ? layout.bannerFontPx : 38
    readonly property int panelWidth: layout ? layout.width : 620
    readonly property int panelHeight: layout ? layout.height : 0
    readonly property real stopbarRangeM: (layout && layout.stopbarRangeM > 0)
                                          ? layout.stopbarRangeM : 1.0
    readonly property bool pulseDanger: layout ? layout.pulseDanger : true
    readonly property bool showConfirm: layout ? layout.showConfirmButton : true

    readonly property string state: bridgeRef ? bridgeRef.cutterSafetyState : "no_data"
    readonly property string phase: bridgeRef ? bridgeRef.cutterPhase : "idle"

    // State -> colour map (must match the docking HUD).
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
    readonly property real clearanceM: bridgeRef ? bridgeRef.cutterClearanceM : NaN
    readonly property real stopDistanceM: bridgeRef ? bridgeRef.cutterStopDistanceM : NaN
    readonly property real ttcS: bridgeRef ? bridgeRef.cutterTtcS : NaN
    readonly property bool barRed: isFinite(stopDistanceM) && isFinite(clearanceM)
                                   && stopDistanceM >= clearanceM

    // The CONFIRM STEP button only appears once the measured approach has
    // reached the ready standoff (i.e. not idle/approach).
    readonly property bool confirmVisible: showConfirm
                                           && (phase === "align" || phase === "open"
                                               || phase === "advance" || phase === "cut")
    readonly property bool confirmEnabled: bridgeRef ? bridgeRef.cutterCanConfirm : false

    // The host (SensorPanel) sizes the loader to the gap between the cutter
    // range and any other bottom panel so the row cannot overlap; fill it.
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

        // Cut-step banner: the operator prompt for the current phase.
        Text {
            width: parent.width
            text: panel.bridgeRef ? panel.bridgeRef.cutterPhaseText : ""
            color: "#e8eef4"
            font.pixelSize: panel.bannerFontPx
            font.bold: true
            elide: Text.ElideRight
        }

        // Clearance alert line, coloured by state.
        Text {
            width: parent.width
            text: panel.bridgeRef ? panel.bridgeRef.cutterGuidanceText : ""
            color: panel.stateColor
            font.pixelSize: panel.captionFontPx
            font.bold: true
            elide: Text.ElideRight
        }

        // Clearance bar: tip clearance (fill) vs required stopping distance.
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

                // Fill = current tip clearance, red when it cannot stop in time.
                Rectangle {
                    height: parent.height
                    radius: 4
                    width: {
                        if (!isFinite(panel.clearanceM)) return 0;
                        var fraction = panel.clearanceM / panel.stopbarRangeM;
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

        // Metrics row: clearance · speed · max safe.  Captions/formatting come
        // from the bridge's ``cutterSafetyRow`` (single source of truth).
        Text {
            width: parent.width
            text: {
                var rows = panel.bridgeRef ? panel.bridgeRef.cutterSafetyRow : []
                var parts = [];
                for (var i = 0; i < rows.length; ++i) {
                    var row = rows[i];
                    if (row.key === "state" || row.key === "phase") continue;
                    parts.push(row.label + " " + row.value);
                }
                return parts.join(" · ");
            }
            color: "#cfe3f5"
            font.pixelSize: panel.captionFontPx
            elide: Text.ElideRight
        }

        // CONFIRM STEP prompt: advances the operator-confirmed cut sequence
        // (open -> advance -> cut).  Disabled while DANGER/NO_DATA, with a
        // reason that matches the actual state (never "tip too close" on a
        // missing range).
        Rectangle {
            id: confirmButton
            visible: panel.confirmVisible
            width: Math.max(180, confirmText.width + 32)
            height: 34
            radius: 6
            color: panel.confirmEnabled ? "#23415e" : "#2b2f33"
            border.color: panel.confirmEnabled ? "#4fc3f7" : "#5a6570"
            border.width: 2

            Text {
                id: confirmText
                anchors.centerIn: parent
                text: {
                    if (panel.confirmEnabled) return "CONFIRM STEP  ▸";
                    if (panel.state === "no_data") return "WAIT — no range";
                    return "WAIT — tip too close";
                }
                color: panel.confirmEnabled ? "#bfe3ff" : "#8b969f"
                font.pixelSize: 16
                font.bold: true
            }

            MouseArea {
                anchors.fill: parent
                enabled: panel.confirmEnabled
                onClicked: {
                    // Prompt-only: never commands the arm.
                    if (panel.bridgeRef) panel.bridgeRef.cutter_confirm_phase();
                }
            }
        }
    }
}
