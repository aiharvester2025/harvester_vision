// Operator HUD panels: Boom (bottom-right), Docking Ranges (bottom-left) and
// Cutter Range (cutter view only).  Each panel's anchor, captions, size and
// fonts come from the admin layout in bridge.hudLayout.
//
// All panels are gated on bridge.operatorHudsVisible (key 2). The docking and
// boom panels are additionally gated on their per-panel `visible` config; the
// cutter panel only shows while the cutter camera is the active view.
//
// Each panel is hosted in a Loader (the positioned item); the anchor is applied
// to the Loader, the loaded SensorHudPanel keeps its own size.
import QtQuick 2.12

Item {
    id: root

    // Anchor a Loader per its panel layout ('bottom-left', ...).
    function applyAnchor(loader, layout) {
        loader.anchors.top = undefined;
        loader.anchors.bottom = undefined;
        loader.anchors.left = undefined;
        loader.anchors.right = undefined;
        loader.anchors.horizontalCenter = undefined;
        // Default to bottom-left if the anchor is unknown.
        var a = (layout && layout.anchor) ? layout.anchor : "bottom-left";
        if (a.indexOf("top") === 0) loader.anchors.top = root.top;
        else loader.anchors.bottom = root.bottom;
        if (a.indexOf("left") >= 0) loader.anchors.left = root.left;
        else if (a.indexOf("center") >= 0) loader.anchors.horizontalCenter = root.horizontalCenter;
        else loader.anchors.right = root.right;
        loader.anchors.margins = (layout && layout.marginPx !== undefined)
                ? layout.marginPx : 12;
    }

    // Docking range rows: map the bridge rows through the config captions.
    property var dockingRows: bridge.dockingRangeRows

    // -- Docking Ranges HUD (bottom-left by default) ----------------------
    Loader {
        id: docking_loader
        active: bridge.operatorHudsVisible && bridge.hudLayout.docking.visible
                && bridge.view !== "cutter"
        sourceComponent: SensorHudPanel {
            layout: bridge.hudLayout.docking
            rows: root.dockingRows
            subtitle: bridge.phaseGuideLine
        }
        Component.onCompleted: root.applyAnchor(docking_loader, bridge.hudLayout.docking)
        onActiveChanged: root.applyAnchor(docking_loader, bridge.hudLayout.docking)
    }

    // -- Boom HUD (bottom-right by default; docking view only) ------------
    Loader {
        id: boom_loader
        active: bridge.operatorHudsVisible && bridge.hudLayout.boom.visible
                && bridge.view !== "cutter"
        sourceComponent: SensorHudPanel {
            layout: bridge.hudLayout.boom
            rows: bridge.boomRows
        }
        Component.onCompleted: root.applyAnchor(boom_loader, bridge.hudLayout.boom)
        onActiveChanged: root.applyAnchor(boom_loader, bridge.hudLayout.boom)
    }

    // -- Cutter Range HUD (cutter view only; key 1 toggles it) ------------
    Loader {
        id: cutter_loader
        active: bridge.view === "cutter" && bridge.cutterHudVisible
                && bridge.hudLayout.cutter_range.visible
        sourceComponent: SensorHudPanel {
            layout: bridge.hudLayout.cutter_range
            rows: bridge.cutterRangeRow
        }
        Component.onCompleted: root.applyAnchor(cutter_loader, bridge.hudLayout.cutter_range)
        onActiveChanged: root.applyAnchor(cutter_loader, bridge.hudLayout.cutter_range)
    }
}
