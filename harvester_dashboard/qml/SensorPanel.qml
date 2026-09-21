// Operator HUD panels: Boom (bottom-right), Docking Ranges (bottom-left),
// Cutter Range (cutter view only) and Docking Safety Guidance (bottom-center,
// docking view only).  Each panel's anchor, captions, size and fonts come from
// the admin layout in bridge.hudLayout.
//
// The three bottom panels (docking | dock guidance | boom) are laid out
// edge-to-edge so they never overlap, regardless of screen width.
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

    // Bottom-row layout so the docking-ranges (left), dock guidance (center)
    // and boom (right) panels never overlap.  The docking and boom panels are
    // pinned to their edge with their configured width; the guidance panel
    // occupies the remaining gap (capped at its configured width and centered
    // in the gap).  Widths are read from the layout config rather than sibling
    // geometry so the math is correct even before the loaders have loaded.
    // When the gap is too narrow to render a legible panel, the guidance panel
    // is hidden rather than allowed to overlap its neighbours.
    readonly property int _minGuidanceWidth: 260

    function layoutBottomRow() {
        var margin = 12;
        var layoutCfg = bridge.hudLayout;
        var dockingVisible = docking_loader.active;
        var boomVisible = boom_loader.active;
        var guidanceVisible = dock_guidance_loader.active;

        if (!guidanceVisible) {
            // Inactive (view/flag/config): never leave a stale hidden state.
            dock_guidance_loader.visible = true;
            return;
        }

        var dockingW = dockingVisible ? Math.max(0, layoutCfg.docking.width) : 0;
        var boomW = boomVisible ? Math.max(0, layoutCfg.boom.width) : 0;
        var maxGuidanceW = (layoutCfg.dock_guidance.width > 0)
                ? layoutCfg.dock_guidance.width : 620;

        var leftEdge = dockingVisible ? (margin + dockingW + margin) : margin;
        var rightEdge = root.width - (boomVisible ? (margin + boomW + margin) : margin);
        var available = Math.max(0, rightEdge - leftEdge);

        // Hide rather than overlap when the gap cannot hold a usable panel.
        if (available < _minGuidanceWidth) {
            dock_guidance_loader.visible = false;
            return;
        }
        dock_guidance_loader.visible = true;

        var guidanceW = Math.min(maxGuidanceW, available);

        // Reset then pin the guidance panel inside the gap.
        dock_guidance_loader.anchors.left = undefined;
        dock_guidance_loader.anchors.right = undefined;
        dock_guidance_loader.anchors.horizontalCenter = undefined;
        dock_guidance_loader.width = guidanceW;
        dock_guidance_loader.anchors.bottom = root.bottom;
        dock_guidance_loader.anchors.left = root.left;
        dock_guidance_loader.anchors.leftMargin =
                leftEdge + Math.max(0, (available - guidanceW) / 2);
        dock_guidance_loader.anchors.bottomMargin = margin;
    }
    onWidthChanged: layoutBottomRow()
    onHeightChanged: layoutBottomRow()

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
        onActiveChanged: {
            root.applyAnchor(docking_loader, bridge.hudLayout.docking)
            root.layoutBottomRow()
        }
        Component.onCompleted: root.applyAnchor(docking_loader, bridge.hudLayout.docking)
        onLoaded: root.layoutBottomRow()
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
        onActiveChanged: {
            root.applyAnchor(boom_loader, bridge.hudLayout.boom)
            root.layoutBottomRow()
        }
        Component.onCompleted: root.applyAnchor(boom_loader, bridge.hudLayout.boom)
        onLoaded: root.layoutBottomRow()
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

    // -- Docking Safety-Guidance HUD (bottom-center, docking view only) ---
    // Anchor/size/caption/fonts and the stop-bar scale all come from the same
    // admin layout as the other panels (bridge.hudLayout.dock_guidance).
    Loader {
        id: dock_guidance_loader
        active: bridge.operatorHudsVisible
                && bridge.hudLayout.dock_guidance.visible
                && bridge.view !== "cutter"
        sourceComponent: DockGuidanceHud {
            layout: bridge.hudLayout.dock_guidance
            bridgeRef: bridge
        }
        onActiveChanged: {
            root.applyAnchor(dock_guidance_loader, bridge.hudLayout.dock_guidance)
            root.layoutBottomRow()
        }
        onLoaded: root.layoutBottomRow()
    }

    Component.onCompleted: layoutBottomRow()
}
