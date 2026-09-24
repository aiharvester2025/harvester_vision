// Operator HUD panels: Boom (bottom-right), Docking Ranges (bottom-left),
// Cutter Range (cutter view only), Docking Safety Guidance (bottom-center,
// docking view only) and Cutter Safety Guidance (bottom-center, cutter view
// only).  Each panel's anchor, captions, size and fonts come from the admin
// layout in bridge.hudLayout.
//
// On the docking view the bottom row (docking | dock guidance | boom) is laid
// out edge-to-edge; on the cutter view the bottom row (cutter range | cutter
// guidance) is laid out the same way, so panels never overlap regardless of
// screen width.
//
// All docking-view panels are gated on bridge.operatorHudsVisible (key 2); the
// cutter-view panels are gated on the cutter view (key 1).  Each is also gated
// on its per-panel `visible` config.
//
// Each panel is hosted in a Loader (the positioned item); the anchor is applied
// to the Loader, the loaded panel keeps its own size.
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

    // Width of the free band [left, right] (used to pick the wider side band
    // around a centered Cutter Range panel).
    function availableFrom(left, right) {
        return Math.max(0, right - left);
    }

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
    onWidthChanged: {
        layoutBottomRow();
        layoutCutterRow();
    }
    onHeightChanged: {
        layoutBottomRow();
        layoutCutterRow();
    }

    // Cutter-view bottom row: the Cutter Range panel and the Cutter
    // Safety-Guide panel.  Each keeps its configured anchor/width; the guidance
    // panel is placed per its own configured anchor and shrunk to fit the free
    // space beside the range panel, so the two never overlap.  The range panel
    // may sit on either edge, so the free band is reserved on whichever side it
    // occupies (this mirrors layoutBottomRow, which reserves both neighbours).
    // A `center` anchor centers the panel on the *screen* (not merely in the
    // leftover gap, which would sit visibly off-center).  Hidden when the free
    // band cannot hold a legible panel.
    function layoutCutterRow() {
        var margin = 12;
        var layoutCfg = bridge.hudLayout;
        var rangeVisible = cutter_loader.active;
        var guidanceVisible = cutter_guidance_loader.active;

        if (!guidanceVisible) {
            cutter_guidance_loader.visible = true;
            return;
        }

        // Reserve the band the Cutter Range panel occupies, on the side its
        // anchor pins it to.  A centered range panel cannot be reserved
        // numerically, so in that case the guidance panel must not be centered
        // too -- hide it rather than risk painting over the range panel.
        var rangeAnchor = (layoutCfg.cutter_range
                           && layoutCfg.cutter_range.anchor)
                ? layoutCfg.cutter_range.anchor : "bottom-left";
        var rangeW = rangeVisible
                ? Math.max(0, layoutCfg.cutter_range.width) : 0;
        var rangeOnLeft = rangeAnchor.indexOf("left") >= 0;
        var rangeOnRight = rangeAnchor.indexOf("right") >= 0;
        var rangeCentered = !rangeOnLeft && !rangeOnRight;

        var ga = (layoutCfg.cutter_guidance.anchor
                  && layoutCfg.cutter_guidance.anchor.length > 0)
                ? layoutCfg.cutter_guidance.anchor : "bottom-center";
        var guideCentered = ga.indexOf("center") >= 0;

        // A centered range panel occupies the middle band and cannot be
        // reserved numerically on either side.  It only collides with the
        // guidance panel when that panel is also centered (a left/right-anchored
        // guidance panel can still sit in the free edge band); hide rather than
        // paint over the range panel in that case.
        if (rangeCentered && rangeW > 0 && guideCentered) {
            cutter_guidance_loader.visible = false;
            return;
        }

        // The free horizontal band the guidance panel may occupy.  For a
        // centered range panel the reserved band is the middle region, so the
        // guidance panel is confined to one of the two side bands (either is
        // disjoint from the centered range panel).
        var freeLeft = rangeOnLeft ? (margin + rangeW + margin) : margin;
        var freeRight = (rangeOnRight
                         ? (root.width - margin - rangeW - margin)
                         : (root.width - margin));
        if (rangeCentered && rangeW > 0) {
            // Split the free space around the centered range panel.
            var midLeft = (root.width - rangeW) / 2 - margin;
            var midRight = (root.width + rangeW) / 2 + margin;
            if (availableFrom(freeLeft, midLeft)
                    >= availableFrom(midRight, freeRight)) {
                freeRight = midLeft;
            } else {
                freeLeft = midRight;
            }
        }
        var available = Math.max(0, freeRight - freeLeft);
        if (available < _minGuidanceWidth) {
            cutter_guidance_loader.visible = false;
            return;
        }
        cutter_guidance_loader.visible = true;

        var maxGuidanceW = (layoutCfg.cutter_guidance.width > 0)
                ? layoutCfg.cutter_guidance.width : 620;

        // Largest width for which a SCREEN-CENTERED panel clears the range
        // panel.  A centered panel of width W spans [(width-W)/2, (width+W)/2];
        // it must stay inside [freeLeft, freeRight], giving
        // W <= width - 2*freeLeft (left reservation) and
        // W <= 2*freeRight - width (right reservation).  Only a `center` anchor
        // uses this: a left/right-anchored panel is placed directly inside the
        // free band and needs only `available`.  (A centered range panel cannot
        // coexist with a centered guidance panel -- handled above.)
        var centerLimit = root.width;
        if (rangeOnLeft && rangeW > 0) {
            centerLimit = root.width - 2 * freeLeft;
        }
        if (rangeOnRight && rangeW > 0) {
            centerLimit = Math.min(centerLimit, 2 * freeRight - root.width);
        }
        var symmetricLimit = Math.max(0, centerLimit);
        var widthLimit = (guideCentered && !rangeCentered)
                ? symmetricLimit : available;
        var guidanceW = Math.min(maxGuidanceW, available, widthLimit);
        if (guidanceW < _minGuidanceWidth) {
            // Too narrow to be legible even when shrunk: hide, never overlap.
            cutter_guidance_loader.visible = false;
            return;
        }

        var onTop = ga.indexOf("top") === 0;

        // Compute the left edge for the configured anchor.  For `center` this
        // is the SCREEN centre (clamped into the free band), so the panel is
        // visually centered rather than centered in the leftover gap.
        var left;
        if (ga.indexOf("left") >= 0) {
            left = freeLeft;
        } else if (guideCentered) {
            left = (root.width - guidanceW) / 2;
            left = Math.max(freeLeft, Math.min(left, freeRight - guidanceW));
        } else {
            left = freeRight - guidanceW;
        }

        cutter_guidance_loader.anchors.left = undefined;
        cutter_guidance_loader.anchors.right = undefined;
        cutter_guidance_loader.anchors.horizontalCenter = undefined;
        cutter_guidance_loader.anchors.top = undefined;
        cutter_guidance_loader.anchors.bottom = undefined;
        cutter_guidance_loader.width = guidanceW;
        if (onTop) {
            cutter_guidance_loader.anchors.top = root.top;
            cutter_guidance_loader.anchors.topMargin = margin;
        } else {
            cutter_guidance_loader.anchors.bottom = root.bottom;
            cutter_guidance_loader.anchors.bottomMargin = margin;
        }
        cutter_guidance_loader.anchors.left = root.left;
        cutter_guidance_loader.anchors.leftMargin = left;
    }
    // Docking range rows: map the bridge rows through the config captions.
    property var dockingRows: bridge.dockingRangeRows

    // -- Docking Ranges HUD (bottom-left by default) ----------------------
    Loader {
        id: docking_loader
        active: bridge.operatorHudsVisible && bridge.hudLayout.docking.visible
                && bridge.view !== "cutter" && !bridge.lidarScanActive
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
                && bridge.view !== "cutter" && !bridge.lidarScanActive
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
                && !bridge.lidarScanActive
        sourceComponent: SensorHudPanel {
            layout: bridge.hudLayout.cutter_range
            rows: bridge.cutterRangeRow
        }
        Component.onCompleted: root.applyAnchor(cutter_loader, bridge.hudLayout.cutter_range)
        onActiveChanged: {
            root.applyAnchor(cutter_loader, bridge.hudLayout.cutter_range)
            root.layoutCutterRow()
        }
        onLoaded: root.layoutCutterRow()
    }

    // -- Cutter Safety-Guide HUD (bottom-center, cutter view only) --------
    // Anchor/size/caption/fonts, the clearance-bar scale and the CONFIRM STEP
    // visibility all come from the same admin layout as the other panels
    // (bridge.hudLayout.cutter_guidance).  Gated on the same key-1 cutter HUD
    // flag as the Cutter Range panel, so key 1 hides/shows both cutter-view
    // HUDs together.  Advisory only: the panel's confirm button advances an
    // operator prompt and never commands the arm.
    Loader {
        id: cutter_guidance_loader
        active: bridge.view === "cutter" && bridge.cutterHudVisible
                && bridge.hudLayout.cutter_guidance.visible
                && !bridge.lidarScanActive
        sourceComponent: CutterGuidanceHud {
            layout: bridge.hudLayout.cutter_guidance
            bridgeRef: bridge
        }
        onActiveChanged: {
            root.applyAnchor(cutter_guidance_loader,
                             bridge.hudLayout.cutter_guidance)
            root.layoutCutterRow()
        }
        onLoaded: root.layoutCutterRow()
    }

    // -- Docking Safety-Guidance HUD (bottom-center, docking view only) ---
    // Anchor/size/caption/fonts and the stop-bar scale all come from the same
    // admin layout as the other panels (bridge.hudLayout.dock_guidance).
    Loader {
        id: dock_guidance_loader
        active: bridge.operatorHudsVisible
                && bridge.hudLayout.dock_guidance.visible
                && bridge.view !== "cutter" && !bridge.lidarScanActive
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

    Component.onCompleted: {
        layoutBottomRow()
        layoutCutterRow()
    }
}