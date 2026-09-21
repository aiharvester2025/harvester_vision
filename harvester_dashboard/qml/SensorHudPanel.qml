// Generic, config-driven HUD panel: a title caption plus caption/value rows.
// The admin layout (bridge.hudLayout) supplies anchor, captions, panel size,
// value/caption font sizes, margin and opacity; this component only renders.
import QtQuick 2.12

Rectangle {
    id: panel

    // Config (from bridge.hudLayout.<name>).
    property var layout
    // Rows: [{ key, label, value, valid }]. The display label falls back to the
    // per-row config name (layout.rows[key]) when the row carries none.
    property var rows: []
    // Optional subtitle line (e.g. the docking phase guide).
    property string subtitle: ""

    readonly property int valueFontPx: layout ? layout.valueFontPx : 40
    readonly property int captionFontPx: layout ? layout.captionFontPx : 26
    readonly property int panelWidth: layout ? layout.width : 520

    width: panelWidth
    height: content.height + 28
    radius: 8
    color: "#000000"
    opacity: layout ? layout.opacity : 0.85
    border.color: "#4fc3f7"
    border.width: 2
    clip: true

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

        // Optional subtitle (phase guide).
        Text {
            width: parent.width
            visible: panel.subtitle.length > 0
            text: panel.subtitle
            color: "#a8d08d"
            font.pixelSize: panel.captionFontPx
            font.bold: true
            elide: Text.ElideRight
        }

        // One row per sensor value.  The model is the array length with the
        // delegate indexing into `panel.rows`; binding a Repeater directly to a
        // `var` array is unreliable in this PySide2/Qt build (it renders no
        // delegates), so we index explicitly.
        Repeater {
            model: panel.rows ? panel.rows.length : 0
            delegate: Item {
                width: content.width
                height: valueText.height
                property var rowData: (panel.rows && index < panel.rows.length)
                                      ? panel.rows[index] : null

                Text {
                    id: labelText
                    anchors.left: parent.left
                    anchors.verticalCenter: parent.verticalCenter
                    width: parent.width * 0.55
                    text: {
                        if (!rowData || rowData.key === undefined)
                            return "";
                        var rowLabel = rowData.label !== undefined
                                ? rowData.label : rowData.key;
                        if (panel.layout && panel.layout.rows
                                && panel.layout.rows[rowData.key] !== undefined)
                            rowLabel = panel.layout.rows[rowData.key];
                        return String(rowLabel);
                    }
                    color: "#9fb4c7"
                    font.pixelSize: panel.captionFontPx
                    elide: Text.ElideRight
                }

                Text {
                    id: valueText
                    anchors.right: parent.right
                    anchors.verticalCenter: parent.verticalCenter
                    width: parent.width * 0.45
                    horizontalAlignment: Text.AlignRight
                    text: (rowData && rowData.value !== undefined
                           && rowData.value !== null)
                          ? String(rowData.value) : ""
                    color: (rowData && rowData.valid === false)
                           ? "#e25c5c" : "#e8eef4"
                    font.pixelSize: panel.valueFontPx
                    font.bold: true
                    elide: Text.ElideLeft
                }
            }
        }
    }
}
