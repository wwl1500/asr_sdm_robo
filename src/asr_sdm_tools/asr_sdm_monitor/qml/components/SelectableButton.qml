import QtQuick
import QtQuick.Controls

Rectangle {
    id: root
    property string label: ""
    property bool selected: false
    property var appPalette
    property int radiusValue: 10
    property int labelPixelSize: 16
    signal clicked

    radius: radiusValue
    color: selected ? appPalette.accent : appPalette.controlBackground
    border.color: selected ? appPalette.accentBorder : appPalette.controlBorder
    border.width: 1
    implicitHeight: 44
    implicitWidth: 140

    Text {
        anchors.centerIn: parent
        width: Math.max(0, parent.width - 16)
        horizontalAlignment: Text.AlignHCenter
        elide: Text.ElideRight
        text: root.label
        font.pixelSize: root.labelPixelSize
        font.bold: root.selected
        color: root.selected ? appPalette.accentText : appPalette.textPrimary
    }

    MouseArea {
        anchors.fill: parent
        hoverEnabled: true
        cursorShape: Qt.PointingHandCursor
        onClicked: root.clicked()
    }
}
