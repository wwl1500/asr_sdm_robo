import QtQuick
import QtQuick.Controls
import QtQuick.Layouts
import "../utils/I18n.js" as I18n

Rectangle {
    id: root
    property var appPalette
    property string language: "en"
    property int currentSection: 0
    property string rosStatus: "--"
    signal sectionSelected(int index)

    color: appPalette.sidebarBackground
    border.color: appPalette.border
    border.width: 1

    Column {
        anchors.fill: parent
        anchors.margins: Math.max(8, Math.min(20, root.width * 0.08))
        spacing: root.width < 150 ? 8 : 14

        SelectableButton {
            label: I18n.t(root.language, "hardware")
            selected: root.currentSection === 0
            appPalette: root.appPalette
            implicitWidth: parent.width
            implicitHeight: root.width < 150 ? 46 : 56
            labelPixelSize: root.width < 150 ? 13 : 16
            onClicked: root.sectionSelected(0)
        }

        SelectableButton {
            label: I18n.t(root.language, "video")
            selected: root.currentSection === 1
            appPalette: root.appPalette
            implicitWidth: parent.width
            implicitHeight: root.width < 150 ? 46 : 56
            labelPixelSize: root.width < 150 ? 13 : 16
            onClicked: root.sectionSelected(1)
        }
    }
}
