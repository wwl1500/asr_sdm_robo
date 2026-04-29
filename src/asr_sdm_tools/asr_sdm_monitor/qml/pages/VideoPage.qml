import QtQuick
import QtQuick.Controls
import QtQuick.Layouts
import RosUi 1.0
import "../components"
import "../utils/I18n.js" as I18n

Item {
    id: root
    property var appPalette
    property string language: "en"

    function makeTopicOptions(topics) {
        let result = [{ label: I18n.t(root.language, "none"), topic: "" }]
        if (topics) {
            for (let i = 0; i < topics.length; ++i)
                result.push({ label: String(topics[i]), topic: String(topics[i]) })
        }
        return result
    }

    function topicAt(index) {
        if (index < 0 || index >= topicOptions.length)
            return ""
        return topicOptions[index].topic
    }

    function indexOfTopic(topic) {
        for (let i = 0; i < topicOptions.length; ++i) {
            if (topicOptions[i].topic === topic)
                return i
        }
        return 0
    }

    function slotTopic(slotIndex) {
        return slotIndex === 0 ? RosUi.videoTopic0 : RosUi.videoTopic1
    }

    function slotStatus(slotIndex) {
        return slotIndex === 0 ? RosUi.videoStatus0 : RosUi.videoStatus1
    }

    function slotFrameRevision(slotIndex) {
        return slotIndex === 0 ? RosUi.videoFrame0Revision : RosUi.videoFrame1Revision
    }

    readonly property int topicComboMinWidth: 64
    readonly property int topicComboMaxWidth: 520
    readonly property var topicOptions: makeTopicOptions(RosUi.videoTopics)

    function scaledTopicComboWidth(headerWidth) {
        let targetWidth = headerWidth * 0.34
        return Math.max(topicComboMinWidth, Math.min(topicComboMaxWidth, targetWidth))
    }

    function scaledTopicPopupHeight(slotHeight, contentHeight) {
        let targetHeight = Math.max(160, Math.min(360, slotHeight * 0.45))
        return Math.min(contentHeight, targetHeight)
    }

    ColumnLayout {
        anchors.fill: parent
        spacing: 16

        RowLayout {
            Layout.fillWidth: true
            Layout.fillHeight: true
            spacing: 16

            Repeater {
                model: 2

                delegate: Rectangle {
                    id: videoSlot
                    property int slotIndex: index
                    readonly property string selectedTopic: root.slotTopic(slotIndex)
                    readonly property int frameRevision: root.slotFrameRevision(slotIndex)
                    readonly property string streamStatus: root.slotStatus(slotIndex)

                    Layout.fillWidth: true
                    Layout.fillHeight: true
                    Layout.minimumWidth: 0
                    clip: true
                    radius: 12
                    color: root.appPalette.surfaceBackground
                    border.color: root.appPalette.border
                    border.width: 1

                    ColumnLayout {
                        anchors.fill: parent
                        anchors.margins: 16
                        spacing: 12
                        clip: true

                        RowLayout {
                            id: headerRow
                            Layout.fillWidth: true
                            Layout.minimumWidth: 0
                            spacing: 8
                            clip: true

                            Text {
                                id: videoWindowTitle
                                Layout.fillWidth: true
                                Layout.minimumWidth: 0
                                text: I18n.t(root.language, videoSlot.slotIndex === 0 ? "videoWindowLeft" : "videoWindowRight")
                                font.pixelSize: 18
                                font.bold: true
                                color: root.appPalette.textPrimary
                                elide: Text.ElideRight
                            }

                            Text {
                                id: topicLabel
                                text: I18n.t(root.language, "videoTopic")
                                font.pixelSize: 16
                                color: root.appPalette.textSecondary
                                elide: Text.ElideRight
                            }

                            ComboBox {
                                id: topicCombo
                                readonly property real comboMaxInsideHeader: Math.max(root.topicComboMinWidth, headerRow.width - topicLabel.implicitWidth - 24)
                                Layout.minimumWidth: Math.min(root.topicComboMinWidth, comboMaxInsideHeader)
                                Layout.maximumWidth: Math.min(root.topicComboMaxWidth, comboMaxInsideHeader)
                                Layout.preferredWidth: Math.min(Math.min(root.topicComboMaxWidth, comboMaxInsideHeader), root.scaledTopicComboWidth(headerRow.width))
                                model: root.topicOptions
                                textRole: "label"
                                currentIndex: root.indexOfTopic(videoSlot.selectedTopic)
                                onActivated: RosUi.setVideoTopic(videoSlot.slotIndex, root.topicAt(currentIndex))

                                palette.window: root.appPalette.inputBackground
                                palette.button: root.appPalette.inputBackground
                                palette.base: root.appPalette.inputBackground
                                palette.text: root.appPalette.textPrimary
                                palette.buttonText: root.appPalette.textPrimary
                                palette.highlight: root.appPalette.accent
                                palette.highlightedText: root.appPalette.accentText

                                background: Rectangle {
                                    radius: 8
                                    color: root.appPalette.inputBackground
                                    border.color: root.appPalette.controlBorder
                                    border.width: 1
                                }

                                contentItem: Text {
                                    leftPadding: 12
                                    rightPadding: 36
                                    verticalAlignment: Text.AlignVCenter
                                    text: topicCombo.displayText
                                    font.pixelSize: 15
                                    color: root.appPalette.textPrimary
                                    elide: Text.ElideRight
                                }

                                delegate: ItemDelegate {
                                    id: topicDelegate
                                    width: topicCombo.width
                                    highlighted: topicCombo.highlightedIndex === index

                                    contentItem: Text {
                                        text: modelData.label
                                        font.pixelSize: 15
                                        color: topicDelegate.highlighted ? root.appPalette.accentText : root.appPalette.textPrimary
                                        elide: Text.ElideRight
                                        verticalAlignment: Text.AlignVCenter
                                    }

                                    background: Rectangle {
                                        color: topicDelegate.highlighted ? root.appPalette.accent : root.appPalette.inputBackground
                                    }
                                }

                                popup: Popup {
                                    y: topicCombo.height
                                    width: topicCombo.width
                                    implicitHeight: root.scaledTopicPopupHeight(videoSlot.height, contentItem.implicitHeight)
                                    padding: 1

                                    contentItem: ListView {
                                        clip: true
                                        implicitHeight: contentHeight
                                        model: topicCombo.popup.visible ? topicCombo.delegateModel : null
                                        currentIndex: topicCombo.highlightedIndex
                                        ScrollIndicator.vertical: ScrollIndicator { }
                                    }

                                    background: Rectangle {
                                        radius: 8
                                        color: root.appPalette.inputBackground
                                        border.color: root.appPalette.controlBorder
                                        border.width: 1
                                    }
                                }
                            }
                        }

                        Rectangle {
                            id: videoFrame
                            Layout.fillWidth: true
                            Layout.fillHeight: true
                            Layout.minimumWidth: 0
                            Layout.minimumHeight: 0
                            radius: 10
                            color: root.appPalette.inputBackground
                            border.color: root.appPalette.border
                            border.width: 1
                            clip: true

                            Image {
                                id: frameImage
                                anchors.fill: parent
                                anchors.margins: 10
                                cache: false
                                asynchronous: false
                                fillMode: Image.PreserveAspectFit
                                visible: videoSlot.selectedTopic !== "" && videoSlot.frameRevision > 0
                                source: visible ? "image://rosvideo/" + videoSlot.slotIndex + "/" + videoSlot.frameRevision : ""
                            }

                            Column {
                                anchors.centerIn: parent
                                spacing: 8
                                visible: !frameImage.visible

                                Text {
                                    anchors.horizontalCenter: parent.horizontalCenter
                                    width: Math.max(0, Math.min(videoFrame.width - 48, 520))
                                    text: videoSlot.selectedTopic === ""
                                          ? I18n.t(root.language, "noTopicSelected")
                                          : I18n.t(root.language, "waitingVideoFrame")
                                    font.pixelSize: 18
                                    font.bold: true
                                    color: root.appPalette.textPrimary
                                    horizontalAlignment: Text.AlignHCenter
                                    wrapMode: Text.Wrap
                                }

                                Text {
                                    anchors.horizontalCenter: parent.horizontalCenter
                                    width: Math.max(0, Math.min(videoFrame.width - 48, 520))
                                    text: (RosUi.videoTopics.length === 0 && videoSlot.selectedTopic === "")
                                          ? I18n.t(root.language, "noVideoTopics")
                                          : videoSlot.streamStatus
                                    font.pixelSize: 15
                                    color: root.appPalette.textSecondary
                                    horizontalAlignment: Text.AlignHCenter
                                    wrapMode: Text.WrapAnywhere
                                }
                            }
                        }

                        RowLayout {
                            id: statusRow
                            Layout.fillWidth: true
                            Layout.minimumWidth: 0
                            spacing: 8
                            clip: true

                            Text {
                                text: I18n.t(root.language, "currentVideoTopic")
                                font.pixelSize: 15
                                color: root.appPalette.textSecondary
                            }

                            Text {
                                Layout.fillWidth: true
                                Layout.minimumWidth: 0
                                text: videoSlot.selectedTopic === "" ? I18n.t(root.language, "none") : videoSlot.selectedTopic
                                font.pixelSize: 15
                                color: root.appPalette.textPrimary
                                elide: Text.ElideMiddle
                            }

                            Text {
                                Layout.maximumWidth: Math.max(72, statusRow.width * 0.42)
                                Layout.minimumWidth: 0
                                text: I18n.t(root.language, "videoStatus") + " " + videoSlot.streamStatus
                                font.pixelSize: 15
                                color: root.appPalette.textSecondary
                                elide: Text.ElideRight
                            }
                        }
                    }
                }
            }
        }
    }

    Component.onCompleted: RosUi.refreshVideoTopics()
}
