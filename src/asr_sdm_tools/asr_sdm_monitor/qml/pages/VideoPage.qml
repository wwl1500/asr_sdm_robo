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
    property int windowCount: 2

    readonly property int topicComboMinWidth: 72
    readonly property int topicComboMaxWidth: 520
    readonly property int countComboMinWidth: 72
    readonly property int countComboMaxWidth: 120
    readonly property var topicOptions: makeTopicOptions(RosUi.videoTopics)
    readonly property var windowCountOptions: [
        { label: "1", value: 1 },
        { label: "2", value: 2 },
        { label: "3", value: 3 },
        { label: "4", value: 4 }
    ]

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

    function slotData(slotIndex) {
        const slots = RosUi.videoSlots
        if (slots && slotIndex >= 0 && slotIndex < slots.length)
            return slots[slotIndex]
        return { topic: "", status: I18n.t(root.language, "noTopicSelected"), frameRevision: 0 }
    }

    function slotTopic(slotIndex) {
        const slot = slotData(slotIndex)
        return slot.topic ? String(slot.topic) : ""
    }

    function slotStatus(slotIndex) {
        const slot = slotData(slotIndex)
        return slot.status ? String(slot.status) : ""
    }

    function slotFrameRevision(slotIndex) {
        const slot = slotData(slotIndex)
        return slot.frameRevision ? Number(slot.frameRevision) : 0
    }

    function scaledTopicComboWidth(headerWidth) {
        let targetWidth = headerWidth * (headerWidth < 420 ? 0.46 : 0.36)
        return Math.max(topicComboMinWidth, Math.min(topicComboMaxWidth, targetWidth))
    }

    function scaledTopicPopupHeight(slotHeight, contentHeight) {
        let targetHeight = Math.max(140, Math.min(360, slotHeight * 0.45))
        return Math.min(contentHeight, targetHeight)
    }

    function windowTitle(slotIndex) {
        return I18n.t(root.language, "videoWindow") + " " + (slotIndex + 1)
    }

    function localizedStatus(status) {
        if (status === "No topic selected")
            return I18n.t(root.language, "noTopicSelected")
        if (status === "Waiting for video frame")
            return I18n.t(root.language, "waitingVideoFrame")
        return status
    }

    onWindowCountChanged: {
        if (windowCount < 1)
            windowCount = 1
        if (windowCount > 4)
            windowCount = 4
        for (let i = windowCount; i < 4; ++i) {
            if (slotTopic(i) !== "")
                RosUi.setVideoTopic(i, "")
        }
    }

    ColumnLayout {
        anchors.fill: parent
        spacing: root.width < 720 ? 10 : 16

        RowLayout {
            id: videoControlRow
            Layout.fillWidth: true
            Layout.minimumWidth: 0
            spacing: root.width < 720 ? 8 : 12
            clip: true

            Text {
                Layout.maximumWidth: Math.max(108, videoControlRow.width * 0.34)
                Layout.minimumWidth: 0
                text: I18n.t(root.language, "videoWindowCount")
                font.pixelSize: 15
                color: root.appPalette.textPrimary
                elide: Text.ElideRight
                verticalAlignment: Text.AlignVCenter
            }

            ComboBox {
                id: windowCountCombo
                Layout.minimumWidth: root.countComboMinWidth
                Layout.maximumWidth: root.countComboMaxWidth
                Layout.preferredWidth: Math.max(root.countComboMinWidth, Math.min(root.countComboMaxWidth, videoControlRow.width * 0.12))
                model: root.windowCountOptions
                textRole: "label"
                currentIndex: Math.max(0, Math.min(3, root.windowCount - 1))
                onActivated: root.windowCount = root.windowCountOptions[currentIndex].value

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
                    rightPadding: 30
                    verticalAlignment: Text.AlignVCenter
                    text: windowCountCombo.displayText
                    font.pixelSize: 15
                    color: root.appPalette.textPrimary
                    elide: Text.ElideRight
                }

                delegate: ItemDelegate {
                    id: countDelegate
                    width: windowCountCombo.width
                    highlighted: windowCountCombo.highlightedIndex === index

                    contentItem: Text {
                        text: modelData.label
                        font.pixelSize: 15
                        color: countDelegate.highlighted ? root.appPalette.accentText : root.appPalette.textPrimary
                        elide: Text.ElideRight
                        verticalAlignment: Text.AlignVCenter
                    }

                    background: Rectangle {
                        color: countDelegate.highlighted ? root.appPalette.accent : root.appPalette.inputBackground
                    }
                }

                popup: Popup {
                    y: windowCountCombo.height
                    width: windowCountCombo.width
                    implicitHeight: Math.min(220, contentItem.implicitHeight)
                    padding: 1

                    contentItem: ListView {
                        clip: true
                        implicitHeight: contentHeight
                        model: windowCountCombo.popup.visible ? windowCountCombo.delegateModel : null
                        currentIndex: windowCountCombo.highlightedIndex
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

            Item { Layout.fillWidth: true }
        }

        GridLayout {
            id: videoGrid
            Layout.fillWidth: true
            Layout.fillHeight: true
            Layout.minimumWidth: 0
            Layout.minimumHeight: 0
            columns: root.windowCount === 1 ? 1 : 2
            rowSpacing: root.width < 720 ? 10 : 16
            columnSpacing: root.width < 720 ? 10 : 16
            clip: true

            Repeater {
                model: root.windowCount

                delegate: Rectangle {
                    id: videoSlot
                    property int slotIndex: index
                    readonly property string selectedTopic: root.slotTopic(slotIndex)
                    readonly property int frameRevision: root.slotFrameRevision(slotIndex)
                    readonly property string streamStatus: root.slotStatus(slotIndex)

                    Layout.fillWidth: true
                    Layout.fillHeight: true
                    Layout.minimumWidth: 0
                    Layout.minimumHeight: 0
                    clip: true
                    radius: 12
                    color: root.appPalette.surfaceBackground
                    border.color: root.appPalette.border
                    border.width: 1

                    ColumnLayout {
                        anchors.fill: parent
                        anchors.margins: Math.max(8, Math.min(16, Math.min(videoSlot.width, videoSlot.height) * 0.045))
                        spacing: videoSlot.height < 280 ? 8 : 12
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
                                text: root.windowTitle(videoSlot.slotIndex)
                                font.pixelSize: videoSlot.width < 360 ? 15 : 18
                                font.bold: true
                                color: root.appPalette.textPrimary
                                elide: Text.ElideRight
                            }

                            Text {
                                id: topicLabel
                                text: I18n.t(root.language, "videoTopic")
                                font.pixelSize: videoSlot.width < 360 ? 14 : 16
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
                                    font.pixelSize: videoSlot.width < 360 ? 13 : 15
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
                                anchors.margins: Math.max(6, Math.min(10, Math.min(videoFrame.width, videoFrame.height) * 0.035))
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
                                    font.pixelSize: videoFrame.width < 360 ? 15 : 18
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
                                          : root.localizedStatus(videoSlot.streamStatus)
                                    font.pixelSize: videoFrame.width < 360 ? 13 : 15
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
                                font.pixelSize: videoSlot.width < 360 ? 13 : 15
                                color: root.appPalette.textSecondary
                            }

                            Text {
                                Layout.fillWidth: true
                                Layout.minimumWidth: 0
                                text: videoSlot.selectedTopic === "" ? I18n.t(root.language, "none") : videoSlot.selectedTopic
                                font.pixelSize: videoSlot.width < 360 ? 13 : 15
                                color: root.appPalette.textPrimary
                                elide: Text.ElideMiddle
                            }

                            Text {
                                Layout.maximumWidth: Math.max(72, statusRow.width * 0.42)
                                Layout.minimumWidth: 0
                                text: I18n.t(root.language, "videoStatus") + " " + root.localizedStatus(videoSlot.streamStatus)
                                font.pixelSize: videoSlot.width < 360 ? 13 : 15
                                color: root.appPalette.textSecondary
                                elide: Text.ElideRight
                            }
                        }
                    }
                }
            }
        }
    }

}
