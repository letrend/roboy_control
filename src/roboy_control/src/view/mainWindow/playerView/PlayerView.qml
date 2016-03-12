import QtQuick 2.4
import QtQuick.Controls 1.3
import QtQuick.Layouts 1.1

import Material 0.2
import Material.ListItems 0.1 as ListItem
import Material.Extras 0.1

import "./dialogs"
import "./multiLaneView"

View {
    property var selectedBehaviorIndex

    height  : 480
    id      : playerView
    width   : 640

    ColumnLayout {
        anchors.fill    : parent
        anchors.margins : Units.dp(32)
        spacing         : Units.dp(16)

        RowLayout {
            anchors.left    : parent.left
            anchors.margins : Units.dp(0)
            anchors.right   : parent.right
            spacing         : Units.dp(16)

            Button {
                elevation : 1
                id        : playButton
                onClicked : cpp_PlayerView.playButtonClicked()

                Icon {
                    anchors.centerIn : parent
                    name             : "av/play_arrow"
                }
            }

            Button {
                elevation : 1
                id        : pauseButton
                onClicked : cpp_PlayerView.pauseButtonClicked()

                Icon {
                    anchors.centerIn : parent
                    name             : "av/pause"
                }
            }

            Button {
                elevation : 1
                id        : stopButton
                onClicked : cpp_PlayerView.stopButtonClicked()

                Icon {
                    anchors.centerIn : parent
                    name             : "av/stop"
                }
            }

            Button {
                elevation : 1
                id        : skipButton
                onClicked : cpp_PlayerView.skipButtonClicked()

                Icon {
                    anchors.centerIn : parent
                    name             : "av/skip_next"
                }
            }
        }

        View {
            elevation         : 1
            Layout.fillHeight : true
            Layout.fillWidth  : true

            MultiLaneView {
                anchors.bottom : parent.bottom
                anchors.left   : parent.left
                anchors.right  : parent.right
                anchors.top    : parent.top
                id             : multiLaneView
                model          : cpp_MultiLaneViewModel

                ActionButton {
                    anchors.bottom  : parent.bottom
                    anchors.margins : Units.dp(16)
                    anchors.right   : parent.right
                    iconName        : "content/add"
                    onClicked       : cpp_PlayerView.addLaneButtonClicked()
                }
            }
        }

        RowLayout {
            spacing: Units.dp(16)

            View {
                elevation         : 1
                Layout.fillHeight : true
                Layout.fillWidth  :  true

                ListView {
                    anchors.fill : parent
                    delegate     : ListItem.Subtitled {
                        action         : Icon {
                            anchors.centerIn: parent
                            name: iconPath
                        }

                        Layout.margins : 0
                        selected       : index === selectedBehaviorIndex
                        subText        : "motor count: " + motorCount
                        text           : title

                        MouseArea {
                            acceptedButtons : Qt.RightButton
                            anchors.fill    : parent

                            onClicked       : {
                                if (Qt.RightButton) {
                                    behaviorListItemActionSheet.open()
                                }
                            }
                        }

                        BottomActionSheet {
                            actions : [
                                Action {
                                    iconName    : "content/add"
                                    name        : "Add to multi lane view"
                                    
                                    onTriggered : {
                                        if (multiLaneView.numLanes < 1) {
                                            showError("No lane in timeline", "To add a behavior to the timeline you first have to add a lane by clicking the plus-button in the toolbar.", "ok")
                                        } else {
                                            addBehaviorDialog.show()
                                        }
                                    }
                                },

                                Action {
                                    iconName : "content/clear"
                                    name     : "Cancel"
                                }
                            ]

                            id      : behaviorListItemActionSheet
                            title   : model.title
                        }

                        onClicked : {
                            behaviorNameValueLabel.text     = title    
                            idValueLabel.text               = "identifier: " + id
                            durationValueLabel.text         = "duration: " + duration + " ms"
                            motorCountValueLabel.text       = "motor count: " + motorCount
                            detailListView.model            = motorInfo 
                            selectedBehaviorIndex           = index
                            addToTimelineButton.enabled     = true
                        }

                        AddBehaviorDialog {
                            id         : addBehaviorDialog
                            numLanes   : multiLaneView.numLanes
                            onAccepted : {
                                switch(cpp_PlayerView.insertBehaviorHandler(index, laneSelector.selectedIndex, timestampLabel.text)) {
                                case -1: {
                                    showError("Error while inserting behavior", "The timestamp of a behavior has to be a multiple of the sample rate which currently is 100.", "ok")
                                }
                                    break;
                                case -2: {
                                    showError("Error while inserting behavior", "With the given timestamp, the behavior does overlap with other behaviors already in the timeline.", "ok")
                                }
                                    break;
                                }
                            }
                        }
                    }
                    model          : cpp_PVBehaviorListModel
                }
            }

            View {
                elevation         : 1
                Layout.fillHeight : true
                Layout.fillWidth  : true

                View {
                    anchors.left    : parent.left
                    anchors.right   : parent.right
                    anchors.top     : parent.top
                    backgroundColor : Theme.primaryColor
                    height          : detailTopColumn.implicitHeight + Units.dp(16)
                    id              : detailTopView

                    Column {
                        anchors.bottomMargin : Units.dp(16)
                        anchors.fill         : parent
                        anchors.topMargin    : Units.dp(16)
                        id                   : detailTopColumn

                        Label {
                            anchors.left    : parent.left
                            anchors.right   : parent.left
                            anchors.margins : Units.dp(16)
                            color           : "white"
                            id              : behaviorNameValueLabel
                            style           : "title"
                            text            : "-"
                        }

                        Item {
                            id                     : titleSpacer
                            Layout.fillWidth       : true
                            Layout.preferredHeight : Units.dp(8)
                        }

                        ListItem.Standard {
                            id : idItem

                            action: Icon {
                                anchors.centerIn : parent
                                color            : "white"
                                name             : "action/accessibility"
                            }

                            content : Label {
                                anchors.centerIn  : parent
                                color             : "white"
                                font.family       : "Roboto"
                                id                : idValueLabel
                                Layout.leftMargin : Units.dp(16)
                                style             : "subheading"
                                text              : "-"
                                width             : parent.width
                            }
                        }

                        ListItem.Standard {
                            id : durationItem

                            action : Icon {
                                anchors.centerIn : parent
                                color            : "white"
                                name             : "action/alarm_on"
                            }

                            content : Label {
                                anchors.centerIn  : parent
                                color             : "white"
                                font.family       : "Roboto"
                                id                : durationValueLabel
                                Layout.leftMargin : Units.dp(16)
                                style             : "subheading"
                                text              : "-"
                                width             : parent.width
                            }
                        }

                        ListItem.Standard {
                            id : motorCountItem

                            action : Icon {
                                anchors.centerIn : parent
                                color            : "white"
                                name             : "av/album"
                            }

                            content : Label {
                                anchors.centerIn  : parent
                                color             : "white"
                                font.family       : "Roboto"
                                id                : motorCountValueLabel
                                Layout.leftMargin : Units.dp(16)
                                style             : "subheading"
                                text              : "-"
                                width             : parent.width
                            }
                        }
                    }
                }

                ListView {
                    anchors.bottom   : detailViewButtonView.top
                    anchors.left     : parent.left
                    anchors.right    : parent.right
                    anchors.top      : detailTopView.bottom
                    id               : detailListView
                    Layout.fillWidth : true

                    delegate        : ListItem.Standard {
                        action : Icon {
                            anchors.centerIn : parent
                            name             : "av/album"
                        }
                        text        : modelData
                    }
                }

                View {
                    anchors.bottom : parent.bottom
                    anchors.left   : parent.left
                    anchors.right  : parent.right
                    height         : Units.dp(68)
                    id             : detailViewButtonView

                    RowLayout {
                        anchors.fill    : parent
                        anchors.margins : Units.dp(16)

                        Button {
                            anchors.right : parent.right
                            id            : addToTimelineButton
                            onClicked     : {
                                if (multiLaneView.numLanes < 1) {
                                    showError("No lane in timeline", "To add a behavior to the timeline you first have to add a lane by clicking the plus-button in the toolbar.", "ok")
                                } else {
                                    addBehaviorDialog.show()
                                }
                            }
                            textColor     : Theme.accentColor
                            text          : "ADD TO TIMELINE"
                        }
                    }

                    AddBehaviorDialog {
                        id         : addBehaviorDialog
                        numLanes   : multiLaneView.numLanes
                        onAccepted : {
                            switch(cpp_PlayerView.insertBehaviorHandler(selectedBehaviorIndex, laneSelector.selectedIndex, timestampLabel.text)) {
                            case -1: {
                                showError("Error while inserting behavior", "The timestamp of a behavior has to be a multiple of the sample rate which currently is 100.", "ok")
                            }
                                break;
                            case -2: {
                                showError("Error while inserting behavior", "With the given timestamp, the behavior does overlap with other behaviors already in the timeline.", "ok")
                            }
                                break;
                            }
                        }
                    }
                }
            }
        }
    }
}
