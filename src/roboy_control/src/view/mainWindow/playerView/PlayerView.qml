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

            Button {
                anchors.margins : Units.dp(16)
                anchors.right   : scaleFactorButton.left
                elevation       : 1
                id              : addLaneButton
                onClicked       : cpp_PlayerView.addLaneButtonClicked()
                
                Icon {
                    anchors.centerIn : parent
                    name             : "content/add"
                }
            }

            Button {
                anchors.right : parent.right
                elevation     : 1
                id            : scaleFactorButton
                onClicked     : scaleFactorDialog.show()

                Icon {
                    anchors.centerIn : parent
                    name             : "action/zoom_in"
                }
            }
        }

        View {
            elevation         : 1
            Layout.fillHeight : true
            Layout.fillWidth  : true

            ColumnLayout {
                anchors.fill : parent

                MultiLaneView {
                    anchors.fill : parent
                    id           : multiLaneView
                    model        : cpp_MultiLaneViewModel
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
                        subText        : "motor count " + motorCount
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
                                        addBehaviorDialog.show()
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
                            addBehaviorActionButton.enabled = true
                        }

                        AddBehaviorDialog {
                            id         : addBehaviorDialog
                            numLanes   : multiLaneView.numLanes
                            onAccepted : {
                                cpp_PlayerView.insertBehaviorHandler(index, laneSelector.selectedIndex, timestampTextField.text)
                            }
                        }
                    }
                    model          : cpp_BehaviorListModel
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
                    height          : Units.dp(200)
                    id              : detailTopView

                    ColumnLayout {
                        anchors.bottomMargin : Units.dp(16)
                        anchors.fill         : parent
                        anchors.topMargin    : Units.dp(16)
                        id : column

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
                            anchors.top            : behaviorNameValueLabel.bottom
                            id                     : titleSpacer
                            Layout.fillWidth       : true
                            Layout.preferredHeight : Units.dp(8)
                        }

                        ListItem.Standard {
                            anchors.top : titleSpacer.bottom
                            id          : idItem

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
                            anchors.top : idItem.bottom
                            id          : durationItem

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
                            anchors.top : durationItem.bottom
                            id          : motorCountItem

                            action : Icon {
                                anchors.centerIn : parent
                                color            : "white"
                                name             : "action/autorenew"
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
                    anchors.bottom   : parent.bottom
                    anchors.left     : parent.left
                    anchors.right    : parent.right
                    anchors.top      : detailTopView.bottom
                    id               : detailListView
                    Layout.fillWidth : true

                    delegate        : ListItem.Standard {
                        action : Icon {
                            anchors.centerIn: parent
                            name: "av/album"
                        }
                        text        : modelData
                    }
                }

                ActionButton {
                    anchors.verticalCenter  : detailTopView.bottom
                    anchors.margins : Units.dp(16)
                    anchors.right   : parent.right
                    enabled         : false
                    iconName        : "content/add"
                    id              : addBehaviorActionButton
                    onClicked       : addBehaviorDialog.show()           

                    AddBehaviorDialog {
                        id         : addBehaviorDialog
                        numLanes   : multiLaneView.numLanes
                        onAccepted : {
                            cpp_PlayerView.insertBehaviorHandler(index, laneSelector.selectedIndex, timestampTextField.text)
                        }
                    }
                }
            }
        }
    }

    Dialog {
        id                 : scaleFactorDialog
        title              : "Select the behavior scale factor"
        positiveButtonText : "Select"

        Label { 
            text: "current scale factor"
        }

        Label { 
            color             : Theme.light.subTextColor
            elide             : Text.ElideRight
            id                : scaleDescriptionLabel
            Layout.leftMargin : Units.dp(16)
            style             : "body1"
            wrapMode          : Text.WordWrap
        }

        Slider {
            alwaysShowValueLabel    : true
            anchors.left            : parent.left
            anchors.right           : parent.right
            height                  : 100
            id                      : scaleFactorSlider
            maximumValue            : 4
            minimumValue            : 1
            numericValueLabel       : true
            stepSize                : 1
            tickmarksEnabled        : true
            value                   : 4

            onValueChanged: {
                switch(value) {
                case 1:
                    multiLaneView.scaleFactor = 1000
                    scaleDescriptionLabel.text = "1: seconds"
                    break;
                case 2:
                    multiLaneView.scaleFactor = 100
                    scaleDescriptionLabel.text = "2: deciseconds"
                    break;
                case 3:
                    multiLaneView.scaleFactor = 10
                    scaleDescriptionLabel.text = "3: centiseconds"
                    break;
                case 4:
                    multiLaneView.scaleFactor = 1
                    scaleDescriptionLabel.text = "4: milliseconds"
                    break;
                }
            }
        }

        onRejected: {
            multiLaneView.scaleFactor = 1
            scaleFactorSlider.value   = 4
        }
    }
}
