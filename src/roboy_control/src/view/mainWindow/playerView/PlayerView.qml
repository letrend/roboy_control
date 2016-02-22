import QtQuick 2.4
import QtQuick.Controls 1.3
import QtQuick.Layouts 1.1

import Material 0.2
import Material.ListItems 0.1 as ListItem
import Material.Extras 0.1

import "./dialogs"
import "./multiLaneView"

View {
    property var selectedBehaviorID

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

                Icon {
                    anchors.centerIn : parent
                    name             : "av/play_arrow"
                }
            }

            Button {
                elevation : 1
                id        : pauseButton

                Icon {
                    anchors.centerIn : parent
                    name             : "av/pause"
                }
            }

            Button {
                elevation : 1
                id        : stopButton

                Icon {
                    anchors.centerIn : parent
                    name             : "av/stop"
                }
            }

            Button {
                elevation : 1
                id        : skipButton

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

                Dialog {
                    id                : scaleFactorDialog
                    title             : "Select the behavior scale factor"
                    positiveButtonText: "Select"

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
                                break;
                            case 2:
                                multiLaneView.scaleFactor = 100
                                break;
                            case 3:
                                multiLaneView.scaleFactor = 10
                                break;
                            case 4:
                                multiLaneView.scaleFactor = 1
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
                        selected       : id === selectedBehaviorID
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
                                    iconName: "content/add"
                                    name: "Add to timeline"
                                },

                                Action {
                                    iconName: "content/clear"
                                    name: "Cancel"
                                }
                            ]

                            id      : behaviorListItemActionSheet
                            title   : model.title
                        }

                        onClicked: {
                            behaviorNameValueLabel.text = title    
                            descriptionValueLabel.text  = description
                            idValueLabel.text           = id
                            motorCountValueLabel.text   = motorCount
                            selectedBehaviorID          = id
                        }
                    }
                    model        : cpp_BehaviorListModel
                }
            }

            View {
                elevation         : 1
                Layout.fillHeight : true
                Layout.fillWidth  :  true

                ScrollView {
                    anchors.fill : parent

                    ColumnLayout {
                        Label {
                            font.family       : "Roboto"
                            id                : behaviorNameLabel
                            Layout.topMargin  : Units.dp(16)
                            Layout.leftMargin : Units.dp(16)
                            style             : "subheading"
                            text              : "Behavior Name"
                        }

                        Label {
                            id                : behaviorNameValueLabel
                            Layout.leftMargin : Units.dp(16)
                            text              : "-"
                        }

                        Label {
                            font.family       : "Roboto"
                            id                : idLabel
                            Layout.leftMargin : Units.dp(16)
                            style             : "subheading"
                            text              : "ID"
                        }

                        Label {
                            id                : idValueLabel
                            Layout.leftMargin : Units.dp(16)
                            text              : "-"
                        }

                        Label {
                            font.family       : "Roboto"
                            id                : motorCountLabel
                            Layout.leftMargin : Units.dp(16)
                            style             : "subheading"
                            text              : "Motor Count"
                        }

                        Label {
                            id                : motorCountValueLabel
                            Layout.leftMargin : Units.dp(16)
                            text              : "-"
                        }

                        Label {
                            font.family       : "Roboto"
                            id                : descriptionLabel
                            Layout.leftMargin : Units.dp(16)
                            style             : "subheading"
                            text              : "Description"
                        }

                        Label {
                            id                : descriptionValueLabel
                            Layout.leftMargin : Units.dp(16)
                            text              : "-"
                        }
                    }
                }
            }
        }
    }

    AddBehaviorDialog {
        id : addBehaviorDialog
    }

    ActionButton {
        anchors.bottom  : parent.bottom
        anchors.margins : Units.dp(48)
        anchors.right   : parent.right
        iconName        : "content/add"
        onClicked       : addBehaviorDialog.show()
    }
}
