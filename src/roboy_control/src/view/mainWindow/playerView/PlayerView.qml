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

    id      : playerView
    width   : 640
    height  : 480

    ColumnLayout {

        anchors.fill    : parent
        anchors.margins : Units.dp(32)
        spacing         : Units.dp(16)

        RowLayout {

            anchors.left: parent.left
            anchors.right: parent.right
            anchors.margins: Units.dp(0)
            spacing: Units.dp(16)

            Button {

                id: playButton
                elevation: 1

                Icon {
                    anchors.centerIn: parent
                    name: "av/play_arrow"
                }
            }

            Button {

                id: pauseButton
                elevation: 1

                Icon {
                    anchors.centerIn: parent
                    name: "av/pause"
                }
            }

            Button {

                id: stopButton
                elevation: 1

                Icon {
                    anchors.centerIn: parent
                    name: "av/stop"
                }
            }

            Button {

                id: skipButton
                elevation: 1

                Icon {
                    anchors.centerIn: parent
                    name: "av/skip_next"
                }
            }
/*
            MenuField {
                id: scaleFactorSelector
                anchors.right: parent.right
                model: ["milliseconds", "centiseconds", "deciseconds", "seconds"]
                onSelectedIndexChanged: {
                    switch(selectedIndex) {
                    case 0:
                        multiLaneView.scaleFactor = 1
                        break;
                    case 1:
                        multiLaneView.scaleFactor = 10
                        break;
                    case 2:
                        multiLaneView.scaleFactor = 100
                        break;
                    case 3:
                        multiLaneView.scaleFactor = 1000
                        break;
                    }
                }
            }*/

            Button {

                id: addLaneButton
                elevation: 1
                anchors.right: scaleFactorButton.left
                anchors.margins: Units.dp(16)

                Icon {

                    anchors.centerIn: parent
                    name: "content/add"
                }
            }

            Button {

                id: scaleFactorButton
                elevation: 1
                anchors.right: parent.right
                onClicked: scaleFactorDialog.show()

                Icon {

                    anchors.centerIn: parent
                    name: "action/zoom_in"
                }

                Dialog {

                    id: scaleFactorDialog
                    title: "Select the behavior scale factor"
                    positiveButtonText: "Select"

                    Slider {

                        id                      : scaleFactorSlider
                        anchors.left            : parent.left
                        anchors.right           : parent.right
                        height                  : 100
                        tickmarksEnabled        : true
                        numericValueLabel       : true
                        alwaysShowValueLabel    : true
                        stepSize                : 1
                        minimumValue            : 1
                        maximumValue            : 4
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

            Layout.fillHeight: true
            Layout.fillWidth: true
            elevation: 1

            ColumnLayout {

                anchors.fill: parent

                MultiLaneView {

                    id: multiLaneView
                    model: myModel
                    anchors.fill: parent
                }
            }
        }

        RowLayout {

            spacing: Units.dp(16)

            View {

                Layout.fillHeight: true
                Layout.fillWidth:  true
                elevation: 1

                ListView {

                    anchors.fill: parent
                    model: testModel.count();
                    delegate: ListItem.Subtitled {
                        selected: testModel.at(index, 0) === selectedBehaviorID
                        text: testModel.at(index, 1)
                        subText: testModel.at(index, 2)
                        Layout.margins: 0

                        action: Icon {
                            anchors.centerIn: parent
                            name: "action/accessibility"
                        }

                        MouseArea {

                            anchors.fill: parent
                            acceptedButtons: Qt.RightButton

                            onClicked: {
                                if (Qt.RightButton) {
                                    behaviorListItemActionSheet.open()
                                }
                            }
                        }

                        BottomActionSheet {

                            id: behaviorListItemActionSheet
                            title: testModel.at(index, 1)
/*
                            actions: [

                                Action {
                                    iconName: "content/add"
                                    name: "Add to timeline"
                                },

                                Action {
                                    iconName: "content/clear"
                                    name: "Cancel"
                                }

                            ]*/
                        }

                        onClicked: {
                            selectedBehaviorID          = testModel.at(index, 0)
                            idValueLabel.text           = testModel.at(index, 0)
                            behaviorNameValueLabel.text = testModel.at(index, 1)
                            motorCountValueLabel.text   = testModel.at(index, 2)

                        }
                    }
                }
            }

            View {

                Layout.fillHeight: true
                Layout.fillWidth:  true
                elevation: 1

                ScrollView {

                    anchors.fill: parent

                    ColumnLayout {

                        Label {
                            id: behaviorNameLabel
                            text: "Behavior Name"
                            Layout.topMargin: Units.dp(16)
                            Layout.leftMargin: Units.dp(16)
                            font.family: "Roboto"
                            style: "subheading"

                        }

                        Label {

                            id: behaviorNameValueLabel
                            text: "-"
                            Layout.leftMargin: Units.dp(16)

                        }

                        Label {

                            id: idLabel
                            text: "ID"
                            Layout.leftMargin: Units.dp(16)
                            font.family: "Roboto"
                            style: "subheading"

                        }

                        Label {
                            id: idValueLabel
                            text: "-"
                            Layout.leftMargin: Units.dp(16)
                        }

                        Label {

                            id: motorCountLabel
                            text: "Motor Count"
                            Layout.leftMargin: Units.dp(16)
                            font.family: "Roboto"
                            style: "subheading"

                        }

                        Label {

                            id: motorCountValueLabel
                            text: "-"
                            Layout.leftMargin: Units.dp(16)
                        }

                        Label {

                            id: descriptionLabel
                            text: "Description"
                            Layout.leftMargin: Units.dp(16)
                            font.family: "Roboto"
                            style: "subheading"

                        }
                    }
                }
            }
        }
    }

    AddBehaviorDialog {
        id: addBehaviorDialog
    }

    ActionButton {

        anchors.right: parent.right
        anchors.bottom: parent.bottom
        anchors.margins: Units.dp(48)
        iconName: "content/add"
        onClicked: addBehaviorDialog.show()

    }
}
