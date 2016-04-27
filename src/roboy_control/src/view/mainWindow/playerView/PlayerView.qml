import QtQuick 2.4
import QtQuick.Controls 1.3
import QtQuick.Layouts 1.1

import Material 0.2
import Material.ListItems 0.1 as ListItem
import Material.Extras 0.1
import PlayerState 1.0

import "./dialogs"
import "./multiLaneView"

View {
    property var selectedBehaviorIndex

    height  : 480
    id      : playerView
    width   : 640

    Component.onCompleted : {
        setupPlayerState()
    }

    Connections {
        target                      : cpp_PlayerView
        onSignalPlayerStatusUpdated : {
            setupPlayerState()
        }
    }

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
                enabled   : false
                id        : playButton
                onClicked : cpp_PlayerView.playButtonClicked()

                Icon {
                    anchors.centerIn : parent
                    name             : "av/play_arrow"
                }
            }

            Button {
                elevation : 1
                enabled   : false
                id        : pauseButton
                onClicked : cpp_PlayerView.pauseButtonClicked()

                Icon {
                    anchors.centerIn : parent
                    name             : "av/pause"
                }
            }

            Button {
                elevation : 1
                enabled   : false
                id        : stopButton
                onClicked : cpp_PlayerView.stopButtonClicked()

                Icon {
                    anchors.centerIn : parent
                    name             : "av/stop"
                }
            }

            Button {
                elevation : 1
                enabled   : false
                id        : preprocessButton
                onClicked : cpp_PlayerView.preprocessButtonClicked()

                Icon {
                    anchors.centerIn : parent
                    name             : "action/build"
                }
            }

            View {
                height            : Units.dp(36)
                Layout.fillWidth  : true

                Label {
                    anchors.left           : parent.left
                    anchors.verticalCenter : parent.verticalCenter
                    anchors.margins        : Units.dp(16)
                    color                  : Theme.light.subTextColor
                    elide                  : Text.ElideRight
                    horizontalAlignment    : Qt.AlignHCenter
                    id                     : statusLabel
                    style                  : "subheading"
                }
            }
        }

        View {
            elevation         : 1
            Layout.fillHeight : true
            Layout.fillWidth  : true

            ColumnLayout {
                anchors.fill : parent
                spacing      : 0

                MultiLaneView {
                    id                : multiLaneView
                    Layout.fillHeight : true
                    Layout.fillWidth  : true
                    model             : cpp_MultiLaneViewModel
                }

                View {
                    backgroundColor : "white"
                    elevation : multiLaneView.atYEnd ? 0 : 1
                    height           : Units.dp(68)
                    Layout.fillWidth : true

                    Button {
                        anchors.right          : parent.right
                        anchors.margins        : Units.dp(16)
                        anchors.verticalCenter : parent.verticalCenter
                        textColor              : Theme.accentColor
                        onClicked              : cpp_PlayerView.addLaneButtonClicked()
                        text                   : "ADD LANE"
                    }
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
                        subText        : "Motor Count: " + motorCount
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
                                            showError("No lane in timeline", 
                                                      "To add a behavior to the timeline you first have to add a lane by clicking the plus-button in the toolbar.", 
                                                      "ok")
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
                            idValueLabel.text               = "Identifier: " + id
                            durationValueLabel.text         = "Duration: " + duration + " ms"
                            motorCountValueLabel.text       = "Motor Count: " + motorCount
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
                                    showError("Error while inserting behavior", 
                                              "The timestamp of a behavior has to be a multiple of the sample rate which currently is 100.", 
                                              "ok")
                                }
                                    break;
                                case -2: {
                                    showError("Error while inserting behavior", 
                                              "With the given timestamp, the behavior does overlap with other behaviors already in the timeline.", 
                                              "ok")
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
                    elevation       : detailListView.atYBeginning ? 0 : 1
                    height          : detailTopColumn.implicitHeight + Units.dp(16)
                    id              : detailTopView

                    Column {
                        anchors.bottomMargin : Units.dp(16)
                        anchors.fill         : parent
                        anchors.topMargin    : Units.dp(16)
                        //spacing              : Units.dp(-10)
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
                            id     : idItem

                            action: Icon {
                                anchors.centerIn : parent
                                color            : "white"
                                name             : "action/info_outline"
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
                                name             : "av/av_timer"
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
                                name             : "hardware/developer_board"
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
                    clip             : true
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
                    anchors.bottom  : parent.bottom
                    anchors.left    : parent.left
                    anchors.right   : parent.right
                    backgroundColor : "white"
                    elevation       : detailListView.atYEnd ? 0 : 1
                    height          : Units.dp(68)
                    id              : detailViewButtonView

                    RowLayout {
                        anchors.fill    : parent
                        anchors.margins : Units.dp(16)

                        Button {
                            anchors.right : parent.right
                            id            : addToTimelineButton
                            onClicked     : {
                                if (multiLaneView.numLanes < 1) {
                                    showError("No lane in timeline", 
                                              "To add a behavior to the timeline you first have to add a lane by clicking the plus-button in the toolbar.", 
                                              "ok")
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
                                showError("Error while inserting behavior", 
                                          "The timestamp of a behavior has to be a multiple of the sample rate which currently is 100.", 
                                          "ok")
                            }
                                break;
                            case -2: {
                                showError("Error while inserting behavior", 
                                          "With the given timestamp, the behavior does overlap with other behaviors already in the timeline.", 
                                          "ok")
                            }
                                break;
                            }
                        }
                    }
                }
            }
        }
    }

    function setupPlayerState() {
        switch(cpp_PlayerView.getCurrentPlayerState()) {
            case PlayerState.PLAYER_NOT_READY:
                playButton.enabled       = false
                pauseButton.enabled      = false
                stopButton.enabled       = false
                preprocessButton.enabled = false
                statusLabel.text         = "Player Not Ready"
                break;
            case PlayerState.PLAYER_READY:
                playButton.enabled       = false
                pauseButton.enabled      = false
                stopButton.enabled       = false
                preprocessButton.enabled = true
                statusLabel.text         = "Player Ready"
                break;
            case PlayerState.PLAYER_PREPROCESSING:
                playButton.enabled       = true
                pauseButton.enabled      = false
                stopButton.enabled       = false
                preprocessButton.enabled = false
                statusLabel.text         = "Player Preprocessing"
                break;
            case PlayerState.PLAYER_PREPROCESS_FAILED_EMPTY:
                playButton.enabled       = false
                pauseButton.enabled      = false
                stopButton.enabled       = false
                preprocessButton.enabled = true
                statusLabel.text         = "Player Preprocess Failed: Empty Timeline"
                showError("Error while processing", 
                          "Your timeline is empty. You have to add behaviors to it before processing.", 
                          "ok")
                break;
            case PlayerState.PLAYER_PREPROCESS_FAILED_LOAD_BEHAVIOR:
                playButton.enabled       = false
                pauseButton.enabled      = false
                stopButton.enabled       = false
                preprocessButton.enabled = true
                statusLabel.text         = "Player Preprocess Failed: Loading Behavior Failed"
                showError("Error while processing", 
                          "One or more of the behaviors in your timeline could not be loaded.", 
                          "ok")
                break;
            case PlayerState.PLAYER_PREPROCESS_FAILED_MODE_CONFLICT:
                playButton.enabled       = false
                pauseButton.enabled      = false
                stopButton.enabled       = false
                preprocessButton.enabled = true
                statusLabel.text         = "Player Preprocess Failed: Conflict"
                showError("Error while processing", 
                          "There seems to be a conflict between the behaviors you inserted into the timeline.", 
                          "ok")
                break;
            case PlayerState.PLAYER_PREPROCESS_FAILED_CONTROLLER_STATE_CONFLICT:
                playButton.enabled       = false
                pauseButton.enabled      = false
                stopButton.enabled       = false
                preprocessButton.enabled = true
                statusLabel.text         = "Player Preprocess Failed: Controller State Conflict"
                showError("Error while processing", 
                          "There seems to be a conflict between the states of your controllers.", 
                          "ok")
                break;
            case PlayerState.PLAYER_PREPROCESS_FAILED_SAMPLERATE_CONFLICT:
                playButton.enabled       = false
                pauseButton.enabled      = false
                stopButton.enabled       = false
                preprocessButton.enabled = true
                statusLabel.text         = "Player Preprocess Failed: Samplerate Conflict"
                showError("Error while processing", 
                          "There seems to be a conflict with your samplerate.", 
                          "ok")
                break;
            case PlayerState.PLAYER_PREPROCESS_FAILED_OVERLAPPING:
                playButton.enabled       = false
                pauseButton.enabled      = false
                stopButton.enabled       = false
                preprocessButton.enabled = true
                statusLabel.text         = "Player Preprocess Failed: Overlapping Behaviors"
                showError("Error while processing", 
                          "Two or more of the behaviors in your timeline overlap. Please fix this.", 
                          "ok")
                break;
            case PlayerState.PLAYER_PREPROCESS_FAILED_COMMUNICATION_TIMEOUT:
                playButton.enabled       = false
                pauseButton.enabled      = false
                stopButton.enabled       = false
                preprocessButton.enabled = true
                statusLabel.text         = "Player Preprocess Failed: Communication Timeout"
                showError("Error while processing", 
                          "A timeout occured when trying to communicate with roboy.", 
                          "ok")
                break;
            case PlayerState.PLAYER_PREPROCESS_SUCCEEDED:
                playButton.enabled       = true
                pauseButton.enabled      = false
                stopButton.enabled       = false
                preprocessButton.enabled = true
                statusLabel.text         = "Preprocess Succeeded"
                break;
            case PlayerState.PLAYER_TRAJECTORY_READY:
                playButton.enabled       = true
                pauseButton.enabled      = false
                stopButton.enabled       = false
                preprocessButton.enabled = true
                statusLabel.text         = "Player Trajectory Ready"
                break;
            case PlayerState.PLAYER_PLAYING:
                playButton.enabled       = false
                pauseButton.enabled      = true
                stopButton.enabled       = true
                preprocessButton.enabled = true
                statusLabel.text         = "Player Playing"
                break;
            case PlayerState.PLAYER_PAUSED:
                playButton.enabled       = true
                pauseButton.enabled      = false
                stopButton.enabled       = true
                preprocessButton.enabled = true
                statusLabel.text         = "Player Paused"
                break;
        }
    }
}
