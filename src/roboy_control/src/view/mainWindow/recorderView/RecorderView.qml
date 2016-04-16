import QtQuick 2.4
import QtQuick.Layouts 1.1

import Material 0.2

View {
    Component.onCompleted : {
        setupRecorderState(cpp_RecorderView.getCurrentRecorderState())
    }

    Connections {
        target                      : cpp_RecorderView
        onSignalRecorderStatusUpdated : {
            setupRecorderState(recorderState)
        }
    }

    Dialog {
        id: behaviorNameDialog
        title: "Insert a name for the recording"
        hasActions: true

        TextField {
            id: nameTextField
            width: parent.width
            placeholderText: "NewBehavior"
        }

        onAccepted: {
            cpp_RecorderView.saveRecorderBehavior(nameTextField.text)
        }
    }

    Connections {
        target                         : cpp_RecorderView
        onSignalRecorderResultReceived : {
            behaviorNameDialog.show()
        }
    }

    ColumnLayout {
        anchors.fill    : parent
        anchors.margins : Units.dp(32)
        spacing         : Units.dp(16)

        Row {
            anchors.left    : parent.left
            anchors.margins : Units.dp(0)
            anchors.right   : parent.right
            spacing         : Units.dp(16)

            Button {
                elevation : 1
                enabled   : false
                id        : recordButton
                onClicked : cpp_RecorderView.recordButtonClicked()

                Icon {
                    anchors.centerIn : parent
                    color            : "#C62828"
                    name             : "av/fiber_manual_record"
                }
            }

            Button {
                elevation : 1
                enabled   : false
                id        : pauseButton
                onClicked : cpp_RecorderView.pauseButtonClicked()

                Icon {
                    anchors.centerIn : parent
                    name             : "av/pause"
                }
            }

            Button {
                elevation : 1
                enabled   : false
                id        : stopRecordButton
                onClicked : cpp_RecorderView.stopRecordButtonClicked()

                Icon {
                    anchors.centerIn : parent
                    name             : "av/stop"
                }
            }
        }

        View {
            elevation         : 1
            Layout.fillHeight : true
            Layout.fillWidth  : true
        }
    }

    function setupRecorderState(recorderState) {
        switch(recorderState) {
            case 0: // RECORDER_READY
                recordButton.enabled     = true
                pauseButton.enabled      = false
                stopRecordButton.enabled = false
                break;
            case 1: // RECORDER_RECORDING
                recordButton.enabled     = false
                pauseButton.enabled      = true
                stopRecordButton.enabled = true
                break;
            case 2: // RECORDER_FINISHED_RECORDING
                recordButton.enabled     = true
                pauseButton.enabled      = false
                stopRecordButton.enabled = false
                break;
        }
    }
}