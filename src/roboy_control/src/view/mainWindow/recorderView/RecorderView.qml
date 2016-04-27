import QtQuick 2.4
import QtQuick.Layouts 1.1

import Material 0.2
import RecorderState 1.0

View {
    Component.onCompleted : {
        setupRecorderState()
    }

    Connections {
        target                      : cpp_RecorderView
        onSignalRecorderStatusUpdated : {
            setupRecorderState()
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

    function setupRecorderState() {
        switch(cpp_RecorderView.getCurrentRecorderState()) {
            case RecorderState.RECORDER_NOT_READY:
                recordButton.enabled     = false
                pauseButton.enabled      = false
                stopRecordButton.enabled = false
                break;
            case RecorderState.RECORDER_READY:
                recordButton.enabled     = true
                pauseButton.enabled      = true
                stopRecordButton.enabled = true
                break;
            case RecorderState.RECORDER_RECORDING:
                recordButton.enabled     = false
                pauseButton.enabled      = true
                stopRecordButton.enabled = true
                break;
            case RecorderState.RECORDER_FINISHED_RECORDING:
                recordButton.enabled     = true
                pauseButton.enabled      = true
                stopRecordButton.enabled = true
                break;
        }
    }
}