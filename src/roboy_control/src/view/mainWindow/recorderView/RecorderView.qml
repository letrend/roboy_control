import QtQuick 2.4

import Material 0.2

View {
/*
    View {
        anchors.fill    : parent
        anchors.margins : Units.dp(32)
        elevation       : 1

        Label {
            anchors.centerIn : parent
            text             : "Recorder View"
        }
    }*/

    Row {
        anchors.left    : parent.left
        anchors.margins : Units.dp(0)
        anchors.right   : parent.right
        spacing         : Units.dp(16)

        Button {
            elevation : 1
            id        : recordButton
            onClicked : cpp_RecorderView.recordButtonClicked()

            Icon {
                anchors.centerIn : parent
                name             : "av/play_arrow"
            }
        }

        Button {
            elevation : 1
            id        : stopRecordButton
            onClicked : cpp_RecorderView.stopRecordButtonClicked()

            Icon {
                anchors.centerIn : parent
                name             : "av/stop"
            }
        }

    }
}