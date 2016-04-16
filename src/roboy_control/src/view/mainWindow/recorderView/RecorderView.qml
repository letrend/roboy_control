import QtQuick 2.4
import QtQuick.Layouts 1.1

import Material 0.2

View {
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
                id        : pauseButton
                onClicked : cpp_RecorderView.pauseButtonClicked()

                Icon {
                    anchors.centerIn : parent
                    name             : "av/pause"
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

        View {
            elevation         : 1
            Layout.fillHeight : true
            Layout.fillWidth  : true
        }
    }
}