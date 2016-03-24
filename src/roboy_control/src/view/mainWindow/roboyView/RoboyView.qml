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
                id        : initializeButton
                onClicked : cpp_RoboyView.initalizeButtonClicked()

                Icon {
                    anchors.centerIn : parent
                    name             : "action/autorenew"
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