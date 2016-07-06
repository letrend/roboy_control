import QtQuick 2.4
import QtQuick.Layouts 1.1

import Material 0.2
import Material.ListItems 0.1 as ListItem

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
                onClicked : cpp_DebugView.initalizeButtonClicked()

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

            ListView {
                anchors.fill : parent
                delegate     : ListItem.Subtitled {
                    action : Icon {
                        anchors.centerIn : parent
                        name             : iconPath
                    }
                    text        : "Motor " + id
                    subText     : motorState
                }
                model        : cpp_MotorStateListModel
            }
        }
    }
}