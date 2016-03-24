import QtQuick 2.0
import QtQuick.Layouts 1.1

import Material 0.2

View {
    property int    laneIndex    : 0
    property int    itemIndex    : 0
    
    property string behaviorName : ""
    property int    motorCount   : 0
    property string iconName     : ""

    backgroundColor : Theme.accentColor
    height          : Units.dp(36)

    RowLayout {
        anchors.fill: parent
        spacing: Units.dp(24)

        Icon {
            anchors.verticalCenter : parent.verticalCenter
            color                  : Palette.colors["white"]["500"]
            Layout.leftMargin      : Units.dp(24)
            name                   : iconName
        }

        ColumnLayout {
            spacing : Units.dp(0)

            Label {
                color            : Palette.colors["white"]["500"]
                font.family      : "Roboto"
                Layout.fillWidth : true
                style            : "subheading"
                text             : behaviorName
            }
        }
    }

    MouseArea {
        acceptedButtons : Qt.RightButton
        anchors.fill    : parent

        onClicked: {
            if (Qt.RightButton) {
                itemActionSheet.open()
            }
        }
    }

    BottomActionSheet {
        id      : itemActionSheet
        title   : behaviorName

        actions : [
            Action {
                iconName    : "action/delete"
                name        : "Delete"

                onTriggered : {
                    parent.removeItem(itemIndex)
                }
            },

            Action {
                iconName : "content/clear"
                name     : "Cancel"
            }
        ]
    }
}
