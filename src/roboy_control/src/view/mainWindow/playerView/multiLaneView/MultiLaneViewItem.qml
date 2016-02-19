import QtQuick 2.0
import QtQuick.Layouts 1.1
import Material 0.2

View {

    property int    laneIndex : 0
    property int    itemIndex : 0
    property string behaviorName : ""
    property int    motorCount : 0
    property string iconName : ""

    elevation: 1
    backgroundColor: Theme.accentColor
    height: Units.dp(36)

    RowLayout {

        anchors.fill: parent
        spacing: Units.dp(24)

        Icon {

            anchors.verticalCenter: parent.verticalCenter
            Layout.leftMargin: Units.dp(24)
            name: iconName
            color: Palette.colors["white"]["500"]

        }

        ColumnLayout {

            spacing: Units.dp(0)

            Label {

                Layout.fillWidth: true
                font.family: "Roboto"
                style: "subheading"
                color: Palette.colors["white"]["500"]
                text: behaviorName
            }
        }
    }

    MouseArea {

        anchors.fill: parent
        acceptedButtons: Qt.RightButton

        onClicked: {
            if (Qt.RightButton) {
                itemActionSheet.open()
            }
        }
    }

    BottomActionSheet {

        id: itemActionSheet

        title: behaviorName

        actions: [

            Action {
                iconName: "content/create"
                name: "Edit"
            },

            Action {
                iconName: "action/delete"
                name: "Delete"

                onTriggered: {
                    parent.removeItem(itemIndex)
                }
            },

            Action {
                iconName: "content/clear"
                name: "Cancel"
            }
        ]
    }
}
