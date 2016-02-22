import QtQuick 2.4
import QtQuick.Layouts 1.1

import Material 0.2

View {
    anchors.fill: parent

    View {
        anchors.horizontalCenter : parent.horizontalCenter
        anchors.top              : parent.top
        id                       : colorShowCaseView
        height                   : Units.dp(80)
        width                    : Units.dp(258)

        Rectangle {
            anchors.bottom : parent.bottom
            anchors.left   : parent.left
            anchors.top    : parent.top
            border.color   : Theme.alpha("#000", 0.26)
            border.width   : Theme.primaryColor === "white" ? Units.dp(2) : 0
            color          : Theme.primaryColor
            height         : Units.dp(80)
            id             : primaryColorRectangle
            radius         : Units.dp(2)
            width          : Units.dp(80)

            Label {
                anchors.centerIn : parent
                color            : "white"
                text             : "Primary"
            }
        }

        Rectangle {
            anchors.bottom : parent.bottom
            anchors.right  : parent.right
            anchors.top    : parent.top
            border.color   : Theme.alpha("#000", 0.26)
            border.width   : Theme.accentColor === "white" ? Units.dp(2) : 0
            color          : Theme.accentColor
            height         : Units.dp(80)
            id             : accentolorRectangle
            radius         : Units.dp(2)
            width          : Units.dp(80)

            Label {
                anchors.centerIn : parent
                color            : "white"
                text             : "Accent"
            }
        }
    }

    MenuField {
        anchors.horizontalCenter : parent.horizontalCenter
        anchors.margins          : Units.dp(16)
        anchors.top              : colorShowCaseView.bottom
        id                       : colorSelector
        model                    : ["Primary color", "Accent color"]
        width                    : Units.dp(258)
    }

    Grid {
        anchors.horizontalCenter : parent.horizontalCenter
        anchors.margins          : Units.dp(16)
        anchors.top              : colorSelector.bottom
        columns                  : 7
        spacing                  : Units.dp(8)

        Repeater {
            model: [
                "red",        "pink",      "purple",    "deepPurple", "indigo",
                "blue",       "lightBlue", "cyan",     "teal",       "green",
                "lightGreen", "lime",      "yellow",   "amber",      "orange",
                "deepOrange", "grey",      "blueGrey", "brown",      "black",
                "white"
            ]

            Rectangle {
                border.color : Theme.alpha("#000", 0.26)
                border.width : modelData === "white" ? Units.dp(2) : 0
                color        : Palette.colors[modelData]["500"]
                height       : Units.dp(30)
                radius       : Units.dp(2)
                width        : Units.dp(30)

                Ink {
                    anchors.fill : parent

                    onPressed: {
                        switch(colorSelector.selectedIndex) {
                        case 0:
                            theme.primaryColor = parent.color
                            break;
                        case 1:
                            theme.accentColor = parent.color
                            break;
                        }
                    }
                }
            }
        }
    }
}
