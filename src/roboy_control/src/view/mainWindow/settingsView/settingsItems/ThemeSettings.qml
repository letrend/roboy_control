import QtQuick 2.4

import Material 0.2

View {
    View {
        anchors.centerIn: parent
        elevation   : 1
        width : Units.dp(290)
        height : Units.dp(300)

        MenuField {
            anchors.horizontalCenter : parent.horizontalCenter
            anchors.margins          : Units.dp(16)
            anchors.top              : parent.top
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
}