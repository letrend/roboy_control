import QtQuick 2.4
import QtQuick.Layouts 1.1

import Material 0.2

View {
    View {
        anchors.centerIn : parent
        elevation        : 1
        width            : Units.dp(282)
        height           : mainColumn.implicitHeight + 2*Units.dp(16)

        Column {
            anchors.fill    : parent
            anchors.margins : Units.dp(16)
            id              : mainColumn
            spacing         : Units.dp(16)

            MenuField {
                anchors.left             : parent.left
                anchors.right            : parent.right
                id                       : colorSelector
                model                    : ["Primary color", "Accent color"]
            }

            Grid {
                anchors.left             : parent.left
                anchors.right            : parent.right
                columns                  : 7
                spacing                  : Units.dp(8)

                Repeater {
                    model: [
                        "red",      "pink",         "purple",   "deepPurple",   "indigo",   "blue",     "lightBlue", 
                        "cyan",     "teal",         "green",    "lightGreen",   "lime",     "yellow",   "amber",      
                        "orange",   "deepOrange",   "grey",     "blueGrey",     "brown",    "black",    "white"
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
                                    theme.accentColor  = parent.color
                                    break;
                                }
                            }
                        }
                    }
                }
            }
        }
    }
}