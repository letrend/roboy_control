import QtQuick 2.4
import QtQuick.Layouts 1.1
import Material 0.2

View {

    anchors.fill: parent

    View {
        id: colorShowCaseView
        anchors.top: parent.top
        anchors.horizontalCenter: parent.horizontalCenter
        width: Units.dp(258)
        height: Units.dp(80)

        Rectangle {
            id: primaryColorRectangle
            anchors.top: parent.top
            anchors.bottom: parent.bottom
            anchors.left: parent.left
            width: Units.dp(80)
            height: Units.dp(80)
            radius: Units.dp(2)
            color: Theme.primaryColor
            border.width: Theme.primaryColor === "white" ? Units.dp(2) : 0
            border.color: Theme.alpha("#000", 0.26)

            Label {
                anchors.centerIn: parent
                text: "Primary"
                color: "white"
            }
        }

        Rectangle {
            id: accentColorRectangle
            anchors.top: parent.top
            anchors.bottom: parent.bottom
            anchors.right: parent.right
            width: Units.dp(80)
            height: Units.dp(80)
            radius: Units.dp(2)
            color: Theme.accentColor
            border.width: Theme.accentColor === "white" ? Units.dp(2) : 0
            border.color: Theme.alpha("#000", 0.26)

            Label {
                anchors.centerIn: parent
                text: "Accent"
                color: "white"
            }
        }
    }

    MenuField {
        id: colorSelector
        anchors.top: colorShowCaseView.bottom
        anchors.horizontalCenter: parent.horizontalCenter
        anchors.margins: Units.dp(16)
        width: Units.dp(258)
        model: ["Primary color", "Accent color"]
    }

    Grid {

        anchors.top: colorSelector.bottom
        anchors.horizontalCenter: parent.horizontalCenter
        anchors.margins: Units.dp(16)
        columns: 7
        spacing: Units.dp(8)

        Repeater {

            model: [
                "red", "pink", "purple", "deepPurple", "indigo",
                "blue", "lightBlue", "cyan", "teal", "green",
                "lightGreen", "lime", "yellow", "amber", "orange",
                "deepOrange", "grey", "blueGrey", "brown", "black",
                "white"
            ]

            Rectangle {
                width: Units.dp(30)
                height: Units.dp(30)
                radius: Units.dp(2)
                color: Palette.colors[modelData]["500"]
                border.width: modelData === "white" ? Units.dp(2) : 0
                border.color: Theme.alpha("#000", 0.26)

                Ink {

                    anchors.fill: parent

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
