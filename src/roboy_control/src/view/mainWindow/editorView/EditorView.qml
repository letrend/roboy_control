import QtQuick 2.4
import Material 0.2

View {
    View {
        anchors.fill: parent
        anchors.margins: Units.dp(32)
        elevation: 1
        Label {
            anchors.centerIn: parent
            text: "Editor View"
        }
    }
}
