import QtQuick 2.0
import Material 0.2

Dialog {

    id: addBehaviorDialog
    title: "Insert Behavior"
    positiveButtonText: "Insert"

    TextField {

        id: timeStampTextField
        anchors.left: parent.left
        anchors.right: parent.right
        placeholderText: "timestamp"
        floatingLabel: true
        validator: IntValidator{}
    }

    MenuField {

        id: laneSelector
        model: []
        anchors.left: parent.left
        anchors.right: parent.right
    }

    onAccepted: {

    }
}
