import QtQuick 2.0

import Material 0.2

Dialog {
    id                 : addBehaviorDialog
    positiveButtonText : "Insert"
    title              : "Insert Behavior"

    TextField {
        anchors.left    : parent.left
        anchors.right   : parent.right
        floatingLabel   : true
        id              : timeStampTextField
        placeholderText : "timestamp"
        validator       : IntValidator{}
    }

    MenuField {
        anchors.left  : parent.left
        anchors.right : parent.right
        id            : laneSelector
        model         : []
    }

    onAccepted: {

    }
}
