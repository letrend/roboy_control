import QtQuick 2.0

import Material 0.2

Dialog {
    property alias timestampTextField : timestampTextField
    property alias laneSelector : laneSelector

    property int numLanes : 0

    onNumLanesChanged : {
        var laneSelectorModel = []
        for (var i = 0; i < numLanes; i++) {
            laneSelectorModel.push("lane " + (i+1))
        }
        laneSelector.model = laneSelectorModel
    }

    id                 : addBehaviorDialog
    positiveButtonText : "Insert"
    title              : "Insert Behavior"

    TextField {
        anchors.left    : parent.left
        anchors.right   : parent.right
        floatingLabel   : true
        id              : timestampTextField
        placeholderText : "timestamp"
        validator       : IntValidator{}
    }

    MenuField {
        anchors.left  : parent.left
        anchors.right : parent.right
        id            : laneSelector
        model         : numLanes
    }
}