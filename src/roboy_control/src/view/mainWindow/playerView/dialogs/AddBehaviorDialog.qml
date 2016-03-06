import QtQuick 2.0
import QtQuick.Layouts 1.1

import Material 0.2
import Material.ListItems 0.1 as ListItem

Dialog {
    property alias timestampLabel : timestampLabel
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
    positiveButtonText : "Add"
    title              : "Add to timeline"

    RowLayout {
        anchors.left           : parent.left
        anchors.right          : parent.right
        Button {
            anchors.left           : parent.left
            anchors.verticalCenter : parent.verticalCenter
            onClicked              : {timestampLabel.text = (Number(timestampLabel.text) > 0)?(Number(timestampLabel.text) - 100):(timestampLabel.text)}
            width                  : height

            Icon {
                anchors.centerIn : parent
                color            : Theme.accentColor
                name             : "content/remove"
            }
        }

        Label {
            anchors.centerIn : parent
            id               : timestampLabel
            text             : "0"
        }

        Button {
            anchors.right          : parent.right
            anchors.verticalCenter : parent.verticalCenter
            onClicked              : {timestampLabel.text = (Number(timestampLabel.text) < Number.MAX_VALUE)?(Number(timestampLabel.text) + 100):(timestampLabel.text)}
            width                  : height

            Icon {
                anchors.centerIn : parent
                color            : Theme.accentColor
                name             : "content/add"
            }
        }
    }

    MenuField {
        anchors.left  : parent.left
        anchors.right : parent.right
        enabled       : numLanes > 0
        id            : laneSelector
    }
}