import QtQuick 2.0

import Material 0.2

View {
    property var laneIndex
    property var multiLaneView

    elevation       : 1
    id              : multiLaneViewLane
    backgroundColor : Palette.colors["blueGrey"]["500"]
    height          : Units.dp(68)

    Canvas {
        anchors.left  : parent.left
        anchors.right : parent.right
        anchors.top   : parent.top
        antialiasing  : false
        height        : Units.dp(40)

        onPaint: {
            var context = getContext("2d")

            for (var tickIndex = 0; tickIndex < width; tickIndex = tickIndex + 10) {

                context.beginPath()
                context.lineWidth   = 1
                context.strokeStyle = "white"
                context.textAlign   = "center"
                context.moveTo(tickIndex, Units.dp(0));

                if (tickIndex > 0 && tickIndex < width) {
                    if (tickIndex % 50 === 0) {
                        context.lineTo(tickIndex, Units.dp(8))
                    } else {
                        context.lineTo(tickIndex, Units.dp(4))
                    }
                }

                context.stroke();
            }
        }
    }

    Canvas {
        anchors.bottom : parent.bottom
        anchors.left   : parent.left
        anchors.right  : parent.right
        antialiasing   : false
        height         : Units.dp(16)

        onPaint: {
            var context = getContext("2d")

            for (var tickIndex = 0; tickIndex <= width; tickIndex = tickIndex + 10) {

                context.beginPath()
                context.lineWidth   = 1
                context.strokeStyle = "white"
                context.textAlign   = "center"
                context.moveTo(tickIndex, height);

                if (tickIndex > 0 && tickIndex < width) {
                    if (tickIndex % 50 === 0) {
                        context.lineTo(tickIndex, height-Units.dp(8))
                    } else {
                        context.lineTo(tickIndex, height-Units.dp(4))
                    }
                }
                context.stroke();
            }
        }
    }

    MouseArea {
        acceptedButtons : Qt.RightButton
        anchors.fill    : parent

        onClicked: {
            if(Qt.RightButton) {
                laneActionSheet.open()
            }
        }
    }

    BottomActionSheet {
        actions : [
            Action {
                iconName    : "action/delete"
                name        : "Delete"

                onTriggered : {
                    if (multiLaneView.model.itemCount(laneIndex) > 0) {
                        laneNotEmptyDialog.show()
                    } else {
                        multiLaneView.removeLane(laneIndex)
                    }
                }
            },

            Action {
                iconName : "content/clear"
                name     : "Cancel"
            }
        ]
        
        id     : laneActionSheet
        title  : "lane " + (laneIndex + 1)
    }

     Dialog {
        hasActions         : true
        id                 : laneNotEmptyDialog
        negativeButtonText : "cancel"
        onAccepted         : multiLaneView.removeLane(multiLaneViewLane.laneIndex)
        positiveButtonText : "proceed"   
        text               : "The lane you want to delete is not empty. If you proceed, the lane and all contained behaviors will be removed."
        title              : "Lane not empty"
        width              : Units.dp(300) 
    }

    function removeItem(itemIndex) {
        multiLaneView.removeItem(laneIndex, itemIndex)
    }
}
