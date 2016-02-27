import QtQuick 2.0

import Material 0.2

View {
    property var laneIndex

    elevation       : 1
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
        acceptedButtons: Qt.RightButton
        anchors.fill: parent

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
                    var laneBackground = parent
                    var laneScrollView = laneBackground.parent
                    var helperView1    = laneScrollView.parent
                    var helperView2    = helperView1.parent
                    var helperView3    = helperView2.parent
                    var helperView4    = helperView3.parent
                    var helperView5    = helperView4.parent
                    var helperView6    = helperView5.parent
                    var multiLaneView  = helperView6.parent
                    multiLaneView.removeLane(laneIndex)
                }
            },

            Action {
                iconName : "content/clear"
                name     : "Cancel"
            }
        ]
        
        id     : laneActionSheet
        title  : "lane " + laneIndex
    }

    function removeItem(itemIndex) {
        var laneBackground = parent
        var laneScrollView = laneBackground.parent
        var helperView1    = laneScrollView.parent
        var helperView2    = helperView1.parent
        var helperView3    = helperView2.parent
        var helperView4    = helperView3.parent
        var helperView5    = helperView4.parent
        var helperView6    = helperView5.parent
        var multiLaneView  = helperView6.parent
        multiLaneView.removeItem(laneIndex, itemIndex)
    }
}
