import QtQuick 2.0
import Material 0.2

View {

    property var laneIndex

    elevation: 1
    height: Units.dp(68)
    backgroundColor: Palette.colors["blueGrey"]["500"]

    Canvas {

        anchors.top:   parent.top
        anchors.right: parent.right
        anchors.left:  parent.left
        antialiasing:  false
        height: Units.dp(16)

        onPaint: {

            /* get drawing context */
            var context = getContext("2d")

            /* draw background */
            context.beginPath()

            for (var tickIndex = 0; tickIndex < width+100; tickIndex = tickIndex + 10) {

                /* draw ticks */
                context.beginPath()
                context.lineWidth   = 1
                context.strokeStyle = "white"
                context.textAlign   = "center"
                context.moveTo(Units.dp(tickIndex), Units.dp(0));

                if (tickIndex > 0 && tickIndex < item.width) {
                    if (tickIndex % 50 === 0) {
                        context.lineTo(Units.dp(tickIndex), Units.dp(8))
                    } else {
                        context.lineTo(Units.dp(tickIndex), Units.dp(4))
                    }
                }

                context.stroke();
            }
        }
    }

    Canvas {

        anchors.bottom: parent.bottom
        anchors.right:  parent.right
        anchors.left:   parent.left
        antialiasing:   false
        height: Units.dp(16)

        onPaint: {

            /* get drawing context */
            var context = getContext("2d")

            /* draw background */
            context.beginPath()

            for (var tickIndex = 0; tickIndex < width; tickIndex = tickIndex + 10) {

                /* draw ticks */
                context.beginPath()
                context.lineWidth   = 1
                context.strokeStyle = "white"
                context.textAlign   = "center"
                context.moveTo(Units.dp(tickIndex), height);

                if (tickIndex > 0 && tickIndex < item.width) {
                    if (tickIndex % 50 === 0) {
                        context.lineTo(Units.dp(tickIndex), height-Units.dp(8))
                    } else {
                        context.lineTo(Units.dp(tickIndex), height-Units.dp(4))
                    }
                }

                context.stroke();
            }
        }
    }

    MouseArea {

        anchors.fill: parent
        acceptedButtons: Qt.RightButton

        onClicked: {
            if(Qt.RightButton) {
                laneActionSheet.open()
            }
        }
    }

    BottomActionSheet {

        id: laneActionSheet

        title: "lane " + laneIndex

        actions: [

            Action {
                iconName: "action/delete"
                name: "Delete"

                onTriggered: {
                    parent.removeLane(laneIndex)
                }
            },

            Action {
                iconName: "content/clear"
                name: "Cancel"
            }
        ]
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
