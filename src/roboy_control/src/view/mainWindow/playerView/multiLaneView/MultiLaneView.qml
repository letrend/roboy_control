import QtQuick 2.0
import QtQuick.Controls 1.4
import QtQuick.Layouts 1.1
import Material 0.2

View {

    id: multiLaneView

    property var model

    onModelChanged: {
        updateView()
    }

    property int scaleFactor: 1

    onScaleFactorChanged: {
        updateView()
    }

    Connections {

        target: model
        onDataChanged: {
            updateView()
        }
    }

    ScrollView {

        anchors.fill: parent

        View {

            id: laneBackground
            width:  Units.dp(100 + 2*16)
            height: Units.dp(68  + 2*16)

        }
    }

    function updateView () {

        /* disposing the current view hierachy */

        var numChildren = laneBackground.children.length

        for (var i = 0; i < numChildren; i++) {
            laneBackground.children[i].destroy()
        }

        /* rebuilding the view hierachy with the new data */

        var numLanes = model.laneCount();

        for (var laneIndex = 0; laneIndex < numLanes; laneIndex++) {

            var laneComponent = Qt.createComponent("MultiLaneViewLane.qml")

            if (laneComponent.status === Component.Ready) {

                var lane = laneComponent.createObject(laneBackground)

                lane.laneIndex       = laneIndex
                lane.y               = Units.dp(laneIndex*68+(laneIndex+1)*16)
                lane.anchors.left    = laneBackground.left
                lane.anchors.right   = laneBackground.right
                lane.anchors.margins = Units.dp(16)

                for (var itemIndex = 0; itemIndex < model.itemCount(laneIndex); itemIndex++) {

                    var itemComponent = Qt.createComponent("MultiLaneViewItem.qml")

                    if (itemComponent.status === Component.Ready) {

                        var item              = itemComponent.createObject(lane)

                        item.laneIndex        = laneIndex
                        item.itemIndex        = itemIndex
                        item.behaviorName     = model.data(laneIndex, itemIndex, 0x0000)                // Qt::DisplayRole
                        item.iconName         = model.data(laneIndex, itemIndex, 0x0001)                            // Qt::DecorationRole
                        item.x                = Units.dp(model.data(laneIndex, itemIndex, 0x0100) / scaleFactor)    // Qt::UserRole
                        item.y                = Units.dp(16)
                        item.width            = Units.dp(model.data(laneIndex, itemIndex, 0x0101) / scaleFactor)    // Qt::UserRole + 1
                        laneBackground.height = Units.dp(numLanes*68+(numLanes+1)*16)
                        laneBackground.width  = (item.x + item.width + Units.dp(100 + 2*16) > laneBackground.width) / scaleFactor ?
                        (item.x + item.width  + Units.dp(100 + 2*16)) / scaleFactor : (laneBackground.width)
                    }
                }
            }
        }
    }

    function removeLane(laneIndex) {
        console.log("remove lane")
    }

    function removeItem(laneIndex, itemIndex) {
        console.log("remove item")
    }
}
