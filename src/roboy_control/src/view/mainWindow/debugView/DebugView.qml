import QtQuick 2.4
import QtQuick.Layouts 1.1

import Material 0.2
import Material.ListItems 0.1 as ListItem

View {
    elevation         : 1
    Layout.fillHeight : true
    Layout.fillWidth  : true

    ListView {
        anchors.fill : parent
        delegate     : ListItem.Subtitled {
            action : Icon {
                anchors.centerIn : parent
                name             : iconPath
            }
            text        : "Motor " + id
            subText     : motorState
        }
        model        : cpp_MotorStateListModel
    }
}