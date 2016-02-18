import QtQuick 2.4
import QtQuick.Controls 1.3
import QtQuick.Window 2.2
import QtQuick.Dialogs 1.2
import QtQuick.Layouts 1.1
import Qt.labs.folderlistmodel 2.0

ListView {

    anchors.fill: parent

    FolderListModel {
        id: folderModel
        folder: "/Users/matthiaslehner/Desktop"
        showDirs: true
        showDirsFirst: true
        //nameFilters: ["*.*"]
    }

    Component {
        id: fileDelegate
        Text { text: fileName }
    }

    model: folderModel
    delegate: fileDelegate
}
