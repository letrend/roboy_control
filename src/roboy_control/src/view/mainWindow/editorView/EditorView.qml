import QtQuick 2.4
import QtQuick.Layouts 1.1

import Material 0.2
import Material.ListItems 0.1 as ListItem

Page {
/*
    property var selectedBehaviorIndex

    id      : editorView
    title   : "Settings"

    RowLayout {
        anchors.fill    : parent

        PageSidebar {
            Layout.fillHeight   : true
            width               : Units.dp(240)

            ListView {
                anchors.fill : parent
                delegate     : ListItem.Subtitled {
                    action         : Icon {
                        anchors.centerIn: parent
                        name: iconPath
                    }

                    Layout.margins : 0
                    selected       : index === selectedBehaviorIndex
                    subText        : "motor count " + motorCount
                    text           : title
            	}
                model          : cpp_BehaviorListModel
            }
        }

        View {
	        width: Units.dp(350)
	        height: column.implicitHeight + Units.dp(32)

	        elevation: 1
	        radius: Units.dp(2)

	        ColumnLayout {
	            id: column

	            anchors {
	                fill: parent
	                topMargin: Units.dp(16)
	                bottomMargin: Units.dp(16)
	            }

	            Label {
	                id: titleLabel

	                anchors {
	                    left: parent.left
	                    right: parent.right
	                    margins: Units.dp(16)
	                }

	                style: "title"
	                text: "Edit behavior"
	            }

	            Item {
	                Layout.fillWidth: true
	                Layout.preferredHeight: Units.dp(8)
	            }

	            ListItem.Standard {
	                action: Icon {
	                    anchors.centerIn: parent
	                    name: "action/account_circle"
	                }

	                content: TextField {
	                    anchors.centerIn: parent
	                    width: parent.width

	                    text: "Alex Nelson"
	                }
	            }

	            ListItem.Standard {
	                action: Icon {
	                    anchors.centerIn: parent
	                    name: "maps/place"
	                }

	                content: TextField {
	                    anchors.centerIn: parent
	                    width: parent.width

	                    text: "100 Main Street"
	                }
	            }

	            ListItem.Standard {
	                action: Item {}

	                content: RowLayout {
	                    anchors.centerIn: parent
	                    width: parent.width

	                    TextField {
	                        Layout.alignment: Qt.AlignVCenter
	                        Layout.preferredWidth: 0.4 * parent.width

	                        text: "New York"
	                    }

	                    MenuField {
	                        Layout.alignment: Qt.AlignVCenter
	                        Layout.preferredWidth: 0.2 * parent.width

	                        model: ["NY", "NC", "ND"]
	                    }

	                    TextField {
	                        Layout.alignment: Qt.AlignVCenter
	                        Layout.preferredWidth: 0.3 * parent.width

	                        text: "10011"
	                    }
	                }
	            }

	            ListItem.Standard {
	                action: Icon {
	                    anchors.centerIn: parent
	                    name: "communication/email"
	                }

	                content: TextField {
	                    anchors.centerIn: parent
	                    width: parent.width

	                    placeholderText: "Email"
	                }
	            }

	            Item {
	                Layout.fillWidth: true
	                Layout.preferredHeight: Units.dp(8)
	            }

	            RowLayout {
	                Layout.alignment: Qt.AlignRight
	                spacing: Units.dp(8)

	                anchors {
	                    right: parent.right
	                    margins: Units.dp(16)
	                }

	                Button {
	                    text: "Cancel"
	                    textColor: Theme.primaryColor
	                }

	                Button {
	                    text: "Save"
	                    textColor: Theme.primaryColor
	                }
	            }
	        }
        }
    }*/
}