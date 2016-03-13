import QtQuick 2.4
import QtQuick.Layouts 1.1

import Material 0.2
import Material.ListItems 0.1 as ListItem

Page {
    property var selectedBehaviorIndex

    onSelectedBehaviorIndexChanged : {
    	cpp_EditorView.setSelectedBehaviorIndex(selectedBehaviorIndex);
    }

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
                        anchors.centerIn : parent
                        name             : iconPath
                    }

                    Layout.margins : 0
                    onClicked      : {
	                	selectedBehaviorIndex         = index
	                	behaviorNameTextfield.text    = title
	                	behaviorNameTextfield.enabled = true
	                	identifierLabel.text          = id
	                	durationLabel.text            = "duration: " + duration + " ms"
	                	motorCountLabel.text          = "motor count: " + motorCount 
	                	motorListView.model           = motorInfo
	                }
                    selected       : index === selectedBehaviorIndex
                    subText        : "motor count: " + motorCount
                    text           : title
            	}
                model       : cpp_EVBehaviorListModel
            }
        }

        View {
        	Layout.fillHeight : true 
        	Layout.fillWidth  : true

	        View {
	        	anchors.fill     : parent
	        	anchors.margins  : Units.dp(32)
		        elevation        : 1
		        radius           : Units.dp(2)

		        View {
		        	anchors.left    : parent.left
		        	anchors.right   : parent.right
		        	backgroundColor : Theme.primaryColor
		        	height          : topColumn.implicitHeight + Units.dp(16)
		        	id              : topView

			        Column {
			        	anchors.bottomMargin : Units.dp(16) 
			        	anchors.fill         : parent
			            anchors.topMargin    : Units.dp(16)
			            id                   : topColumn

			            Label {
			            	anchors.left    : parent.left
			            	anchors.right   : parent.right
			            	anchors.margins : Units.dp(16)
			            	color           : "white"
			                id              : titleLabel
			                style           : "title"
			                text            : "Edit behavior"
			            }

			            Item {
			                Layout.fillWidth       : true
			                Layout.preferredHeight : Units.dp(8)
			            }

			            ListItem.Standard {
			                action : Icon {
			                    anchors.centerIn : parent
			                    color            : "white"
			                    name             : "action/accessibility"
			                }

			                content : TextField {
			                    anchors.centerIn  : parent
			                    color             : "white"
			                    enabled           : false
			                    id                : behaviorNameTextfield
			                    onEditingFinished : { cpp_EditorView.updateBehaviorName(text) }
			                    placeholderText   : "behavior name"
			                    textColor         : "white"
			                    width             : parent.width
			                }
			            }

			            ListItem.Standard {
			                action : Icon {
			                    anchors.centerIn : parent
			                    color            : "white"
			                    name             : "action/info_outline"
			                }

			                content : Label {
			                	anchors.centerIn : parent
			                    color            : "white"
			                    font.pixelSize   : Units.dp(16)
			                    id               : identifierLabel
			                   	text             : "-"
			                    width            : parent.width
			                }
			            }

			            ListItem.Standard {
			                action : Icon {
			                    anchors.centerIn : parent
			                    color            : "white"
			                    name             : "av/av_timer"
			                }

			                content : Label {
			                	anchors.centerIn : parent
			                    color            : "white"
			                    font.pixelSize   : Units.dp(16)
			                    id               : durationLabel
			                   	text             : "-"
			                    width            : parent.width
			                }
			            }

			            ListItem.Standard {
			                action : Icon {
			                    anchors.centerIn : parent
			                    color            : "white"
			                    name             : "av/album"
			                }

			                content : Label {
			                	anchors.centerIn : parent
			                    color            : "white"
			                    font.pixelSize   : Units.dp(16)
			                    id               : motorCountLabel
			                   	text             : "-"
			                    width            : parent.width
			                }
			            }
	                }
	            }

                ListView {
                    anchors.bottom : seperator.top
                    anchors.left   : parent.left
                    anchors.right  : parent.right
                    anchors.top    : topView.bottom
                    height         : Units.dp(200)
                    id             : motorListView

                    delegate       : ListItem.Standard {
                        action                   : Icon {
                            anchors.centerIn : parent
                            name             : "av/album"
                        }
                        itemLabel.font.pixelSize : Units.dp(16)
                        text                     : modelData
                    }
                }

	            Item {
	            	anchors.bottom : buttonLayout.top
	                anchors.left   : parent.left
	                anchors.right  : parent.right
	                height         : Units.dp(8)
	                id             : seperator
	            }

	            RowLayout {
	            	anchors.bottom   : parent.bottom
	            	anchors.margins  : Units.dp(16)
	            	anchors.right    : parent.right
	            	id               : buttonLayout
	                Layout.alignment : Qt.AlignRight
	                spacing          : Units.dp(8)

	                Button {
	                    text      : "Cancel"
	                    textColor : Theme.accentColor
	                }

	                Button {
	                	onClicked : {
	                		cpp_EditorView.saveButtonClicked();
	                		snackbar.open("Behavior updated")
	                	}
	                    text      : "Save"
	                    textColor : Theme.accentColor
	                }
	            }
	        }
        }
    }

    Snackbar {
        id: snackbar
    }
}