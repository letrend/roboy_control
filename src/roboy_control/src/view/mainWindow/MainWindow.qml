import QtQuick 2.4
import Material 0.2
import Material.ListItems 0.1 as ListItem

import "./playerView"
import "./recorderView"
import "./editorView"

ApplicationWindow {

    id: roboyControlWindow
    title: "Roboy Control"
    minimumWidth: 600
    minimumHeight: 500

    /* necessary when loading the window from C++ */
    visible: true

    theme {

        primaryColor: Palette.colors["blueGrey"]["500"]
        accentColor: Palette.colors["indigo"]["500"]
        tabHighlightColor: "white"

    }

    initialPage: TabbedPage {

        title: "Roboy Control"

        actions: [

            Action {
                iconName: "action/account_circle"
                name: "Colors"
                hoverAnimation: true
                onTriggered: colorPicker.show()
            },

            Action {
                iconName: "action/settings"
                name: "Settings"
                hoverAnimation: true
                onTriggered: pageStack.push(Qt.resolvedUrl("./settingsView/SettingsView.qml"))
            }
        ]

        Tab {

            title: "Player"

            PlayerView {

            }
        }

        Tab {

            title: "Editor"

            EditorView {

            }
        }

        Tab {

            title: "Recorder"

            RecorderView {

            }
        }
    } 
}