import QtQuick 2.4
import Material 0.2

import "./playerView"
import "./recorderView"
import "./editorView"

ApplicationWindow {

    id            : roboyControlWindow
    minimumWidth  : 600
    minimumHeight : 500
    title         : "Roboy Control"
    visible       : true

    theme {
        accentColor       : Palette.colors["indigo"]["500"]
        primaryColor      : Palette.colors["blueGrey"]["500"]
        tabHighlightColor : "white"
    }

    initialPage : TabbedPage {
        actions : [
            Action {
                iconName       : "action/settings"
                hoverAnimation : true
                name           : "Settings"
                onTriggered    : pageStack.push(Qt.resolvedUrl("qrc:/mainWindow/settingsView/SettingsView.qml"))
            }
        ]

        title   : "Roboy Control"

        Tab {
            title : "Player"

            PlayerView {

            }
        }

        Tab {
            title : "Editor"

            EditorView {

            }
        }

        Tab {
            title : "Recorder"

            RecorderView {

            }
        }
    } 
}