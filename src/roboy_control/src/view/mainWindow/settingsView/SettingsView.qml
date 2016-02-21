import QtQuick 2.4
import QtQuick.Layouts 1.1

import Material 0.2
import Material.ListItems 0.1 as ListItem

Page {
    property var selectedSettingName

    id      : settingsPage
    title   : "Settings"

    RowLayout {
        anchors.fill    : parent

        PageSidebar {
            Layout.fillHeight   : true
            width               : Units.dp(240)

            ListView {
                anchors.fill    : parent

                delegate        : ListItem.Standard {
                    action : Icon {
                        anchors.centerIn: parent
                        name: settingIcon
                    }

                    onClicked : {
                        selectedSettingName         = settingName
                        settingsPageLoader.source   = Qt.resolvedUrl(settingsFile)
                    }

                    selected    : settingName === selectedSettingName
                    text        : settingName
                }

                model           : ListModel {
                    ListElement {
                        settingsFile : "qrc:/mainWindow/settingsView/settingsItems/GeneralSettings.qml"
                        settingIcon  : "action/settings"
                        settingName  : "General"
                    }

                    ListElement {
                        settingsFile : "qrc:/mainWindow/settingsView/settingsItems/ThemeSettings.qml"
                        settingIcon  : "image/color_lens"
                        settingName  : "Theme"
                    }

                    ListElement {
                        settingsFile : "qrc:/mainWindow/settingsView/settingsItems/GeneralSettings.qml"
                        settingIcon  : "content/save"
                        settingName  : "Database"
                    }

                    ListElement {
                        settingsFile : "qrc:/mainWindow/settingsView/settingsItems/ThirdPartySettings.qml"
                        settingIcon  : "action/settings_input_component"
                        settingName  : "Third party libraries"
                    }
                }
            }
        }

        Loader {
            id                  : settingsPageLoader
            Layout.fillHeight   : true
            Layout.fillWidth    : true
            Layout.margins      : Units.dp(32)
        }
    }
}

