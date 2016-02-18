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

            width               : Units.dp(240)
            Layout.fillHeight   : true

            ListView {

                anchors.fill    : parent

                delegate        : ListItem.Standard {

                    text        : settingName
                    selected    : settingName === selectedSettingName

                    onClicked: {
                        selectedSettingName         = settingName
                        settingsPageLoader.source   = Qt.resolvedUrl(settingsFile)
                    }

                    action: Icon {
                        anchors.centerIn: parent
                        name: settingIcon
                    }
                }

                model   : ListModel {

                    ListElement {
                        settingName:    "General"
                        settingIcon:    "action/settings"
                        settingsFile:   "settingsItems/GeneralSettings.qml"
                    }

                    ListElement {
                        settingName:    "Theme"
                        settingIcon:    "image/color_lens"
                        settingsFile:   "settingsItems/ThemeSettings.qml"
                    }

                    ListElement {
                        settingName:    "Database"
                        settingIcon:    "content/save"
                        settingsFile:   "settingsItems/GeneralSettings.qml"
                    }

                    ListElement {
                        settingName:    "Third party libraries"
                        settingIcon:    "action/settings_input_component"
                        settingsFile:   "settingsItems/ThirdPartySettings.qml"
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

