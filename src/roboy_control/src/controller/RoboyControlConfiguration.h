#ifndef ROBOYCONTROLCONFIGURATION_H
#define ROBOYCONTROLCONFIGURATION_H

#include "DataTypes.h"
#include "LogDefines.h"
#include <QFile>
#include <QProcessEnvironment>
#include <QXmlStreamReader>

#define CONFIG_FILE_NAME "RoboyControlConfig.xml"

class RoboyControlConfiguration {
private:
    QXmlStreamReader m_xmlReader;

    QMap<QString, QString> m_mapModelConfig;
    QList<ROSController> m_listControllerConfig;

public:
    static RoboyControlConfiguration& instance() {
        static RoboyControlConfiguration _instance;
        return _instance;
    }

    RoboyControlConfiguration();
    void update();
    QString getModelConfig(const QString attributeName) const;
    const QList<ROSController> & getControllersConfig() const;

private:
    void readConfig();
    void readModelConfig();
    void readControllersConfig();

};

#endif // ROBOYCONTROLCONFIGURATION_H
