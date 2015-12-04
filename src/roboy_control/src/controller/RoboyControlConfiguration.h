#ifndef ROBOYCONTROLCONFIGURATION_H
#define ROBOYCONTROLCONFIGURATION_H

#include "DataTypes.h"
#include "LogDefines.h"

#include <QProcessEnvironment>
#include <QXmlStreamReader>
#include <QFile>

#define CONFIG_FILE_NAME "RoboyControlConfig.xml"

class RoboyControlConfiguration
{
private:
    QXmlStreamReader m_xmlReader;

    QMap<QString, QString> m_mapModelConfig;

public:
    static RoboyControlConfiguration& instance() {
        static RoboyControlConfiguration _instance;
        return _instance;
    }


    RoboyControlConfiguration();
    QString getModelConfig(const QString attributeName);

private:
    void readModelConfig();

};

#endif // ROBOYCONTROLCONFIGURATION_H
