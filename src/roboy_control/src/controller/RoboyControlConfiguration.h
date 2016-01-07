#ifndef ROBOYCONTROLCONFIGURATION_H
#define ROBOYCONTROLCONFIGURATION_H

#include "DataTypes.h"
#include "LogDefines.h"

#include <QFile>
#include <QProcessEnvironment>
#include <QXmlStreamReader>

#define CONFIG_FILE_NAME "RoboyControlConfig.xml"

class RoboyControlConfiguration
{
private:
    QXmlStreamReader m_xmlReader;

    QMap<QString, QString> m_mapModelConfig;
    QList<qint8> m_listControllerConfig;

public:
    static RoboyControlConfiguration& instance() {
        static RoboyControlConfiguration _instance;
        return _instance;
    }

    RoboyControlConfiguration();
    QString getModelConfig(const QString attributeName) const;
    const QList<qint8> & getControllersConfig() const;

private:
    void readModelConfig();
    void readControllersConfig();

};

#endif // ROBOYCONTROLCONFIGURATION_H
