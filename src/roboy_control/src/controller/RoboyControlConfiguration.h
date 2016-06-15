#ifndef ROBOYCONTROLCONFIGURATION_H
#define ROBOYCONTROLCONFIGURATION_H

#include "DataTypes.h"
#include "LogDefines.h"

#include <QFile>
#include <QProcessEnvironment>
#include <QXmlStreamReader>

#define CONFIG_FILE_NAME "RoboyControlConfig.xml"

struct MotorControllerConfig {
    qint32 id;
    ControlMode controlMode;
};

class RoboyControlConfiguration {

private:
    QXmlStreamReader m_xmlReader;

    QMap<QString, QString> m_mapModelConfig;
    QList<MotorControllerConfig> m_listControllerConfig;

public:
    static RoboyControlConfiguration& instance() {
        static RoboyControlConfiguration _instance;
        return _instance;
    }

    void reloadConfig();
    QString getModelConfig(const QString attributeName) const;
    const QList<MotorControllerConfig> & getMotorConfig() const;

private:
    RoboyControlConfiguration() {};

    void readConfig();
    void readModelConfig();
    void readControllersConfig();

};

#endif // ROBOYCONTROLCONFIGURATION_H
