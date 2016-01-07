#include "RoboyControlConfiguration.h"

RoboyControlConfiguration::RoboyControlConfiguration() {
    QString roboyControlHome = QProcessEnvironment::systemEnvironment().value("ROBOY_CONTROL_HOME");
    if (roboyControlHome == "") {
        CONFIG_DBG << " - ERROR: Environment Variable not set.";
        CONFIG_DBG << "Set the environment variable ROBOY_CONTROL_HOME to the installation"
                      "directory of RoboyControl. Add the line\n"
                      "\texport ROBOY_CONTROL_HOME=<path_to_roboy_control>"
                      "to your ~/.bash_rc file.";
        exit(0);
    }

    QString filename = roboyControlHome + "/etc/" + CONFIG_FILE_NAME;
    QFile configFile(filename);

    if( !configFile.open(QFile::ReadOnly | QFile::Text) ) {
        CONFIG_DBG << " - ERROR opening file: " << filename;
        CONFIG_DBG << "Make sure your environment is set up correctly and the configuration file"
                      "is placed in /etc of your <roboy_control_install_dir>.";
    }

    CONFIG_DBG << "INFO open successful: " << filename;

    m_xmlReader.setDevice(&configFile);

    m_xmlReader.readNextStartElement();
    if (m_xmlReader.name() == "RoboyControlConfiguration") {
        while(m_xmlReader.readNextStartElement()) {
            if (m_xmlReader.name() == "DataModel") {
                CONFIG_DBG << "Read Model Config";
                readModelConfig();
            } else if (m_xmlReader.name() == "Controllers") {
                CONFIG_DBG << "Read Controllers Config";
                readControllersConfig();
            } else {
                m_xmlReader.skipCurrentElement();
            }
        }
    } else {
        CONFIG_DBG << " - ERROR found invalid config file.";
    }
}

void RoboyControlConfiguration::readModelConfig() {
    for (QXmlStreamAttribute attribute : m_xmlReader.attributes()) {
        m_mapModelConfig.insert(attribute.name().toString(), attribute.value().toString());
    }
    m_xmlReader.readElementText();
}

void RoboyControlConfiguration::readControllersConfig() {
    while(m_xmlReader.readNextStartElement()) {
        if(m_xmlReader.name() == "Controller") {
            qint8 id = m_xmlReader.attributes().value("id").toString().toShort();
            bool value = false;
            m_xmlReader.readElementText() == "true" ? value = true : value = false;
            if (value)
                m_listControllerConfig.append(id);
            CONFIG_DBG << "Parsed Controller: " << id << " Value: " << value;
        } else {
            m_xmlReader.skipCurrentElement();
        }
    }
}

QString RoboyControlConfiguration::getModelConfig(const QString attributeName) const {
    if(m_mapModelConfig.contains(attributeName)) {
        return m_mapModelConfig.value(attributeName);
    }
    return QString::null;
}

const QList<qint8> & RoboyControlConfiguration::getControllersConfig() const {
    return m_listControllerConfig;
}