#include "RoboyBehaviorXmlParser.h"

RoboyBehaviorXmlParser::RoboyBehaviorXmlParser() {
    m_databasePath = RoboyControlConfiguration::instance().getModelConfig("databasePath");
    MODEL_DBG << "DATABASE PATH: " << m_databasePath;
}

void RoboyBehaviorXmlParser::persistRoboyBehavior( const RoboyBehavior & behavior ) {
    const QString & name = behavior.m_metadata.m_sBehaviorName;

    MODEL_DBG << "WRITE BEHAVIOR " << name << "TO DATABASE";

    QString path = m_databasePath + "/" + behavior.m_metadata.m_sBehaviorName + ".xml";
    QFile file (path);

    if ( file.exists() ) {
        MODEL_DBG << " - WARNING behavior already exists -> Will replace it.";
        file.remove();
    }

    if (!file.open(QFile::ReadWrite | QFile::Text)) {
        MODEL_DBG << " - ERROR - Failed to open file: " << path;
        return;
    }

    MODEL_DBG << " - INFO - Write file :" << name + ".xml";

    m_xmlWriter.setDevice(&file);

    m_xmlWriter.setAutoFormatting(true);
    m_xmlWriter.writeStartDocument();
    m_xmlWriter.writeStartElement("roboybehavior");
    m_xmlWriter.writeAttribute("name", behavior.m_metadata.m_sBehaviorName);
    m_xmlWriter.writeAttribute("behaviorid", QString::number(behavior.m_metadata.m_ulBehaviorId));

    writeMotorData(behavior);

    m_xmlWriter.writeEndElement();
    m_xmlWriter.writeEndDocument();

    MODEL_DBG << " - INFO - Finished successfully";
}

void RoboyBehaviorXmlParser::writeMotorData( const RoboyBehavior & behavior ) {
    for (quint32 motor : behavior.m_mapMotorTrajectory.keys()) {
        Trajectory currentTrajectory = behavior.m_mapMotorTrajectory.value(motor);

        m_xmlWriter.writeStartElement("motor");
        m_xmlWriter.writeAttribute("motorid", QString::number(motor));
        m_xmlWriter.writeAttribute("controlmode", QString::number(currentTrajectory.m_controlMode));
        m_xmlWriter.writeAttribute("samplerate", QString::number(currentTrajectory.m_sampleRate));

        for (RoboyWaypoint wp : currentTrajectory.m_listWaypoints) {
            m_xmlWriter.writeStartElement("waypoint");
            m_xmlWriter.writeAttribute("id", QString::number(wp.m_ulId));
            m_xmlWriter.writeCharacters(QString::number(wp.m_ulValue));
            m_xmlWriter.writeEndElement();
        }
        m_xmlWriter.writeEndElement();
    }
}

void RoboyBehaviorXmlParser::readRoboyBehaviorMetadata( RoboyBehaviorMetadata & metadata ) {
    QString name = metadata.m_sBehaviorName;

    MODEL_DBG << "READ BEHAVIOR META " << name;

    QString path = m_databasePath + "/" + metadata.m_sBehaviorName + ".xml";
    QFile file(path);

    if (!file.open(QFile::ReadOnly | QFile::Text)) {
        MODEL_DBG << " - ERROR: Failed to open file: " << name + ".xml";
        return;
    }

    m_xmlReader.setDevice(&file);
    m_xmlReader.readNextStartElement();
    if (m_xmlReader.name() == "roboybehavior")
        readBehaviorHeader(metadata);
}

void RoboyBehaviorXmlParser::readRoboyBehavior( RoboyBehavior & behavior ) {
    QString & name = behavior.m_metadata.m_sBehaviorName;

    MODEL_DBG << "READ BEHAVIOR " << name;

    QString path = m_databasePath + "/" + behavior.m_metadata.m_sBehaviorName + ".xml";
    QFile file( path );

    if (!file.open(QFile::ReadOnly | QFile::Text)) {
        MODEL_DBG << " - ERROR: Failed to open file: " << name + ".xml";
        return;
    }

    m_xmlReader.setDevice(&file);
    MODEL_DBG << " - INFO: Read file" << name + ".xml";

    while ( m_xmlReader.readNextStartElement() ) {
        if ( m_xmlReader.name() == "roboybehavior" ) {
            readBehaviorHeader(behavior.m_metadata);
        } else if ( m_xmlReader.name() == "motor" ) {
            readMotorData(behavior);
        } else {
            m_xmlReader.skipCurrentElement();
        }
    }

    MODEL_DBG << " - INFO: Finishd reading successfully";
}

bool RoboyBehaviorXmlParser::readBehaviorHeader( RoboyBehaviorMetadata & metadata ) {
    if ( m_xmlReader.name() == "roboybehavior" &&
         m_xmlReader.attributes().hasAttribute("name") &&
         m_xmlReader.attributes().hasAttribute("behaviorid") ) {

        metadata.m_sBehaviorName = m_xmlReader.attributes().value("name").toString();
        metadata.m_ulBehaviorId = m_xmlReader.attributes().value("behaviorid").toString().toULong();

        MODEL_DBG << "\t- Name:\t" << metadata.m_sBehaviorName;
        MODEL_DBG << "\t- Id:\t" << metadata.m_ulBehaviorId;

        return true;
    }

    MODEL_DBG << "FAILED TO READ BEHAVIOR: couldn't find correct 'roboybehavior'-tag";
    return false;
}

bool RoboyBehaviorXmlParser::readMotorData( RoboyBehavior & behavior ) {
    if ( m_xmlReader.name() == "motor" ) {
        quint32 motor_id = m_xmlReader.attributes().value("motorid").toString().toUInt();
        qint32 controlMode = m_xmlReader.attributes().value("controlmode").toString().toInt();
        qint32 sampleRate = m_xmlReader.attributes().value("samplerate").toString().toInt();

        Trajectory trajectory;
        trajectory.m_controlMode = (ControlMode) controlMode;
        trajectory.m_sampleRate = sampleRate;
        RoboyWaypoint waypoint;

        while (m_xmlReader.readNextStartElement()) {
            if (m_xmlReader.name() == "waypoint") {
                waypoint.m_ulId = m_xmlReader.attributes().value("id").toString().toULong();
                waypoint.m_ulValue = m_xmlReader.readElementText().toULong();
                trajectory.m_listWaypoints.append(waypoint);
            } else {
                m_xmlReader.skipCurrentElement();
            }
        }

        behavior.m_mapMotorTrajectory.insert(motor_id, trajectory);

        MODEL_DBG << "\t- MOTOR ID: " << motor_id << " WAYPOINT COUNT: " << trajectory.m_listWaypoints.count();

        return true;
    }

    return false;
}
