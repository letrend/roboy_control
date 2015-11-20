#include "RoboyBehaviorXmlParser.h"

RoboyBehaviorXmlParser::RoboyBehaviorXmlParser()
{
    LOG << "DATABASE PATH: " << DB_PATH;
}

void RoboyBehaviorXmlParser::persistRoboyBehavior( const RoboyBehavior * pBehavior ) {
    const QString & name = pBehavior->m_metadata.m_sBehaviorName;

    LOG << "WRITE BEHAVIOR " << name << "TO DATABASE";

    QString path = DB_PATH + pBehavior->m_metadata.m_sBehaviorName + ".xml";
    QFile file (path);

    if ( file.exists() ) {
        LOG << " - WARNING behavior already exists -> Will replace it.";
        file.remove();
    }

    if (!file.open(QFile::ReadWrite | QFile::Text)) {
        LOG << " - ERROR - Failed to open file: " << path;
        return;
    }

    LOG << " - INFO - Write file :" << name + ".xml";

    m_xmlWriter.setDevice(&file);

    m_xmlWriter.setAutoFormatting(true);
    m_xmlWriter.writeStartDocument();
    m_xmlWriter.writeStartElement("roboybehavior");
    m_xmlWriter.writeAttribute("name", pBehavior->m_metadata.m_sBehaviorName);
    m_xmlWriter.writeAttribute("behaviorid", QString::number(pBehavior->m_metadata.m_ulBehaviorId));

    writeMotorData(pBehavior);

    m_xmlWriter.writeEndElement();
    m_xmlWriter.writeEndDocument();

    LOG << " - INFO - Finished successfully";
}

void RoboyBehaviorXmlParser::writeMotorData( const RoboyBehavior * pBehavior ) {
    for (quint32 motor : pBehavior->m_mapMotorWaypoints.keys()) {
        m_xmlWriter.writeStartElement("motor");
        m_xmlWriter.writeAttribute("motorid", QString::number(motor));
        for (RoboyWaypoint wp : pBehavior->m_mapMotorWaypoints.value(motor)) {
            m_xmlWriter.writeStartElement("waypoint");
            m_xmlWriter.writeAttribute("id", QString::number(wp.m_ulId));
            m_xmlWriter.writeAttribute("timestamp", QString::number(wp.m_ulTimestamp));
            m_xmlWriter.writeCharacters(QString::number(wp.m_ulPosition));
            m_xmlWriter.writeEndElement();
        }
        m_xmlWriter.writeEndElement();
    }
}

void RoboyBehaviorXmlParser::readRoboyBehavior( RoboyBehavior * pBehavior ) {
    QString & name = pBehavior->m_metadata.m_sBehaviorName;

    LOG << "READ BEHAVIOR " << name;

    QString path = DB_PATH + pBehavior->m_metadata.m_sBehaviorName + ".xml";
    QFile file( path );

    if (!file.open(QFile::ReadOnly | QFile::Text)) {
        LOG << " - ERROR: Failed to open file: " << name + ".xml";
        return;
    }

    m_xmlReader.setDevice(&file);

    LOG << " - INFO: Read file" << name + ".xml";

    while ( m_xmlReader.readNextStartElement() ) {
        if ( m_xmlReader.name() == "roboybehavior" ) {
            readBehaviorHeader(pBehavior);
        } else if ( m_xmlReader.name() == "motor" ) {
            readMotorData(pBehavior);
        } else {
            m_xmlReader.skipCurrentElement();
        }
    }

    LOG << " - INFO: Finishd reading successfully";
}

bool RoboyBehaviorXmlParser::readBehaviorHeader( RoboyBehavior * p_behavior ) {
    if ( m_xmlReader.name() == "roboybehavior" &&
         m_xmlReader.attributes().hasAttribute("name") &&
         m_xmlReader.attributes().hasAttribute("behaviorid") ) {

        p_behavior->m_metadata.m_sBehaviorName = m_xmlReader.attributes().value("name").toString();
        p_behavior->m_metadata.m_ulBehaviorId = m_xmlReader.attributes().value("behaviorid").toString().toULong();

        LOG << "\t- Name:\t" << p_behavior->m_metadata.m_sBehaviorName;
        LOG << "\t- Id:\t" << p_behavior->m_metadata.m_ulBehaviorId;

        return true;
    }

    LOG << "FAILED TO READ BEHAVIOR: couldn't find correct 'roboybehavior'-tag";
    return false;
}

bool RoboyBehaviorXmlParser::readMotorData( RoboyBehavior *p_behavior ) {
    if ( m_xmlReader.name() == "motor" ) {
        u_int32_t motor_id = m_xmlReader.attributes().value("motorid").toString().toUInt();

        QList<RoboyWaypoint> waypointList;
        RoboyWaypoint waypoint;

        while (m_xmlReader.readNextStartElement()) {
            if (m_xmlReader.name() == "waypoint") {
                waypoint.m_ulId = m_xmlReader.attributes().value("id").toString().toULong();
                waypoint.m_ulTimestamp = m_xmlReader.attributes().value("timestamp").toString().toULong();
                waypoint.m_ulPosition = m_xmlReader.readElementText().toULong();
                waypointList.append(waypoint);
            } else {
                m_xmlReader.skipCurrentElement();
            }
        }

        p_behavior->m_mapMotorWaypoints.insert(motor_id, waypointList);

        LOG << "\t- MOTOR ID: " << motor_id << " WAYPOINT COUNT: " << waypointList.count();

        return true;
    }

    return false;
}
