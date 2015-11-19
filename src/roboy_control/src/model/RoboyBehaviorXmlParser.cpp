#include "RoboyBehaviorXmlParser.h"

RoboyBehaviorXmlParser::RoboyBehaviorXmlParser()
{

}

void RoboyBehaviorXmlParser::readRoboyBehavior( const RoboyBehaviorMetadata behaviorMetadata ) {

    QString path = DB_PATH + behaviorMetadata.m_sBehaviorName + ".xml";
    QFile file( path );

    if (!file.open(QFile::ReadOnly | QFile::Text)) {
        LOG << "Failed to open file: " << path;
        exit(0);
    }

    LOG << "Open file successful: " << path;

    m_xmlReader.setDevice(&file);

    RoboyBehavior behavior;

    LOG << "Start to parse RoboyBehavior ...";

    while ( m_xmlReader.readNextStartElement() ) {
        if ( m_xmlReader.name() == "roboybehavior" ) {
            readBehaviorHeader(&behavior);
        } else if ( m_xmlReader.name() == "motor" ) {
            readMotorData(&behavior);
        } else {
            m_xmlReader.skipCurrentElement();
        }
    }

    LOG << "Finished to read behavior: " << behaviorMetadata.m_sBehaviorName;
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
