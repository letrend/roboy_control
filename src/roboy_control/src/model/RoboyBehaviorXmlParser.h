#ifndef ROBOYBEHAVIORXMLPARSER_H
#define ROBOYBEHAVIORXMLPARSER_H

#include <QXmlStreamReader>
#include <QFile>
#include <QDebug>

#define LOG_MSG "[ XML Parser ]"
#define LOG     qDebug() << LOG_MSG
#define DB_PATH "/home/bruh/catkin_workspace/src/roboy_control/database/"

struct RoboyBehaviorMetadata {
    u_int64_t   m_ulBehaviorId;
    QString     m_sBehaviorName;
};

struct RoboyWaypoint {
    u_int64_t   m_ulId;
    u_int64_t   m_ulTimestamp;
    u_int64_t   m_ulPosition;
};

struct RoboyBehavior {
    RoboyBehaviorMetadata m_metadata;
    QMap<u_int32_t, QList<RoboyWaypoint>> m_mapMotorWaypoints;
};

class RoboyBehaviorXmlParser
{
private:
    QXmlStreamReader m_xmlReader;

    bool readBehaviorHeader( RoboyBehavior * p_behavior );
    bool readMotorData( RoboyBehavior * p_behavior );

public:
    RoboyBehaviorXmlParser();

    void readRoboyBehavior( const RoboyBehaviorMetadata behaviorMetadata );

};

#endif // ROBOYBEHAVIORXMLPARSER_H
