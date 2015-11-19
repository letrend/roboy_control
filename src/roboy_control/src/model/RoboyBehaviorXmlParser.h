#ifndef ROBOYBEHAVIORXMLPARSER_H
#define ROBOYBEHAVIORXMLPARSER_H

#include "../DataTypes.h"

#include <QXmlStreamReader>
#include <QFile>
#include <QDebug>

#define LOG_MSG "[ XML Parser ]"
#define LOG     qDebug() << LOG_MSG
#define DB_PATH "/home/bruh/catkin_workspace/src/roboy_control/database/"

class RoboyBehaviorXmlParser
{
private:
    QXmlStreamReader m_xmlReader;

    bool readBehaviorHeader( RoboyBehavior * p_behavior );
    bool readMotorData( RoboyBehavior * p_behavior );

public:
    RoboyBehaviorXmlParser();

    void readRoboyBehavior( RoboyBehavior * behaviorMetadata );

};

#endif // ROBOYBEHAVIORXMLPARSER_H
