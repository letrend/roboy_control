#ifndef ROBOYBEHAVIORXMLPARSER_H
#define ROBOYBEHAVIORXMLPARSER_H

#include "../DataTypes.h"

#include <QXmlStreamReader>
#include <QXmlStreamWriter>
#include <QFile>
#include <QDebug>
#include <QDir>

#define LOG_MSG "[ XML Parser ]"
#define LOG     qDebug() << LOG_MSG
#define DB_PATH "/home/bruh/catkin_workspace/src/roboy_control/database/"

class RoboyBehaviorXmlParser
{
private:
    QXmlStreamReader m_xmlReader;
    QXmlStreamWriter  m_xmlWriter;

    void writeMotorData( const RoboyBehavior * pBehavior );
    bool readBehaviorHeader( RoboyBehaviorMetadata * p_behavior );
    bool readMotorData( RoboyBehavior * p_behavior );
public:
    RoboyBehaviorXmlParser();

    void persistRoboyBehavior( const RoboyBehavior * pBehavior );
    void readRoboyBehavior( RoboyBehavior * pBehavior );
    void readRoboyBehaviorMetadata( RoboyBehaviorMetadata * pBehaviorMetadata);
};

#endif // ROBOYBEHAVIORXMLPARSER_H
