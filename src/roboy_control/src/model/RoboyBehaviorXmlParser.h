#ifndef ROBOYBEHAVIORXMLPARSER_H
#define ROBOYBEHAVIORXMLPARSER_H

#include "../DataTypes.h"
#include "../controller/RoboyControlConfiguration.h"

#include <QXmlStreamReader>
#include <QXmlStreamWriter>
#include <QFile>
#include <QDir>

#define MODEL_DBG DBG << "[ XML Parser ]"

class RoboyBehaviorXmlParser
{
private:
    QString m_databasePath;
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
