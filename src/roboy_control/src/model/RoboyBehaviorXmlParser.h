#ifndef ROBOYBEHAVIORXMLPARSER_H
#define ROBOYBEHAVIORXMLPARSER_H

#include "DataTypes.h"
#include "RoboyControlConfiguration.h"

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

    void writeMotorData( const RoboyBehavior & behavior );
    bool readBehaviorHeader( RoboyBehaviorMetadata & behavior );
    bool readMotorData( RoboyBehavior & behavior );
public:
    RoboyBehaviorXmlParser();

    void persistRoboyBehavior( const RoboyBehavior & behavior );
    void readRoboyBehavior( RoboyBehavior & behavior );
    void readRoboyBehaviorMetadata( RoboyBehaviorMetadata & metadata );
};

#endif // ROBOYBEHAVIORXMLPARSER_H
