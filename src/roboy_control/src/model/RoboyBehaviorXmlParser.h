#ifndef ROBOYBEHAVIORXMLPARSER_H
#define ROBOYBEHAVIORXMLPARSER_H

#include "DataTypes.h"
#include "LogDefines.h"
#include "RoboyControlConfiguration.h"

#include <QXmlStreamReader>
#include <QXmlStreamWriter>
#include <QFile>
#include <QDir>

class RoboyBehaviorXmlParser
{
private:
    QString m_databasePath;
    QXmlStreamReader m_xmlReader;
    QXmlStreamWriter  m_xmlWriter;

    void writeTrajectories(const RoboyBehavior &behavior);
    bool readBehaviorHeader( RoboyBehaviorMetadata & behavior );
    bool readTrajectories(RoboyBehavior &behavior);
public:
    RoboyBehaviorXmlParser();

    void persistRoboyBehavior( const RoboyBehavior & behavior );
    void readRoboyBehavior( RoboyBehavior & behavior );
    void readRoboyBehaviorMetadata( RoboyBehaviorMetadata & metadata );
};

#endif // ROBOYBEHAVIORXMLPARSER_H
