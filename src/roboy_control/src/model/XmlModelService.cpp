#include "XmlModelService.h"

XmlModelService::XmlModelService()
{

}

void XmlModelService::persistNewRoboyBehavior( const RoboyBehavior behavior ) {
     m_xmlParser.persistRoboyBehavior(&behavior);
}

QList<RoboyBehaviorMetadata> XmlModelService::getBehaviorList() {
    QDir dbDirectory(DB_PATH);
    QList<RoboyBehaviorMetadata> behaviorList;
    RoboyBehaviorMetadata behavior;
    for (QString fileName : dbDirectory.entryList()) {
        if (fileName.endsWith("Behavior.xml", Qt::CaseSensitive)) {
            behavior.m_sBehaviorName = fileName.split(".")[0];
            m_xmlParser.readRoboyBehaviorMetadata(&behavior);
            behaviorList.append(behavior);
        }
    }
    return behaviorList;
}

RoboyBehavior XmlModelService::getBehavior(const RoboyBehaviorMetadata metadata) {
    RoboyBehavior behavior;
    behavior.m_metadata = metadata;

    m_xmlParser.readRoboyBehavior( &behavior );
    return behavior;
}
