#include "XmlModelService.h"

void XmlModelService::createRoboyBehavior ( const RoboyBehavior & behavior ) {
    m_xmlParser.persistRoboyBehavior(behavior);
    this->notifyAll();
}

RoboyBehavior XmlModelService::retrieveRoboyBehavior ( const RoboyBehaviorMetadata & metadata ) {
    RoboyBehavior behavior;
    behavior.m_metadata.m_sBehaviorName  = metadata.m_sBehaviorName;
    behavior.m_metadata.m_ulBehaviorId   = metadata.m_ulBehaviorId;

    m_xmlParser.readRoboyBehavior(behavior);
    return behavior;
};

void XmlModelService::updateRoboyBehavior ( const RoboyBehavior & behavior ) {
    // TODO:
}

void XmlModelService::deleteRoboyBehavior ( const RoboyBehaviorMetadata & metadata ) {
    // TODO:
}
QList<RoboyBehaviorMetadata> XmlModelService::getBehaviorList() {
    QDir dbDirectory(RoboyControlConfiguration::instance().getModelConfig("databasePath"));
    QList<RoboyBehaviorMetadata> behaviorList;
    RoboyBehaviorMetadata behavior;
    for (QString fileName : dbDirectory.entryList()) {
        if (fileName.endsWith("Behavior.xml", Qt::CaseSensitive)) {
            behavior.m_sBehaviorName = fileName.split(".")[0];
            m_xmlParser.readRoboyBehaviorMetadata(behavior);
            behaviorList.append(behavior);
        }
    }
    return behaviorList;
}