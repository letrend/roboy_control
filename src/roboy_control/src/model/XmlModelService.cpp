#include "XmlModelService.h"

XmlModelService::~XmlModelService() {

}

void XmlModelService::createRoboyBehavior ( const RoboyBehavior & behavior ) {
    m_xmlParser.persistRoboyBehavior(behavior);
    this->notifyAll();
}

bool XmlModelService::retrieveRoboyBehavior(RoboyBehavior &behavior) {
    return m_xmlParser.readRoboyBehavior(behavior);
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
        if (fileName.endsWith(".xml", Qt::CaseSensitive)) {
            behavior.m_sBehaviorName = fileName.split(".")[0];
            m_xmlParser.readRoboyBehaviorMetadata(behavior);
            behaviorList.append(behavior);
        }
    }
    return behaviorList;
}