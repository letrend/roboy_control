#include "XmlModelService.h"

XmlModelService::XmlModelService()
{

}

void XmlModelService::persistNewRoboyBehavior( const RoboyBehavior behavior ) {
     m_xmlParser.persistRoboyBehavior(&behavior);
}

QList<RoboyBehavior> XmlModelService::getBehaviorList() {

}

RoboyBehavior XmlModelService::getBehavior(const RoboyBehaviorMetadata metadata) {
    RoboyBehavior behavior;
    behavior.m_metadata = metadata;

    m_xmlParser.readRoboyBehavior( &behavior );
    return behavior;
}
