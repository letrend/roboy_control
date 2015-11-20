#ifndef XMLMODELSERVICE_H
#define XMLMODELSERVICE_H

#include "IModelService.h"
#include "RoboyBehaviorXmlParser.h"

class XmlModelService : public IModelService
{

private:
    RoboyBehaviorXmlParser m_xmlParser;

public:
    XmlModelService();

    void persistNewRoboyBehavior( const RoboyBehavior behavior );
    QList<RoboyBehavior> getBehaviorList();
    RoboyBehavior getBehavior(const RoboyBehaviorMetadata metadata);

};

#endif // XMLMODELSERVICE_H
