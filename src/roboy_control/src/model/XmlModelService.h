#ifndef XMLMODELSERVICE_H
#define XMLMODELSERVICE_H

#include "IModelService.h"
#include "RoboyBehaviorXmlParser.h"

class XmlModelService : public IModelService
{

private:
    RoboyBehaviorXmlParser m_xmlParser;

public:
    void            createRoboyBehavior   ( const RoboyBehavior & behavior );
    RoboyBehavior   retrieveRoboyBehavior ( const RoboyBehaviorMetadata & metadata );
    void            updateRoboyBehavior   ( const RoboyBehavior & behavior );
    void            deleteRoboyBehavior   ( const RoboyBehaviorMetadata & metadata );
    QList<RoboyBehaviorMetadata> getBehaviorList();

};

#endif // XMLMODELSERVICE_H
