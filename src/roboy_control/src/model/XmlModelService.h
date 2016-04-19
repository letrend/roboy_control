#ifndef XMLMODELSERVICE_H
#define XMLMODELSERVICE_H

#include "IModelService.h"
#include "RoboyBehaviorXmlParser.h"

class XmlModelService : public IModelService
{

private:
    RoboyBehaviorXmlParser m_xmlParser;

public:
    ~XmlModelService();

    void            createRoboyBehavior   ( const RoboyBehavior & behavior );
    bool            retrieveRoboyBehavior ( RoboyBehavior & behavior );
    void            updateRoboyBehavior   ( const RoboyBehavior & behavior );
    void            deleteRoboyBehavior   ( const RoboyBehaviorMetadata & metadata );
    QList<RoboyBehaviorMetadata> getBehaviorList();

};

#endif // XMLMODELSERVICE_H
