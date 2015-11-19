#ifndef XMLMODELSERVICE_H
#define XMLMODELSERVICE_H

#include "IModelService.h"

class XmlModelService : public IModelService
{

private:

public:
    XmlModelService();

    QList<RoboyBehavior> getBehaviorList();
    RoboyBehavior getBehavior(const RoboyBehaviorMetadata metadata);

};

#endif // XMLMODELSERVICE_H
