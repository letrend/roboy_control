#ifndef IMODELSERVICE_H
#define IMODELSERVICE_H

#include "../DataTypes.h"

class IModelService {

public:
    virtual QList<RoboyBehavior> getBehaviorList() = 0;
    virtual RoboyBehavior getBehavior( const RoboyBehaviorMetadata metadata ) = 0;
};

#endif // IMODELSERVICE_H
