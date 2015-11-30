#ifndef IMODELSERVICE_H
#define IMODELSERVICE_H

#include "../DataTypes.h"
#include "../interfaces/IObservable.h"

/* Class IModelService defines the interface to the datamodel of Roboy Control.
 * Is derived from the interface IObservalbe, to be observable from other data-
 * dependent components such as the View.
 */
class IModelService : public IObservable {

public:
    /* Database Operations:
     * - Create Behavior
     * - Retrieve Behavior
     * - Update Behavior
     * - Delete Behavior
     * - Get List of all Behaviors
     */
    virtual void persistNewRoboyBehavior( const RoboyBehavior behavior ) = 0;
    virtual QList<RoboyBehaviorMetadata> getBehaviorList() = 0;
    virtual RoboyBehavior getBehavior( const RoboyBehaviorMetadata metadata ) = 0;
};

#endif // IMODELSERVICE_H
