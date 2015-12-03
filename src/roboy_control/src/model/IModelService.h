#ifndef IMODELSERVICE_H
#define IMODELSERVICE_H

#include "DataTypes.h"
#include "IObservable.h"

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
    virtual void            createRoboyBehavior   ( const RoboyBehavior & behavior ) = 0;
    virtual RoboyBehavior   retrieveRoboyBehavior ( const RoboyBehaviorMetadata & metadata ) = 0;
    virtual void            updateRoboyBehavior   ( const RoboyBehavior & behavior ) = 0;
    virtual void            deleteRoboyBehavior   ( const RoboyBehaviorMetadata & metadata ) = 0;
    virtual QList<RoboyBehaviorMetadata> getBehaviorList() = 0;

};

#endif // IMODELSERVICE_H
