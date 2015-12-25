#ifndef ROBOYMULTILANEMODEL_H
#define ROBOYMULTILANEMODEL_H

#include <QVariant>

#include "IMultiLaneViewModel.h"

class RoboyMultiLaneModel : public IMultiLaneViewModel
{
    Q_OBJECT

public:
    RoboyMultiLaneModel(){}
    ~RoboyMultiLaneModel(){}
    virtual void initializeWidget   ();
    qint8  addLane            ();
    qint8  insertLane         (qint32 index);
    qint8  removeLane         (qint32 index);
    qint8  insertBehaviorExec (qint32 laneIndex, quint64 ulTimestamp, RoboyBehavior behavior);
    qint8  removeBehaviorExec (qint32 laneIndex, qint32 itemIndex);
    qint8  removeBehaviorExec (qint32 laneIndex, qint64 lId);

    qint32              laneCount ();
    qint32              itemCount (qint32 laneIndex);
    QVariant            data(qint32 laneIndex, qint32 itemIndex, qint32 role);
    RoboyBehaviorPlan   getBehaviorPlan ();

private:
    QList< QList<RoboyBehaviorExecution> > behaviors;
    quint64 nextAvailableId = 0;
};

#endif // ROBOYMULTILANEMODEL_H
