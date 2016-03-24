#ifndef ROBOYMULTILANEMODEL_H
#define ROBOYMULTILANEMODEL_H

#include <QVariant>

#include "multiLaneView/IMultiLaneViewModel.h"

class RoboyMultiLaneModel : public IMultiLaneViewModel {
    Q_OBJECT

public:
    RoboyMultiLaneModel();
    ~RoboyMultiLaneModel();
    qint8  addLane            ();
    qint8  insertLane         (qint32 index);
    qint8  removeLane         (qint32 index);
    qint8  insertBehaviorExec (qint32 laneIndex, qint64 lTimestamp, RoboyBehavior behavior);
    qint8  removeBehaviorExecWithIndex (qint32 laneIndex, qint32 itemIndex);
    qint8  removeBehaviorExecWithID (qint32 laneIndex, qint64 lId);
    qint8  removeBehaviorExecWithTimestamp (qint32 laneIndex, qint64 timestamp);

public slots:
    qint32                  laneCount ();
    qint32                  itemCount (qint32 laneIndex);
    QVariant                data(qint32 laneIndex, qint32 itemIndex, qint32 role);
    RoboyBehaviorMetaplan   getBehaviorMetaPlan ();

private:
    QList<QList<RoboyBehaviorExecution>> mBehaviors;
    qint64 mNextAvailableId = 0;
};

#endif // ROBOYMULTILANEMODEL_H
