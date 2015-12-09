#ifndef IMULTILANEVIEWMODEL_H
#define IMULTILANEVIEWMODEL_H

#include <QObject>

#include "DataTypes.h"

// http://stackoverflow.com/questions/17943496/declare-abstract-signal-in-interface-class

class IMultiLaneViewModel : public QObject
{
    Q_OBJECT

public:
    virtual int laneCount() const = 0;
    virtual int itemCountForLane(int laneIndex) = 0;
    virtual QVariant data(int laneIndex, int itemIndex) const = 0;
    virtual void addLane() = 0;
    virtual void insertBehaviorExecution(int lane, quint64 ulTimestamp, RoboyBehavior behavior) = 0;
    virtual void removeBehaviorExecution(qint64 lId) = 0;
    virtual RoboyBehaviorPlan getBehaviorPlan() = 0;

signals:
    virtual void dataChanged() = 0;
};

Q_DECLARE_INTERFACE(IMultiLaneViewModel, "IMultiLaneViewModel")

#endif // IMULTILANEVIEWMODEL_H
