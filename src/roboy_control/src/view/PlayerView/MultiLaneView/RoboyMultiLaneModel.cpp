#include <QtCore>
#include <QIcon>

#include "RoboyMultiLaneModel.h"
#include "MultiLaneViewLane.h"

/* public methods */

/**
 * @brief RoboyMultiLaneModel::initializeWidget method to initialize the MultiLaneWidget
 */
void RoboyMultiLaneModel::initializeWidget()
{
    int numBehaviorLanes = this->behaviors.count();
    for (int i = 0; i < numBehaviorLanes; i++) {

        emit laneInserted(i);

        QList<RoboyBehaviorExecution> behaviorLane = this->behaviors[i];
        for (int j = 0; j < behaviorLane.count(); j++) {
           emit itemInserted(i, j);
        }
    }
}

/**
 * @brief RoboyMultiLaneModel::addLane method to add a lane to the MultiLaneWidget
 * @return 0 for success, -1 on failure
 */
qint8 RoboyMultiLaneModel::addLane()
{
    this->behaviors.append(QList<RoboyBehaviorExecution>());
    emit laneInserted(this->behaviors.count()-1);
    return 0;
}

/**
 * @brief RoboyMultiLaneModel::insertLane method to insert a lane to the MultiLaneWidet
 * @param index index at which the lane should be inserted
 * @return 0 for success, -1 on failure
 */
qint8 RoboyMultiLaneModel::insertLane(qint32 index)
{
    if (index >= 0 && index < this->behaviors.count()) {
        QList<RoboyBehaviorExecution> newLane;
        this->behaviors.insert(index, newLane);
        emit laneInserted(index);
        return 0;
    }
    return -1;
}

/**
 * @brief RoboyMultiLaneModel::removeLane method to remove a lane from the MultiLaneWidget
 * @param index index of the lane that should be removed
 * @return 0 for success, -1 on failure
 */
qint8 RoboyMultiLaneModel::removeLane(qint32 index)
{
    if (index >= 0 && index < this->behaviors.count()) {
        this->behaviors.removeAt(index);
        emit laneRemoved(index);
        return 0;
    }
    return -1;
}

/**
 * @brief RoboyMultiLaneModel::insertBehaviorExec method to insert a new RoboyBehaviorExecution
 * @param laneIndex index of the lane where the RoboyBehaviorExecution should be inserted
 * @param lTimestamp timestamp at which the RoboyBehaviorExecution should be inserted
 * @param behavior RoboyBehavior for the RoboyBehaviorExecution
 * @return 0 for success, -1 on failure
 */
qint8  RoboyMultiLaneModel::insertBehaviorExec(qint32 laneIndex, qint64 lTimestamp, RoboyBehavior behavior)
{
    qint64 behaviorDuration = behavior.getDuration();
    /* edge case empty list */
    if (laneIndex >= 0 && laneIndex < this->behaviors.count()) {
        if(this->behaviors[laneIndex].count() < 1) {
            const RoboyBehaviorExecution bExec = {this->nextAvailableId++, lTimestamp, behavior};
            this->behaviors[laneIndex].append(bExec);
            emit itemInserted(laneIndex, 0);
            return 0;
        }

        for (int i = 0; i < this->behaviors[laneIndex].count()-1; i++) {
            if(this->behaviors[laneIndex][i].lTimestamp <= lTimestamp && this->behaviors[laneIndex][i+1].lTimestamp > lTimestamp+behaviorDuration) {
                RoboyBehaviorExecution bExec = {this->nextAvailableId++, lTimestamp, behavior};
                this->behaviors[laneIndex].insert(i, bExec);
                emit itemInserted(laneIndex, i);
                return 0;
            }
        }
        /* edge case first item */

        if (lTimestamp + behaviorDuration < this->behaviors[laneIndex].first().lTimestamp) {
            RoboyBehaviorExecution bExec = {this->nextAvailableId++, lTimestamp, behavior};
            this->behaviors[laneIndex].insert(0, bExec);
            emit itemInserted(laneIndex, 0);
            return 0;
        }

        /*edge case last item */
        if (lTimestamp > this->behaviors[laneIndex].last().lTimestamp) {
            RoboyBehaviorExecution bExec = {this->nextAvailableId++, lTimestamp, behavior};
            this->behaviors[laneIndex].append(bExec);
            emit itemInserted(laneIndex, this->behaviors[laneIndex].count()-1);
            return 0;
        }
    }
    return -1;
}

/**
 * @brief RoboyMultiLaneModel::removeBehaviorExec method to remove a previously inserted RoboyBehaviorExecution
 * @param laneIndex index of the lane of the RoboyBehaviorExecution
 * @param itemIndex index of the RoboyBehaviorExecution in the lane
 * @return 0 for success, -1 on failure
 */
qint8 RoboyMultiLaneModel::removeBehaviorExecWithIndex(qint32 laneIndex, qint32 itemIndex)
{
    if (laneIndex >= 0 && laneIndex < this->behaviors.count()) {
        if(itemIndex >= 0 && itemIndex < this->behaviors[laneIndex].count()) {
            this->behaviors[laneIndex].removeAt(itemIndex);
            emit itemRemoved(laneIndex, itemIndex);
            return 0;
        }
    }
    return -1;
}

/**
 * @brief RoboyMultiLaneModel::removeBehaviorExec method to remove a previously inserted RoboyBehaviorExecution
 * @param laneIndex index of the lane of the RoboyBehaviorExecution
 * @param lId ID of the RoboyBehaviorExecution that should be removed
 * @return 0 for success, -1 on failure
 */
qint8 RoboyMultiLaneModel::removeBehaviorExecWithID(qint32 laneIndex, qint64 lId)
{
    if(laneIndex >= 0 && laneIndex < this->behaviors.count()) {
        for (int i = 0; i < this->behaviors[laneIndex].count(); i++) {
            if (this->behaviors[laneIndex][i].lId == lId) {
                this->behaviors[laneIndex].removeAt(i);
                emit itemRemoved(laneIndex, i);
                return 0;
            }
        }
    }
    return -1;
}

/**
 * @brief RoboyMultiLaneModel::removeBehaviorExec method to remove a previously inserted RoboyBehaviorExecution
 * @param laneIndex index of the lane of the RoboyBehaviorExecution
 * @param timestamp
 * @return 0 for success, -1 on failure
 */
qint8 RoboyMultiLaneModel::removeBehaviorExecWithTimestamp(qint32 laneIndex, qint64 timestamp)
{
    if(laneIndex >= 0 && laneIndex < this->behaviors.count()) {
        for (int i = 0; i < this->behaviors[laneIndex].count(); i++) {
            if (this->behaviors[laneIndex][i].lTimestamp == timestamp) {
                this->behaviors[laneIndex].removeAt(i);
                emit itemRemoved(laneIndex, i);
                return 0;
            }
        }
    }
    return -1;
}

/**
 * @brief RoboyMultiLaneModel::laneCount method for retrieving the current number of lanes
 * @return number of lanes
 */
qint32 RoboyMultiLaneModel::laneCount()
{
    return this->behaviors.count();
}

/**
 * @brief RoboyMultiLaneModel::itemCount method for retrieving the current number of items in the lane at the index laneIndex
 * @param laneIndex index of the lane
 * @return number of items in the lane at laneIndex
 */
qint32 RoboyMultiLaneModel::itemCount(qint32 laneIndex)
{
    if (laneIndex >= 0 && laneIndex < this->behaviors.count()) {
        return this->behaviors[laneIndex].count();
    }
    return 0;
}

/**
 * @brief RoboyMultiLaneModel::data method for retrieving the data to display in the MultiLaneWidget
 * @param laneIndex index of the lane of the RoboyBehaviorExecution of which the data should be retrieved
 * @param itemIndex index of the RoboyBehaviorExecution in the lane of which the data should be retrieved
 * @param role the role for which you want to retrieve the data
 * @return the QVariant that should be displayed
 */
QVariant RoboyMultiLaneModel::data(qint32 laneIndex, qint32 itemIndex, qint32 role)
{
    if(laneIndex >= 0 && laneIndex < this->behaviors.count()) {
        if(itemIndex >=  0 && itemIndex < this->behaviors[laneIndex].count()) {
            RoboyBehaviorExecution behaviorExec = this->behaviors[laneIndex][itemIndex];
            if (role == Qt::DisplayRole) // behavior name
                return behaviorExec.behavior.m_metadata.m_sBehaviorName;
            else if (role == Qt::DecorationRole) // behavior icon
                return QIcon(":/behavior-img-light.png");
            else if (role == Qt::UserRole) // behavior timestamp
                return QVariant(behaviorExec.lTimestamp);
            else if (role == Qt::UserRole + 1) //behavior duration
                return QVariant(behaviorExec.behavior.getDuration());
            else if (role == Qt::UserRole + 2) // behavior motor count
                return (QVariant(behaviorExec.behavior.m_mapMotorTrajectory.count()));
        }
    }
    return QVariant();
}

/**
 * @brief RoboyMultiLaneModel::getBehaviorPlan method to retrieve a RoboyBehaviorPlan from the MultiLaneWidget
 * @return a RoboyBehaviorPlan representing the executions displayed in the MultiLaneWidget
 */
RoboyBehaviorMetaplan RoboyMultiLaneModel::getBehaviorPlan()
{
    struct BehaviorExecComparator
    {
      bool operator()(RoboyBehaviorMetaExecution a, RoboyBehaviorMetaExecution b) const
      {
        return a.lTimestamp < b.lTimestamp;
      }
    };

    RoboyBehaviorMetaplan metaPlan;

    for(QList<RoboyBehaviorExecution> behaviorList : this->behaviors) {
        for(RoboyBehaviorExecution behaviorExec : behaviorList) {
            RoboyBehaviorMetaExecution metaExec;
            metaExec.lId                = behaviorExec.lId;
            metaExec.lTimestamp         = behaviorExec.lTimestamp;
            metaExec.behaviorMetadata   = behaviorExec.behavior.m_metadata;
            metaPlan.listExecutions.append(metaExec);
        }
    }

    qSort(metaPlan.listExecutions.begin(), metaPlan.listExecutions.end(), BehaviorExecComparator());

    return metaPlan;
}
