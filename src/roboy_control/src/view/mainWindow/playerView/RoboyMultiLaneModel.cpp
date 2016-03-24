#include <QtCore>
#include <QIcon>

#include "RoboyMultiLaneModel.h"

/* public methods */

/**
 * @brief RoboyMultiLaneModel::RoboyMultiLaneModel constructor of RoboyMultiLaneModel
 */
RoboyMultiLaneModel::RoboyMultiLaneModel() {
    addLane(); // initially one lane
}

/**
 * @brief RoboyMultiLaneModel::~RoboyMultiLaneModel destructor of RoboyMultiLaneModel
 */
RoboyMultiLaneModel::~RoboyMultiLaneModel() {

}

/**
 * @brief RoboyMultiLaneModel::addLane method to add a lane to the MultiLaneWidget
 * @return 0 for success, -1 on failure
 */
qint8 RoboyMultiLaneModel::addLane() {
    mBehaviors.append(QList<RoboyBehaviorExecution>());
    emit dataChanged();
    return 0;
}

/**
 * @brief RoboyMultiLaneModel::insertLane method to insert a lane to the MultiLaneWidet
 * @param index index at which the lane should be inserted
 * @return 0 for success, -1 on failure
 */
qint8 RoboyMultiLaneModel::insertLane(qint32 index) {
    if (index >= 0 && index < mBehaviors.count()) {
        QList<RoboyBehaviorExecution> newLane;
        mBehaviors.insert(index, newLane);
        emit dataChanged();
        return 0;
    }
    return -1;
}

/**
 * @brief RoboyMultiLaneModel::removeLane method to remove a lane from the MultiLaneWidget
 * @param index index of the lane that should be removed
 * @return 0 for success, -1 on failure
 */
qint8 RoboyMultiLaneModel::removeLane(qint32 index) {
    if (index >= 0 && index < mBehaviors.count()) {
        mBehaviors.removeAt(index);
        emit dataChanged();
        return 0;
    }
    return -1;
}

/**
 * @brief RoboyMultiLaneModel::insertBehaviorExec method to insert a new RoboyBehaviorExecution
 * @param laneIndex index of the lane where the RoboyBehaviorExecution should be inserted
 * @param lTimestamp timestamp at which the RoboyBehaviorExecution should be inserted
 * @param behavior RoboyBehavior for the RoboyBehaviorExecution
 * @return 0 for success, -1 if timestamp is no multiple of 100, -2 if behaviors would overlap
 */
qint8  RoboyMultiLaneModel::insertBehaviorExec(qint32 laneIndex, qint64 lTimestamp, RoboyBehavior behavior) {
    /* the timestamp can only be a multiple of the sample rate which is 100ms */
    if (lTimestamp%100 != 0) {
        return -1;
    }

    qint64 behaviorDuration = behavior.getDuration();

    /* edge case empty list */
    if (laneIndex >= 0 && laneIndex < mBehaviors.count()) {
        if (mBehaviors[laneIndex].count() < 1) {
            const RoboyBehaviorExecution bExec = {mNextAvailableId++, lTimestamp, behavior};
            mBehaviors[laneIndex].append(bExec);
            emit dataChanged();
            return 0;
        }

        for (int i = 0; i < mBehaviors[laneIndex].count()-1; i++) {
            if (mBehaviors[laneIndex][i].lTimestamp + mBehaviors[laneIndex][i].behavior.getDuration() <= lTimestamp &&
                    mBehaviors[laneIndex][i+1].lTimestamp > lTimestamp+behaviorDuration) {
                RoboyBehaviorExecution bExec = {mNextAvailableId++, lTimestamp, behavior};
                mBehaviors[laneIndex].insert(i, bExec);
                emit dataChanged();
                return 0;
            }
        }

        /* edge case first item */
        if (lTimestamp + behaviorDuration < mBehaviors[laneIndex].first().lTimestamp) {
            RoboyBehaviorExecution bExec = {mNextAvailableId++, lTimestamp, behavior};
            mBehaviors[laneIndex].insert(0, bExec);
            emit dataChanged();
            return 0;
        }

        /*edge case last item */
        if (lTimestamp > mBehaviors[laneIndex].last().lTimestamp + mBehaviors[laneIndex].last().behavior.getDuration()) {
            RoboyBehaviorExecution bExec = {mNextAvailableId++, lTimestamp, behavior};
            mBehaviors[laneIndex].append(bExec);
            emit dataChanged();
            return 0;
        }
    }
    return -2;
}

/**
 * @brief RoboyMultiLaneModel::removeBehaviorExec method to remove a previously inserted RoboyBehaviorExecution
 * @param laneIndex index of the lane of the RoboyBehaviorExecution
 * @param itemIndex index of the RoboyBehaviorExecution in the lane
 * @return 0 for success, -1 on failure
 */
qint8 RoboyMultiLaneModel::removeBehaviorExecWithIndex(qint32 laneIndex, qint32 itemIndex) {
    if (laneIndex >= 0 && laneIndex < mBehaviors.count()) {
        if(itemIndex >= 0 && itemIndex < mBehaviors[laneIndex].count()) {
            mBehaviors[laneIndex].removeAt(itemIndex);
            emit dataChanged();
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
qint8 RoboyMultiLaneModel::removeBehaviorExecWithID(qint32 laneIndex, qint64 lId) {
    if(laneIndex >= 0 && laneIndex < mBehaviors.count()) {
        for (int i = 0; i < mBehaviors[laneIndex].count(); i++) {
            if (mBehaviors[laneIndex][i].lId == lId) {
                mBehaviors[laneIndex].removeAt(i);
                emit dataChanged();
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
qint8 RoboyMultiLaneModel::removeBehaviorExecWithTimestamp(qint32 laneIndex, qint64 timestamp) {
    if(laneIndex >= 0 && laneIndex < mBehaviors.count()) {
        for (int i = 0; i < mBehaviors[laneIndex].count(); i++) {
            if (mBehaviors[laneIndex][i].lTimestamp == timestamp) {
                mBehaviors[laneIndex].removeAt(i);
                emit dataChanged();
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
qint32 RoboyMultiLaneModel::laneCount() {
    return mBehaviors.count();
}

/**
 * @brief RoboyMultiLaneModel::itemCount method for retrieving the current number of items in the lane at the index laneIndex
 * @param laneIndex index of the lane
 * @return number of items in the lane at laneIndex
 */
qint32 RoboyMultiLaneModel::itemCount(qint32 laneIndex) {
    if (laneIndex >= 0 && laneIndex < mBehaviors.count()) {
        return mBehaviors[laneIndex].count();
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
QVariant RoboyMultiLaneModel::data(qint32 laneIndex, qint32 itemIndex, qint32 role) {
    if(laneIndex >= 0 && laneIndex < mBehaviors.count()) {
        if(itemIndex >=  0 && itemIndex < mBehaviors[laneIndex].count()) {
            RoboyBehaviorExecution behaviorExec = mBehaviors[laneIndex][itemIndex];
            if (role == Qt::DisplayRole) // behavior name
                return behaviorExec.behavior.m_metadata.m_sBehaviorName;
            else if (role == Qt::DecorationRole) // behavior icon
                return QString("action/accessibility");
            else if (role == Qt::UserRole) // behavior timestamp
                return QVariant(behaviorExec.lTimestamp);
            else if (role == Qt::UserRole + 1) //behavior duration
                return QVariant(behaviorExec.behavior.getDuration());
            else if (role == Qt::UserRole + 2) // behavior motor count
                return (QVariant(behaviorExec.behavior.m_mapMotorTrajectory.count()));
                return 0;
        }
    }
    return QVariant();
}

/**
 * @brief RoboyMultiLaneModel::getBehaviorMetaPlan method to retrieve a RoboyBehaviorPlan from the MultiLaneWidget
 * @return a RoboyBehaviorPlan representing the executions displayed in the MultiLaneWidget
 */
RoboyBehaviorMetaplan RoboyMultiLaneModel::getBehaviorMetaPlan() {
    struct BehaviorExecComparator {
        bool operator()(RoboyBehaviorMetaExecution a, RoboyBehaviorMetaExecution b) const {
            return a.lTimestamp < b.lTimestamp;
        }
    };

    RoboyBehaviorMetaplan metaPlan;

    for(QList<RoboyBehaviorExecution> behaviorList : mBehaviors) {
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
