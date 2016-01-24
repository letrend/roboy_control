//
// Created by bruh on 1/7/16.
//

#include "DataTypes.h"

#include "IModelService.h"

RoboyBehaviorPlan::RoboyBehaviorPlan(IModelService * modelService, const RoboyBehaviorMetaplan & metaPlan) {
    // Build full-size execution for every MetaExecution by fetching data from model
    RoboyBehaviorExecution execution;
    for (RoboyBehaviorMetaExecution metaExecution : metaPlan.listExecutions) {
        execution.lId = metaExecution.lId;
        execution.lTimestamp = metaExecution.lTimestamp;
        // TODO: Check result -> Whether Behavior exists
        execution.behavior = modelService->retrieveRoboyBehavior(metaExecution.behaviorMetadata);
        listExecutions.append(execution);
    }
}

qint64 RoboyBehaviorPlan::getStartTimestamp() {
    if (lStartTimestamp == -1) {
        qint64 currentTimestamp = -1;
        for (RoboyBehaviorExecution exec : listExecutions) {
            currentTimestamp = exec.lTimestamp;
            currentTimestamp < lStartTimestamp ? lStartTimestamp = currentTimestamp : lStartTimestamp;
        }
    }
    return lStartTimestamp;
}

qint64 RoboyBehaviorPlan::getEndTimestamp() {
    if (lEndTimestamp == -1) {
        qint64 currentEndTimestamp = -1;
        for (RoboyBehaviorExecution exec : listExecutions) {
            currentEndTimestamp = exec.getEndTimestamp();
            currentEndTimestamp > lEndTimestamp ? lEndTimestamp = currentEndTimestamp : lEndTimestamp;
        }
    }
    return lEndTimestamp;
}

qint64 RoboyBehaviorPlan::getDuration() {
    return getEndTimestamp() - getStartTimestamp();
}

const QList<RoboyBehaviorExecution> & RoboyBehaviorPlan::getExcutionsList() const {
    return listExecutions;
}

QMap<qint32, Trajectory> RoboyBehaviorPlan::getTrajectories() const {
    QMap<qint32, Trajectory> mapTrajectories;
    for(RoboyBehaviorExecution execution : listExecutions) {
        for(qint32 motor : execution.behavior.m_mapMotorTrajectory.keys()) {
            if(mapTrajectories.contains(motor)) {
                CONTROLLER_DBG << "Motor-Id clash when building flattened trajectories";
            } else {
                mapTrajectories.insert(motor, execution.behavior.m_mapMotorTrajectory.value(motor));
            }
        }
    }
    return mapTrajectories;
}