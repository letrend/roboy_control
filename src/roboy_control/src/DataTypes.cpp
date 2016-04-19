//
// Created by bruh on 1/7/16.
//

#include "DataTypes.h"

#include "IControllerCommunication.h"
#include "IModelService.h"

RoboyBehaviorPlan::RoboyBehaviorPlan(IModelService * modelService, const RoboyBehaviorMetaplan & metaPlan) {
    // Build full-size execution for every MetaExecution by fetching data from model
    m_bLoadedCompletely = true;
    RoboyBehaviorExecution execution;
    for (RoboyBehaviorMetaExecution metaExecution : metaPlan.listExecutions) {
        execution.lId = metaExecution.lId;
        execution.lTimestamp = metaExecution.lTimestamp;
        RoboyBehavior behavior;
        behavior.m_metadata = metaExecution.behaviorMetadata;
        if(modelService->retrieveRoboyBehavior(behavior)) {
            execution.behavior = behavior;
            m_listExecutions.append(execution);
        } else {
            PLAN_WAR << "Behavior: " << behavior.m_metadata.m_sBehaviorName << " not found.";
            m_bLoadedCompletely = false;
        }
    }

    m_sampleRate = 100;

    setStartTimestamp();
    setEndTimestamp();
}

bool RoboyBehaviorPlan::isEmpty() const {
    return m_listExecutions.isEmpty();
}

bool RoboyBehaviorPlan::isLoadedCompletely() const {
    return m_bLoadedCompletely;
}

qint64 RoboyBehaviorPlan::getStartTimestamp() const {
    return m_startTimestamp;
}
qint64 RoboyBehaviorPlan::getEndTimestamp() const {
    return m_endTimestamp;
}

qint64 RoboyBehaviorPlan::getDuration() const {
    return (getEndTimestamp() - getStartTimestamp());
}

const QList<RoboyBehaviorExecution> & RoboyBehaviorPlan::getExecutionsList() const {
    return m_listExecutions;
}

QMap<qint32, Trajectory> RoboyBehaviorPlan::getTrajectories() const {
    return m_mapMotorTrajectories;
}

// Private Interface

void RoboyBehaviorPlan::setStartTimestamp() {
    m_startTimestamp = 0xffffffffffffff;
    qint64 currentTimestamp = 0;
    for (RoboyBehaviorExecution exec : m_listExecutions) {
        currentTimestamp = exec.lTimestamp;
        if(currentTimestamp < m_startTimestamp)
            m_startTimestamp = currentTimestamp;
    }
}

void RoboyBehaviorPlan::setEndTimestamp() {
    qint64 currentEndTimestamp = -1;
    for (RoboyBehaviorExecution exec : m_listExecutions) {
        currentEndTimestamp = exec.getEndTimestamp();
        if(currentEndTimestamp > m_endTimestamp)
            m_endTimestamp = currentEndTimestamp;
    }
}

bool RoboyBehaviorPlan::doFlattening() {
    // Build MotorId - Trajectory Map for whole plan
    qint32 waypointCount = this->getDuration() / m_sampleRate;

    PLAN_DBG << "Begin flattening Trajectories";
    PLAN_DBG << "\t- Start Timestamp: " << this->m_startTimestamp;
    PLAN_DBG << "\t- End Timestamp: "   << this->m_endTimestamp;
    PLAN_DBG << "\t- Duration: "        << this->getDuration();
    PLAN_DBG << "\t- SampleRate:"       << this->m_sampleRate;
    PLAN_DBG << "\t- Waypoint Count:"   << waypointCount;

    Trajectory trajectory;
    trajectory.m_controlMode = ControlMode::UNDEFINED_CONTROL;
    trajectory.m_sampleRate =  m_sampleRate;

    RoboyWaypoint waypoint;
    waypoint.m_ulValue = 0xffffffffffffffff;

    for(RoboyBehaviorExecution execution : m_listExecutions) {
        // Initialize Motor-Trajectory Map with defaul Values 0xfffffffffffffffff
        for(qint32 motorId : execution.behavior.m_mapMotorTrajectory.keys()) {
            if(!m_mapMotorTrajectories.contains(motorId)){
                trajectory.m_listWaypoints.clear();
                for(int i = 0; i < waypointCount; i++) {
                    trajectory.m_listWaypoints.append(waypoint);
                }
                m_mapMotorTrajectories.insert(motorId, trajectory);
            }
        }
        // Try to insert every execution
        if(!insertExecution(execution))
            return false;
    }

    printMap();
    return true;
}

bool RoboyBehaviorPlan::insertExecution(RoboyBehaviorExecution & execution) {
    qint64 frontOffset = execution.lTimestamp - this->m_startTimestamp;
    qint64 backOffset = this->m_endTimestamp - execution.getEndTimestamp();
    qint64 duration = execution.behavior.getDuration();

    PLAN_DBG << "Insert Execution:";
    PLAN_DBG << "\tBehaviorName: " << execution.behavior.m_metadata.m_sBehaviorName;
    PLAN_DBG << "\tFront Offset: " << frontOffset;
    PLAN_DBG << "\tBack Offset: "  << backOffset;
    PLAN_DBG << "\tDuration: "     << duration;

    // Try to insert every Trajectory of execution
    for(qint32 motorId : execution.behavior.m_mapMotorTrajectory.keys()) {
        // Check for conflicting ControlModes
        auto currentExecutionMode = execution.behavior.m_mapMotorTrajectory[motorId].m_controlMode;
        auto & globalTrajectoryMode = m_mapMotorTrajectories[motorId].m_controlMode;

        if(globalTrajectoryMode == ControlMode::UNDEFINED_CONTROL) {
            globalTrajectoryMode = currentExecutionMode;
        } else if (globalTrajectoryMode != currentExecutionMode) {
            PLAN_WAR << "ERROR: Conflicting ControlModes";
            return false;
        }

        // Calculate Start Offset
        qint32 wpOffset = frontOffset / 100;
        qint32 waypointCount = execution.behavior.m_mapMotorTrajectory[motorId].m_listWaypoints.length();
        // Try to insert every waypoint of trajectory
        for(int i = 0; i < waypointCount; i++) {
            RoboyWaypoint & currentWaypoint = m_mapMotorTrajectories[motorId].m_listWaypoints[i+wpOffset];
            RoboyWaypoint & insertWaypoint = execution.behavior.m_mapMotorTrajectory[motorId].m_listWaypoints[i];

            if(!(currentWaypoint.m_ulValue == 0xffffffffffffffff)) {
                PLAN_DBG << "ERROR: Overlapping Trajectories. Abort.";
                return false;
            } else {
                currentWaypoint.m_ulValue = insertWaypoint.m_ulValue;
            }
        }
    }

    return true;
}

void RoboyBehaviorPlan::printMap() const {
    for(qint32 motorId : m_mapMotorTrajectories.keys()) {
        QString line;
        line.sprintf("M%i\t:", motorId);
        for(RoboyWaypoint wp : m_mapMotorTrajectories[motorId].m_listWaypoints) {
            if(wp.m_ulValue == 0xffffffffffffffff) {
                line.append(" - ");
            } else {
                line.append(" X ");
            }
        }
        PLAN_DBG << line;
    }
}

ROSController::~ROSController() {
    if(m_communication != nullptr){
        delete m_communication;
    }
}