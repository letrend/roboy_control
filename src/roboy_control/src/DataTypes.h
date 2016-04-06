#ifndef DATATYPES_H
#define DATATYPES_H

#include "CommonDefinitions.h"

#include <QDebug>
#include <QString>
#include <QMap>

class IModelService;
class IMasterCommunication;
class IControllerCommunication;

enum PlayerState {  PLAYER_UNINITIALIZED,
                    PLAYER_INITIALIZED,
                    PLAYER_PREPROCESSING,
                    PLAYER_TRAJECTORY_READY,
                    PLAYER_TRAJECTORY_FAILED,
                    PLAYER_PAUSED,
                    PLAYER_PLAYING};

struct RoboyBehaviorMetadata {
    quint64   m_ulBehaviorId;
    QString   m_sBehaviorName;
};

struct RoboyWaypoint {
    quint64   m_ulId;
    quint64   m_ulValue;
};

struct Trajectory {
    ControlMode          m_controlMode;
    qint32               m_sampleRate;
    QList<RoboyWaypoint> m_listWaypoints;
    qint32 getDuration() {
        return m_listWaypoints.count() * m_sampleRate;
    }

    QString toString() const {
        QString string;
        string.sprintf("TRAJECTORY [controlMode:%i][sampleRate:%i][wp-count:%i]",
                       (int) m_controlMode,
                       (int) m_sampleRate,
                       m_listWaypoints.size());
        return string;
    }
};

struct RoboyBehavior {
    RoboyBehaviorMetadata       m_metadata;
    QMap<qint32, Trajectory> m_mapMotorTrajectory;

    QString toString() {
        QString string;
        string.sprintf("ROBOY BEHAVIOR: "
                    "%s\tId:%lu\tMotor Count:%i",
                    m_metadata.m_sBehaviorName.toLatin1().data(),
                    (unsigned long) m_metadata.m_ulBehaviorId,
                    m_mapMotorTrajectory.count());

        return string;
    }

    quint64 getDuration() const {
        int maxDuration = 0;
        int currentDuration = 0;
        for (Trajectory trajectory : m_mapMotorTrajectory) {
            currentDuration = trajectory.getDuration();
            if(currentDuration > maxDuration)
                maxDuration = currentDuration;
        }
        return maxDuration;
    }
};

struct RoboyBehaviorMetaExecution {
    qint64                 lId;
    qint64                 lTimestamp;
    RoboyBehaviorMetadata   behaviorMetadata;
};

struct RoboyBehaviorExecution{
    qint64                  lId;
    qint64                  lTimestamp;
    RoboyBehavior           behavior;

    qint64 getEndTimestamp() {
        return lTimestamp + behavior.getDuration();
    }
};

struct RoboyBehaviorMetaplan {
    QList<RoboyBehaviorMetaExecution>   listExecutions;
};

struct RoboyBehaviorPlan {

private:
    QList<RoboyBehaviorExecution>       m_listExecutions;
    QMap<qint32, Trajectory>            m_mapMotorTrajectories;

    bool    m_isValid = false;

    qint64  m_startTimestamp = -1;
    qint64  m_endTimestamp = -1;

    ControlMode m_controlMode;
    qint32      m_sampleRate;


public:
    RoboyBehaviorPlan(IModelService * modelService, const RoboyBehaviorMetaplan & metaPlan);

    bool isValid() const;

    qint64 getStartTimestamp() const;
    qint64 getEndTimestamp() const;
    qint64 getDuration() const;

    const QList<RoboyBehaviorExecution> & getExecutionsList() const;

    QMap<qint32, Trajectory> getTrajectories() const;

private:
    void setStartTimestamp();
    void setEndTimestamp();

    bool doFlattening();
    bool insertExecution(RoboyBehaviorExecution & execution);

    void printMap() const;
};

struct ROSController {
    qint32                m_id;
    ControlMode           m_controlMode;
    ControllerState       m_state;

    IControllerCommunication * m_communication;

    QString toString() const {
        QString string;
        string.sprintf("ROSController: [id:%i][controlMode:%i][state:%i]", m_id, m_controlMode, m_state);
        return string;
    }
};

#endif // DATATYPES_H
