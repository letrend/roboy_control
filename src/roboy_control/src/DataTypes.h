#ifndef DATATYPES_H
#define DATATYPES_H

#include "CommonDefinitions.h"

#include <QString>
#include <QMap>

class IModelService;
class ITransceiverService;

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

    quint64 getDuration() {
        int maxDuration = 0;
        int currentDuration = 0;
        for (Trajectory trajectory : m_mapMotorTrajectory) {
            currentDuration = trajectory.getDuration();
            currentDuration > maxDuration ? maxDuration = currentDuration : maxDuration;
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
    QList<RoboyBehaviorExecution>       listExecutions;

    qint64  lStartTimestamp = -1;
    qint64  lEndTimestamp = -1;

public:
    RoboyBehaviorPlan(IModelService * modelService, const RoboyBehaviorMetaplan & metaPlan);

    qint64 getStartTimestamp();
    qint64 getEndTimestamp();
    qint64 getDuration();

    const QList<RoboyBehaviorExecution> & getExcutionsList() const;

    QMap<qint32, Trajectory> getTrajectories() const;
};

struct ROSController {
    qint8                 id;
    STATUS                state;
    ITransceiverService * transceiver;

    QString toString() const {
        QString string;
        string.sprintf("ROSController: [id:%i][state:%i]", id, state);
        return string;
    }
};

#endif // DATATYPES_H
