#ifndef DATATYPES_H
#define DATATYPES_H

#include <QString>
#include <QMap>

class XmlModelService;

enum ControlMode {POSITION_CONTROL, FORCE_CONTROL};

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
    qint64               m_sampleRate;
    QList<RoboyWaypoint> m_listWaypoints;

    qint32 getDuration() {
        return m_listWaypoints.count() * m_sampleRate;
    }
};

struct RoboyBehavior {
    RoboyBehaviorMetadata       m_metadata;
    QMap<u_int32_t, Trajectory> m_mapMotorTrajectory;

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
    qint64  lDuration = -1;

public:
    RoboyBehaviorPlan() {

    }

    RoboyBehaviorPlan(const XmlModelService * modelService, const RoboyBehaviorMetaplan & metaPlan) {
        // TODO: Do initialiation
    }

    qint64 getStartTimestamp() {
        if (lStartTimestamp == -1) {
            qint64 currentTimestamp = -1;
            for (RoboyBehaviorExecution exec : listExecutions) {
                currentTimestamp = exec.lTimestamp;
                currentTimestamp < lStartTimestamp ? lStartTimestamp = currentTimestamp : lStartTimestamp;
            }
        }
        return lStartTimestamp;
    }

    qint64 getEndTimestamp() {
        if (lEndTimestamp == -1) {
            qint64 currentEndTimestamp = -1;
            for (RoboyBehaviorExecution exec : listExecutions) {
                currentEndTimestamp = exec.getEndTimestamp();
                currentEndTimestamp > lEndTimestamp ? lEndTimestamp = currentEndTimestamp : lEndTimestamp;
            }
        }
        return lEndTimestamp;
    }

    qint64 getDuration() {
        return getEndTimestamp() - getStartTimestamp();
    }

};

#endif // DATATYPES_H
