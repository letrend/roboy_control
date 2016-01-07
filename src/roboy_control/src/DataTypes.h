#ifndef DATATYPES_H
#define DATATYPES_H

#include <QString>
#include <QMap>

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

struct RoboyBehaviorExecution {
    quint64         lId;
    quint64         ulTimestamp;
    RoboyBehavior   behavior;

};

struct RoboyBehaviorPlan {
    quint64      startTimestamp;
    quint64      stopTimestamp;
    quint64      duration;
    QList<RoboyBehaviorExecution>   listExecutions;
};

#endif // DATATYPES_H
