#ifndef DATATYPES_H
#define DATATYPES_H

#include <QString>
#include <QMap>

struct RoboyBehaviorMetadata {
    quint64   m_ulBehaviorId;
    QString     m_sBehaviorName;
};

struct RoboyWaypoint {
    quint64   m_ulId;
    quint64   m_ulTimestamp;
    quint64   m_ulPosition;
};

struct RoboyBehavior {
    RoboyBehaviorMetadata m_metadata;
    QMap<u_int32_t, QList<RoboyWaypoint>> m_mapMotorWaypoints;

    QString toString() {
        QString string;
        string.sprintf("ROBOY BEHAVIOR: "
                    "%s\tId:%lu\tMotor Count:%i",
                    m_metadata.m_sBehaviorName.toLatin1().data(),
                    (unsigned long) m_metadata.m_ulBehaviorId,
                    m_mapMotorWaypoints.count());

        return string;
    }

    quint64 getDuration() {
        quint64 firstTs = 0;
        quint64 lastTs  = 0;
        for (QList<RoboyWaypoint> trajectory : m_mapMotorWaypoints) {
            if (trajectory.first().m_ulTimestamp < firstTs) {
                firstTs = trajectory.first().m_ulTimestamp;
            }
            if (trajectory.last().m_ulTimestamp > lastTs) {
                lastTs = trajectory.last().m_ulTimestamp;
            }
        }
        return lastTs-firstTs;
    }
};

struct RoboyBehaviorExecution {
    quint64          lId;
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
