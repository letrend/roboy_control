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
};

struct RoboyBehaviorExecution {
    qint64          lId;
    quint64         ulTimestamp;
    RoboyBehavior   behavior;

};

struct RoboyBehaviorPlan {
    qint64      startTimestamp;
    qint64      stopTimestamp;
    qint64      duration;
    QList<RoboyBehaviorExecution>   listExecutions;
};

#endif // DATATYPES_H
