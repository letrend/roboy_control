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
        string.sprintf("ROBOY BEHAVIOR\n"
                    "\t- Name:\t\t%s\n"
                    "\t- Id:\t\t%lu \n"
                    "\t- Motor Count:\t%i\n",
                    m_metadata.m_sBehaviorName.toLatin1().data(),
                    (unsigned long) m_metadata.m_ulBehaviorId,
                    m_mapMotorWaypoints.count());

        return string;
    }
};

#endif // DATATYPES_H
