//
// Created by bruh on 4/11/16.
//

#ifndef ROBOY_CONTROL_DATAPOOL_H
#define ROBOY_CONTROL_DATAPOOL_H

#include "DataTypes.h"
#include <QMutex>

class DataPool : QObject {

    Q_OBJECT

private:
    static DataPool * instance;

    QMutex          m_mutexData;

private:
    DataPool() { }

    /*
    RoboyState
    PlayerState
    RecordState
    */

    QMap<qint32, ControllerState> m_controllerStates;

    // Record Results
    bool           m_result;
    RoboyBehavior  m_recordedBehavior;

public:
    static DataPool * getInstance();

    void setControllerStatus(qint32 id, ControllerState state);

    void setRecordResult(bool result, RoboyBehavior * pBehavior);

signals:
    void signalNotifyOnControllerStateUpdated();
    void signalNotifyOnRecordResult();
};

#endif //ROBOY_CONTROL_DATAPOOL_H
