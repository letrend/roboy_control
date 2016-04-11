//
// Created by bruh on 4/11/16.
//

#include "DataPool.h"

DataPool * DataPool::instance = nullptr;

DataPool * DataPool::getInstance() {
    if(instance == nullptr) {
        instance = new DataPool();
    }
    return instance;
}

void DataPool::setControllerStatus(qint32 id, ControllerState state) {
    m_mutexData.lock();
    if(m_controllerStates.contains(id))
        m_controllerStates[id] = state;
    else
        m_controllerStates.insert(id, state);
    m_mutexData.unlock();

    qDebug() << "Emit 'ControllerStatusUpdated'";
    emit signalNotifyOnControllerStateUpdated();
}


void DataPool::setRecordResult(bool result, RoboyBehavior * pBehavior) {
    m_mutexData.lock();
    m_result = result;
    m_recordedBehavior = *pBehavior;
    m_mutexData.unlock();

    qDebug() << "Emit 'RecordResult'";
    emit signalNotifyOnRecordResult();
}
