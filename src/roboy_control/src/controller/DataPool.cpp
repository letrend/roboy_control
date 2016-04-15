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

void DataPool::setPlayerState(PlayerState state) {
    m_mutexData.lock();
    m_playerState = state;
    m_mutexData.unlock();
    emit signalNotifyOnPlayerStateUpdated();
}

void DataPool::setRecorderState(RecorderState state) {
    m_mutexData.lock();
    m_recorderState = state;
    m_mutexData.unlock();
    emit signalNotifyOnRecorderStateUpdated();
}

void DataPool::setControllerState(qint32 id, ControllerState state) {
    bool update = false;
    m_mutexData.lock();
    if(m_controllerStates.contains(id)) {
        ControllerState previousState = m_controllerStates[id];
        m_controllerStates[id] = state;
        update = previousState == state ? false : true;
    } else {
        m_controllerStates.insert(id, state);
    }
    m_mutexData.unlock();

    if(update)
        emit signalNotifyOnControllerStateUpdated(id);
}


void DataPool::setRecordResult(bool result, RoboyBehavior * pBehavior) {
    m_mutexData.lock();
    m_recordResult = result;
    m_recordBehavior = *pBehavior;
    m_mutexData.unlock();
    emit signalNotifyOnRecordResult();
}

PlayerState DataPool::getPlayerState() {
    return m_playerState;
}

RecorderState DataPool::getRecorderState() {
    return m_recorderState;
}

ControllerState DataPool::getControllerState(qint32 motorId) {
    if(m_controllerStates.contains(motorId))
        return m_controllerStates[motorId];
    else
        return ControllerState::UNDEFINED;
}

bool DataPool::getRecordResult() {
    return m_recordResult;
}

RoboyBehavior * DataPool::getRecordedBehavior() {
    return &m_recordBehavior;
}

