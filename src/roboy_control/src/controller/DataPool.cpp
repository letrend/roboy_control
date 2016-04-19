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

void DataPool::insertController(const ROSController & controller) {
    if(!m_controllerStates.contains(controller.m_id)) {
        m_controllerStates.insert(controller.m_id, controller);
    }
}


void DataPool::reset() {
    m_playerState = PlayerState::PLAYER_NOT_READY;
    m_recorderState = RecorderState::RECORDER_NOT_READY;

    m_controllerStates.clear();
    m_recordResult = false;

    emit signalNotifyOnDataPoolReset();
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
        ControllerState previousState = m_controllerStates[id].m_state;
        m_controllerStates[id].m_state = state;
        update = previousState == state ? false : true;
    } else {
        ROSController controller;
        controller.m_id = id;
        controller.m_state = state;
        controller.m_controlMode = ControlMode::UNDEFINED_CONTROL;
        m_controllerStates.insert(id, controller);
        update = true;
    }
    m_mutexData.unlock();

    if(update)
        emit signalNotifyOnControllerStateUpdated(id);
}

void DataPool::setSampleRate(qint32 sampleRate) {
    m_sampleRate = sampleRate;
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
        return m_controllerStates[motorId].m_state;
    else
        return ControllerState::UNDEFINED;
}

ControlMode DataPool::getControlMode(qint32 motorId) {
    if(m_controllerStates.contains(motorId))
        return m_controllerStates[motorId].m_controlMode;
    else
        return ControlMode::UNDEFINED_CONTROL;
}


float DataPool::getSampleRate() {
    return m_sampleRate;
}


bool DataPool::getRecordResult() {
    return m_recordResult;
}

RoboyBehavior * DataPool::getRecordedBehavior() {
    return &m_recordBehavior;
}

