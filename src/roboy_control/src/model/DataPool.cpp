//
// Created by bruh on 4/11/16.
//

#include "DataPool.h"

#include "RoboyControlConfiguration.h"
#include "ROSMyoMaster.h"
#include "ROSMotorController.h"

void DataPool::initializeDataPool() {
    reset();

    m_myoMaster = new ROSMyoMaster();

    RoboyControlConfiguration::instance().reloadConfig();
    const QList<MotorControllerConfig> & controllers = RoboyControlConfiguration::instance().getMotorConfig();
    for(auto c : controllers) {
        IMotorController * controller = new ROSMotorController(c.id, c.controlMode);
        if(m_motorControllers.contains(c.id))
            DATAPOOL_WAR << "Motor controller with id: " << c.id << " already exists -> replace";
        m_motorControllers.insert(c.id, controller);
    }
}

void DataPool::setMotorControllerState(qint32 id, ControllerState state) {
    if(!m_motorControllers.contains(id)) {
        DATAPOOL_WAR << "Try to updated not tracked controller: " << id;
    } else if(m_motorControllers[id]->getState() != state) {
        m_motorControllers[id]->setState(state);

        updateApplicationStates(state);

        emit signalNotifyOnControllerStateUpdated(id);
    }
}

ControllerState DataPool::getMotorControllerState(qint32 id) const {
    if(m_motorControllers.contains(id))
        return m_motorControllers[id]->getState();
    else
        DATAPOOL_WAR << "Try to access not tracked controller: " << id;
    return ControllerState::UNDEFINED;
}

ControlMode DataPool::getMotorControlMode(qint32 id) const {
    return m_motorControllers[id]->getControlMode();
}

IMyoMaster * DataPool::getMyoMaster() const {
    return m_myoMaster;
}

const QList<IMotorController *> DataPool::getMotorControllers() const {
    return m_motorControllers.values();
}

IMotorController * DataPool::getMotorController(qint32 id) const {
    return m_motorControllers[id];
}

void DataPool::setPlayerState(PlayerState state) {
    m_playerState = state;
    emit signalNotifyOnPlayerStateUpdated();
}

PlayerState DataPool::getPlayerState() const {
    return m_playerState;
}

void DataPool::setCurrentBehaviorPlan(const RoboyBehaviorPlan * plan) {
    m_currentRoboyPlan = plan;
}

// private interface
void DataPool::reset() {
    m_playerState   = PlayerState::PLAYER_NOT_READY;
    m_recorderState = RecorderState::RECORDER_NOT_READY;

    if(m_myoMaster != nullptr)
        delete m_myoMaster;

    for(IMotorController * c : m_motorControllers) {
        if(c != nullptr)
            delete c;
    }

    m_motorControllers.clear();
    m_recordResult = false;

    emit signalNotifyOnDataPoolReset();
}

void DataPool::updateApplicationStates(ControllerState state) {
    if(state == ControllerState::INITIALIZED) {
        if(checkControllersForState(m_motorControllers.keys(), state))
            setPlayerState(PlayerState::PLAYER_READY);
    } else if (state == ControllerState::TRAJECTORY_READY) {
        if (checkControllersForState(m_currentRoboyPlan->getTrajectories().keys(), state))
            setPlayerState(PlayerState::PLAYER_TRAJECTORY_READY);
    }
}

bool DataPool::checkControllersForState(const QList<qint32> idList, ControllerState state) const {
    for(auto id : idList)
        if (m_motorControllers[id]->getState() != state)
            return false;

    return true;
}


/*
void DataPool::setPlayerState(PlayerState state) {
    m_mutexData.lock();
    m_playerState = state;
    m_mutexData.unlock();
    emit signalNotifyOnPlayerStateUpdated();
}

void DataPool::setRecorderState(RecorderState state) {
    qDebug() << "Update Recorder State:" << state;
    m_mutexData.lock();
    m_recorderState = state;
    m_mutexData.unlock();
    emit signalNotifyOnRecorderStateUpdated();
}

void DataPool::setControllerState(qint32 id, ControllerState state) {
    bool update = false;
    m_mutexData.lock();
    if(m_motorControllers.contains(id)) {
        ControllerState previousState = m_motorControllers[id].m_state;
        m_motorControllers[id].m_state = state;
        update = previousState == state ? false : true;
    } else {
        MotorController controller;
        controller.m_id = id;
        controller.m_state = state;
        controller.m_controlMode = ControlMode::UNDEFINED_CONTROL;
        m_motorControllers.insert(id, controller);
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
    if(m_motorControllers.contains(motorId))
        return m_motorControllers[motorId].m_state;
    else
        return ControllerState::UNDEFINED;
}

ControlMode DataPool::getControlMode(qint32 motorId) {
    if(m_motorControllers.contains(motorId))
        return m_motorControllers[motorId].m_controlMode;
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

         if(waitForControllerStatus(mapTrajectories.keys(), ControllerState::TRAJECTORY_READY)) {
            MYOCONTROLLER_SUC << "All Controllers ready to play Trajectory.";
            DataPool::getInstance()->setPlayerState(PlayerState::PLAYER_PREPROCESS_SUCCEEDED);
            result = true;
        } else {
            MYOCONTROLLER_WAR << "Preprocessing of Trajectories timed out.";
            DataPool::getInstance()->setPlayerState(PlayerState::PLAYER_PREPROCESS_FAILED_COMMUNICATION_TIMEOUT);
            result = false;
        }
        return result;




*/