//
// Created by bruh on 1/21/16.
//

#include <communication/ROSControllerCommunication.h>
#include "DataPool.h"
#include "MyoController.h"
#include "RoboyController.h"

MyoController::MyoController() {
    MYOCONTROLLER_DBG << "Initialize Myo-Controller";
    initializeControllerMap();
}

MyoController::~MyoController() {
    delete m_myoMasterTransceiver;

    for(auto controller : m_mapControllers) {
        delete controller->m_communication;
        delete controller;
    }
}

// RoboyController Interface
bool MyoController::handleEvent_initializeControllers() {
    MYOCONTROLLER_DBG << "Send Initialize Request to Myo-Master";

    if(m_bInitialized)
        m_myoMasterTransceiver->stopControllers(m_mapControllers.keys());

    bool result = false;
    m_myoMasterTransceiver->sendInitializeRequest(m_mapControllers.values());

    if(waitForControllerStatus(m_mapControllers.keys(), ControllerState::INITIALIZED)){
        MYOCONTROLLER_SUC << "Initialization Complete!";
        MYOCONTROLLER_DBG << "Start Controllers.";
        m_myoMasterTransceiver->startControllers(m_mapControllers.keys());
        DataPool::getInstance()->setPlayerState(PlayerState::PLAYER_READY);
        result = true;
        m_bInitialized = true;
    } else {
        MYOCONTROLLER_WAR << "Initialization timed out.";
        DataPool::getInstance()->setPlayerState(PlayerState::PLAYER_NOT_READY);
        result = false;
        m_bInitialized = false;
    }

    return result;
}

bool MyoController::handleEvent_preprocessRoboyPlan(RoboyBehaviorPlan & behaviorPlan) {
    bool result = false;
    MYOCONTROLLER_DBG << "Get Flattended Trajectories";

    DataPool::getInstance()->setPlayerState(PlayerState::PLAYER_PREPROCESSING);

    QMap<qint32, Trajectory> mapTrajectories = behaviorPlan.getTrajectories();

    for(qint32 id : mapTrajectories.keys()) {
        ROSController * controller = m_mapControllers[id];
        controller->m_state = ControllerState::PREPROCESS_TRAJECTORY;
        controller->m_communication->sendTrajectory(mapTrajectories.value(id));
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
}

bool MyoController::handleEvent_playPlanExecution() {
    m_myoMasterTransceiver->sendSteeringMessage(SteeringCommand::PLAY_TRAJECTORY);
    DataPool::getInstance()->setPlayerState(PlayerState::PLAYER_PLAYING);
}

bool MyoController::handleEvent_pausePlanExecution() {
    m_myoMasterTransceiver->sendSteeringMessage(SteeringCommand::PAUSE_TRAJECTORY);
    DataPool::getInstance()->setPlayerState(PlayerState::PLAYER_PAUSED);
}

bool MyoController::handleEvent_stopPlanExecution() {
    m_myoMasterTransceiver->sendSteeringMessage(SteeringCommand::STOP_TRAJECTORY);
    DataPool::getInstance()->setPlayerState(PlayerState::PLAYER_TRAJECTORY_READY);
}

bool MyoController::handleEvent_recordBehavior() {
    m_myoMasterTransceiver->startRecording(m_mapControllers.values());
    DataPool::getInstance()->setRecorderState(RecorderState::RECORDER_RECORDING);
}

bool MyoController::handleEvent_stopRecording() {
    m_myoMasterTransceiver->sendRecordSteeringMessage(SteeringCommand::STOP_TRAJECTORY);
}

// Communication Feedback-Slots
void MyoController::slotControllerStatusUpdated(qint32 motorId) {
    m_mutexCVController.lock();
    m_conditionStatusUpdated.wakeAll();
    m_mutexCVController.unlock();
    DataPool::getInstance()->setControllerState(motorId, m_mapControllers[motorId]->m_state);
}

void MyoController::slotRecordFinished(bool result) {
    MYOCONTROLLER_DBG << "Record finished with status " << result;
    RoboyBehavior * behavior = m_myoMasterTransceiver->getRecordedBehavior();
    for(auto id : behavior->m_mapMotorTrajectory.keys()) {
        MYOCONTROLLER_DBG << "Trajectory: " << "motor-id: " << id << " wp-count: " << behavior->m_mapMotorTrajectory[id].m_listWaypoints.size();
    }

    DataPool::getInstance()->setRecorderState(RecorderState::RECORDER_FINISHED_RECORDING);
    DataPool::getInstance()->setRecordResult(result, behavior);
}

// private
void MyoController::initializeControllerMap() {
    // Create Myo-Master Transceiver
    QString name;
    name.sprintf("myomaster");
    m_myoMasterTransceiver = new ROSMasterCommunication();
    m_myoMasterTransceiver->start();

    connect(m_myoMasterTransceiver, SIGNAL(signalRecordFinished(bool)), this, SLOT(slotRecordFinished(bool)));

    // Create Controller Status Structs
    MYOCONTROLLER_DBG << "Instantiate ROSControllers:";
    QList<ROSController> controllers = RoboyControlConfiguration::instance().getControllersConfig();
    for(ROSController & c : controllers) {
        ROSController * controller = new ROSController();
        controller->m_id = c.m_id;
        controller->m_state = ControllerState::UNDEFINED;
        controller->m_controlMode = c.m_controlMode;
        controller->m_communication = new ROSControllerCommunication(controller);
        controller->m_communication->start();
        m_mapControllers.insert(controller->m_id, controller);
        DataPool::getInstance()->setControllerState(c.m_id, ControllerState::UNDEFINED);
        connect(controller->m_communication, SIGNAL(signalControllerStatusUpdated(qint32)), this, SLOT(slotControllerStatusUpdated(qint32)));
    }

}

bool MyoController::waitForControllerStatus(QList<qint32> idList, ControllerState state, quint32 timeout) {
    // TODO: Implement Timeout
    while(!checkControllersForState(idList, state)) {
        MYOCONTROLLER_DBG << "Controllers not ready. Wait.";
        m_mutexCVController.lock();
        m_conditionStatusUpdated.wait(&m_mutexCVController, 5000);
        m_mutexCVController.unlock();
    }
    return true;
}

bool MyoController::checkControllersForState(QList<qint32> idList, ControllerState state) {
    for(auto id : idList)
        if (m_mapControllers[id]->m_state != state)
            return false;

    return true;
}