//
// Created by bruh on 1/21/16.
//

#include <ROSMotorController.h>

#include "DataPool.h"
#include "MyoController.h"
#include "RoboyController.h"

MyoController::MyoController() {
    MYOCONTROLLER_DBG << "Initialize DataPool";
    DataPool::instance().initializeDataPool();
}

MyoController::~MyoController() {
}

// RoboyController Interface
bool MyoController::handleEvent_initializeControllers() const {
    MYOCONTROLLER_DBG << "Send Initialize Request to Myo-Master";

    bool result = false;
//    m_myoMasterTransceiver->sendInitializeRequest(m_mapControllers.values());
//
//    if(waitForControllerStatus(m_mapControllers.keys(), ControllerState::INITIALIZED)){
//        MYOCONTROLLER_SUC << "Initialization Complete!";
//        MYOCONTROLLER_DBG << "Start Controllers.";
////       m_myoMasterTransceiver->startControllers(m_mapControllers.keys());
//        DataPool::getInstance()->setPlayerState(PlayerState::PLAYER_READY);
//        DataPool::getInstance()->setRecorderState(RecorderState::RECORDER_READY);
//        result = true;
//    } else {
//        MYOCONTROLLER_WAR << "Initialization timed out.";
//        DataPool::getInstance()->setPlayerState(PlayerState::PLAYER_NOT_READY);
//        DataPool::getInstance()->setRecorderState(RecorderState::RECORDER_NOT_READY);
//        result = false;
//    }

    return result;
}

bool MyoController::handleEvent_preprocessRoboyPlan(RoboyBehaviorPlan & behaviorPlan) const {
    bool result = false;
    MYOCONTROLLER_DBG << "Get Flattended Trajectories";

//    QMap<qint32, Trajectory> mapTrajectories = behaviorPlan.getTrajectories();
//
//    for(qint32 id : mapTrajectories.keys()) {
//        // Check ControlModes
//        if (m_mapControllers[id]->m_controlMode != mapTrajectories[id].m_controlMode) {
//            MYOCONTROLLER_WAR << "Controller " << id << " not in respective mode to execute behavior";
//            DataPool::getInstance()->setPlayerState(PlayerState::PLAYER_PREPROCESS_FAILED_MODE_CONFLICT);
//            return false;
//        }
//    }
//    for(qint32 id : mapTrajectories.keys()) {
//        MotorController * controller = m_mapControllers[id];
//        controller->m_state = ControllerState::PREPROCESS_TRAJECTORY;
//        controller->m_communication->sendTrajectory(mapTrajectories.value(id));
//    }
//
//    if(waitForControllerStatus(mapTrajectories.keys(), ControllerState::TRAJECTORY_READY)) {
//        MYOCONTROLLER_SUC << "All Controllers ready to play Trajectory.";
//        DataPool::getInstance()->setPlayerState(PlayerState::PLAYER_PREPROCESS_SUCCEEDED);
//        result = true;
//    } else {
//        MYOCONTROLLER_WAR << "Preprocessing of Trajectories timed out.";
//        DataPool::getInstance()->setPlayerState(PlayerState::PLAYER_PREPROCESS_FAILED_COMMUNICATION_TIMEOUT);
//        result = false;
//    }
    return result;
}

bool MyoController::handleEvent_playPlanExecution() const {
//    m_myoMasterTransceiver->sendSteeringMessage(SteeringCommand::PLAY_TRAJECTORY);
//     DataPool::getInstance()->setPlayerState(PlayerState::PLAYER_PLAYING);
}

bool MyoController::handleEvent_pausePlanExecution() const {
//    m_myoMasterTransceiver->sendSteeringMessage(SteeringCommand::PAUSE_TRAJECTORY);
//    DataPool::getInstance()->setPlayerState(PlayerState::PLAYER_PAUSED);
}

bool MyoController::handleEvent_stopPlanExecution() const {
//    m_myoMasterTransceiver->sendSteeringMessage(SteeringCommand::STOP_TRAJECTORY);
//    DataPool::getInstance()->setPlayerState(PlayerState::PLAYER_TRAJECTORY_READY);
}

bool MyoController::handleEvent_recordBehavior() const {
//    DataPool::getInstance()->setRecorderState(RecorderState::RECORDER_RECORDING);
//    m_myoMasterTransceiver->startRecording(m_mapControllers, DataPool::getInstance()->getSampleRate());
}

bool MyoController::handleEvent_stopRecording() const {
//    m_myoMasterTransceiver->sendRecordSteeringMessage(SteeringCommand::STOP_TRAJECTORY);
}

/*

// Communication Feedback-Slots
void MyoController::slotControllerStatusUpdated(qint32 motorId) {
    m_mutexCVController.lock();
    m_conditionStatusUpdated.wakeAll();
    m_mutexCVController.unlock();
    DataPool::getInstance()->setControllerState(motorId, m_mapControllers[motorId]->m_state);
}

void MyoController::slotRecordFinished(bool result) {
    MYOCONTROLLER_DBG << "Record finished with status " << result;
//    RoboyBehavior * behavior = m_myoMasterTransceiver->getRecordedBehavior();
    RoboyBehavior * behavior;
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
    m_myoMasterTransceiver = new ROSMyoMaster();
    //m_myoMasterTransceiver->start();

//    connect(m_myoMasterTransceiver, SIGNAL(signalRecordFinished(bool)), this, SLOT(slotRecordFinished(bool)));

    // Create Controller Status Structs
    MYOCONTROLLER_DBG << "Instantiate ROSControllers:";
    RoboyControlConfiguration::instance().update();
    QList<MotorController> controllers = RoboyControlConfiguration::instance().getMotorConfig();
    for(MotorController & c : controllers) {
        MotorController * controller = new MotorController();
        controller->m_id = c.m_id;
        controller->m_state = ControllerState::UNDEFINED;
        controller->m_controlMode = c.m_controlMode;
        DataPool::getInstance()->insertController(*controller);
        controller->m_communication = new ROSMotorController(0, (FORCE_CONTROL));
//        controller->m_communication->start();
        m_mapControllers.insert(controller->m_id, controller);
        DataPool::getInstance()->setControllerState(c.m_id, ControllerState::UNDEFINED);
//        connect(controller->m_communication, SIGNAL(signalControllerStatusUpdated(qint32)), this, SLOT(slotControllerStatusUpdated(qint32)));
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

 */