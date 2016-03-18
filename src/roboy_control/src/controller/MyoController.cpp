//
// Created by bruh on 1/21/16.
//

#include <communication/ROSControllerCommunication.h>
#include "MyoController.h"

MyoController::MyoController() {
    MYOCONTROLLER_DBG << "Initialize Myo-Controller";

    // Create Myo-Master Transceiver
    QString name;
    name.sprintf("myomaster");
    m_myoMasterTransceiver = new ROSMasterCommunication();
    m_myoMasterTransceiver->start();

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
        connect(controller->m_communication, SIGNAL(signalControllerStatusUpdated()), this, SLOT(slotControllerStatusUpdated()));
    }
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
    bool result = false;
    m_myoMasterTransceiver->sendInitializeRequest(m_mapControllers.values());

    if(waitForControllerStatus(m_mapControllers.keys(), ControllerState::INITIALIZED)){
        MYOCONTROLLER_SUC << "Initialization Complete!";
        MYOCONTROLLER_DBG << "Start Controllers.";
        m_myoMasterTransceiver->startControllers(m_mapControllers.keys());
        result = true;
    } else {
        MYOCONTROLLER_WAR << "Initialization timed out.";
        result = false;
    }

    return result;
}

bool MyoController::handleEvent_preprocessRoboyPlan(RoboyBehaviorPlan & behaviorPlan) {
    bool result = false;
    MYOCONTROLLER_DBG << "Get Flattended Trajectories";
    QMap<qint32, Trajectory> mapTrajectories = behaviorPlan.getTrajectories();

    for(qint32 id : mapTrajectories.keys()) {
        ROSController * controller = m_mapControllers[id];
        controller->m_state = ControllerState::PREPROCESS_TRAJECTORY;
        controller->m_communication->sendTrajectory(mapTrajectories.value(id));
    }

    if(waitForControllerStatus(mapTrajectories.keys(), ControllerState::TRAJECTORY_READY)) {
        MYOCONTROLLER_SUC << "All Controllers ready to play Trajectory.";
        result = true;
    } else {
        MYOCONTROLLER_WAR << "Preprocessing of Trajectories timed out.";
        result = false;
    }
    return result;
}

bool MyoController::handleEvent_playPlanExecution() {
    m_myoMasterTransceiver->sendSteeringMessage(SteeringCommand::PLAY_TRAJECTORY);
}

bool MyoController::handleEvent_pausePlanExecution() {
    m_myoMasterTransceiver->sendSteeringMessage(SteeringCommand::PAUSE_TRAJECTORY);
}

bool MyoController::handleEvent_stopPlanExecution() {
    m_myoMasterTransceiver->sendSteeringMessage(SteeringCommand::STOP_TRAJECTORY);
}

bool MyoController::handleEvent_recordBehavior() {
    m_myoMasterTransceiver->startRecording(m_mapControllers.values());
}

bool MyoController::handleEvent_stopRecording() {
    m_myoMasterTransceiver->sendRecordSteeringMessage(SteeringCommand::STOP_TRAJECTORY);
}

// Slots
void MyoController::slotControllerStatusUpdated() {
    m_mutexCVController.lock();
    m_conditionStatusUpdated.wakeAll();
    m_mutexCVController.unlock();
}

// private
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