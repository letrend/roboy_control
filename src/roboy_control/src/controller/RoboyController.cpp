//
// Created by bruh on 30.11.15.
//

#include <ros/spinner.h>
#include "RoboyController.h"

#include "DataPool.h"
#include "MyoController.h"
#include "ViewController.h"
#include "XmlModelService.h"

RoboyController::RoboyController() {
    m_pModelService = new XmlModelService();
    m_pViewController = new ViewController(m_pModelService);

    connect(m_pViewController, SIGNAL(signalInitialize()), this, SLOT(slotInitializeRoboy()));
    connect(m_pViewController, SIGNAL(signalPreprocess()), this, SLOT(slotPreprocessPlan()));

    connect(m_pViewController, SIGNAL(signalPlay()), this, SLOT(slotPlayPlan()));
    connect(m_pViewController, SIGNAL(signalStop()), this, SLOT(slotStopPlan()));
    connect(m_pViewController, SIGNAL(signalPause()), this, SLOT(slotPausePlan()));

    connect(m_pViewController, SIGNAL(signalRecord()), this, SLOT(slotRecordBehavior()));
    connect(m_pViewController, SIGNAL(signalStopRecording()), this, SLOT(slotStopRecording()));
}

RoboyController::~RoboyController() {
    delete m_pMyoController;
    delete m_pModelService;
    delete m_pViewController;

    if (this->isFinished())
        CONTROLLER_DBG << "Thread terminated";
}

void RoboyController::run() {
    CONTROLLER_DBG << "Controller Thread Started. ID=" << currentThreadId();

    // RoboyController handles only events coming in as signals
    exec();

    CONTROLLER_DBG << "Controller Thread Interrupted. Quit.";
}

// Slots
void RoboyController::slotInitializeRoboy() {
    CONTROLLER_DBG << "\n\n";
    CONTROLLER_SUC << "---------------- EVENT: Initialize ------------------";

    DataPool::instance().initializeDataPool();
    msleep(1000);
    DataPool::instance().getMyoMaster()->sendInitializeRequest(DataPool::instance().getMotorControllers());

    CONTROLLER_SUC << "------------------ /EVENT: Initialize -----------------\n\n";
}

void RoboyController::slotPreprocessPlan() {
    CONTROLLER_DBG << "\n\n";
    CONTROLLER_SUC << "---------------- EVENT: Preprocess ------------------";
    preprocessCurrentRoboyPlan();
    CONTROLLER_SUC << "------------------ /EVENT: Preprocess -----------------\n\n";
}

void RoboyController::slotPlayPlan() {
    CONTROLLER_SUC << "---------------- EVENT: Play ------------------";
    DataPool::instance().getMyoMaster()->sendSteeringMessage(SteeringCommand::PLAY_TRAJECTORY);
//     DataPool::getInstance()->setPlayerState(PlayerState::PLAYER_PLAYING);
    CONTROLLER_SUC << "<3";
    CONTROLLER_SUC << "------------------ /EVENT: Play -----------------\n\n";
}

void RoboyController::slotStopPlan() {
    CONTROLLER_DBG << "Triggered 'Stop Execution' from View";
    DataPool::instance().getMyoMaster()->sendSteeringMessage(SteeringCommand::STOP_TRAJECTORY);
//    DataPool::getInstance()->setPlayerState(PlayerState::PLAYER_TRAJECTORY_READY);
}

void RoboyController::slotPausePlan() {
    CONTROLLER_DBG << "Triggered 'Pause Execution' from View";
    DataPool::instance().getMyoMaster()->sendSteeringMessage(SteeringCommand::PAUSE_TRAJECTORY);
//    DataPool::getInstance()->setPlayerState(PlayerState::PLAYER_PAUSED);
}

void RoboyController::slotRecordBehavior() {
    CONTROLLER_DBG << "Triggered 'Record Behavior' from View";
//    DataPool::getInstance()->setRecorderState(RecorderState::RECORDER_RECORDING);
//    m_myoMasterTransceiver->startRecording(m_mapControllers, DataPool::getInstance()->getSampleRate());
}

void RoboyController::slotStopRecording() {
    CONTROLLER_DBG << "Triggered 'Stop Recording' from View";
//    m_myoMasterTransceiver->sendRecordSteeringMessage(SteeringCommand::STOP_TRAJECTORY);
}

// Private Interface
void RoboyController::preprocessCurrentRoboyPlan() {
    DataPool::instance().setPlayerState(PlayerState::PLAYER_PREPROCESSING);

    // TODO: Get Metaplan from DataPool
    CONTROLLER_DBG << "Get Metaplan from ViewController";
    RoboyBehaviorMetaplan metaplan = m_pViewController->fromController_getCurrentRoboyPlan();

    CONTROLLER_DBG << "Build BehaviorPlan";
    RoboyBehaviorPlan * plan = new RoboyBehaviorPlan(m_pModelService, metaplan);

    DataPool::instance().setCurrentBehaviorPlan(plan);

    // Check Empty (no behavior)
    if(plan->isEmpty()) {
        DataPool::instance().setPlayerState(PlayerState::PLAYER_PREPROCESS_FAILED_EMPTY);
        return;
    }

    // Check if behaviors found in DB
    if(!plan->isLoadedCompletely()) {
        DataPool::instance().setPlayerState(PlayerState::PLAYER_PREPROCESS_FAILED_LOAD_BEHAVIOR);
        return;
    }

    // TODO: Check if Controllers exist

    if(plan->doFlattening()) {
        MYOCONTROLLER_DBG << "Get Flattended Trajectories";

        QMap<qint32, Trajectory> mapTrajectories = plan->getTrajectories();

        for(qint32 id : mapTrajectories.keys()) {
            // Check ControlModes
            if (DataPool::instance().getMotorControlMode(id) != mapTrajectories[id].m_controlMode) {
                MYOCONTROLLER_WAR << "Controller " << id << " not in respective mode to execute behavior";
                DataPool::instance().setPlayerState(PlayerState::PLAYER_PREPROCESS_FAILED_MODE_CONFLICT);
                return;
            }
        }

        for(qint32 id : mapTrajectories.keys()) {
            IMotorController * controller = DataPool::instance().getMotorController(id);
            controller->sendTrajectory(mapTrajectories[id]);
        }
    } else {
        CONTROLLER_WAR << "Flattening of Plan failed.";
        DataPool::instance().setPlayerState(PlayerState::PLAYER_PREPROCESS_FAILED_OVERLAPPING);
        return;
    }
}