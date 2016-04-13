//
// Created by bruh on 30.11.15.
//

#include "RoboyController.h"
#include "DataPool.h"

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
    CONTROLLER_DBG << "Controller Thread Started";
    CONTROLLER_DBG << "ControllerThread-Id is: " << this->currentThreadId();

    m_pMyoController = new MyoController();
    msleep(1000);

    exec();

    CONTROLLER_DBG << "Controller Thread Interrupted. Quit.";
}

// Slots
void RoboyController::slotInitializeRoboy() {
    CONTROLLER_DBG << "\n\n";
    CONTROLLER_SUC << "---------------- EVENT: Initialize ------------------";
    if(m_pMyoController->handleEvent_initializeControllers())
        CONTROLLER_SUC << "Initialization Complete";
    else
        CONTROLLER_WAR << "Initialization failed.";
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
    m_pMyoController->handleEvent_playPlanExecution();
    CONTROLLER_SUC << "<3";
    CONTROLLER_SUC << "------------------ /EVENT: Play -----------------\n\n";
}

void RoboyController::slotStopPlan() {
    CONTROLLER_DBG << "Triggered 'Stop Execution' from View";
    m_pMyoController->handleEvent_stopPlanExecution();
}

void RoboyController::slotPausePlan() {
    CONTROLLER_DBG << "Triggered 'Pause Execution' from View";
    m_pMyoController->handleEvent_pausePlanExecution();
}

void RoboyController::slotRecordBehavior() {
    CONTROLLER_DBG << "Triggered 'Record Behavior' from View";
    m_pMyoController->handleEvent_recordBehavior();
}

void RoboyController::slotStopRecording() {
    CONTROLLER_DBG << "Triggered 'Stop Recording' from View";
    m_pMyoController->handleEvent_stopRecording();
}

// Private Interface
void RoboyController::preprocessCurrentRoboyPlan() {
    CONTROLLER_DBG << "Get Metaplan from ViewController";
    RoboyBehaviorMetaplan metaplan = m_pViewController->fromController_getCurrentRoboyPlan();

    if(metaplan.listExecutions.isEmpty()) {
        CONTROLLER_WAR << "Empty Execution-List. Nothing to execute. Abort.";
        DataPool::getInstance()->setPlayerState(PlayerState::PLAYER_PREPROCESS_FAILED_EMPTY);
        return;
    }

    CONTROLLER_DBG << "Build BehaviorPlan";
    RoboyBehaviorPlan plan(m_pModelService, metaplan);
    // TODO: Check ModeConflict, ControllerStates
    if(plan.isValid()){
        if(m_pMyoController->handleEvent_preprocessRoboyPlan(plan)) {
            CONTROLLER_SUC << "Plan is ready to be executed on Roboy";
            DataPool::getInstance()->setPlayerState(PlayerState::PLAYER_TRAJECTORY_READY);
        } else {
            CONTROLLER_WAR << "Damn. Something went wrong sending the plan. See MyoController-Log";
        }
    } else {
        CONTROLLER_WAR << "Could not build valid plan. Abort.";
        DataPool::getInstance()->setPlayerState(PlayerState::PLAYER_PREPROCESS_FAILED_OVERLAPPING);
    }
}