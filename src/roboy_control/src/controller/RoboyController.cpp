//
// Created by bruh on 30.11.15.
//

#include "RoboyController.h"

RoboyController::RoboyController() {
    m_pModelService = new XmlModelService();
    m_pViewController = new ViewController(this, m_pModelService);

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
    CONTROLLER_DBG << "Triggered 'Initialize' from View";
    if(m_pMyoController->initializeControllers())
        CONTROLLER_SUC << "Initialization Complete";
    else
        CONTROLLER_WAR << "Initialization failed.";
}

void RoboyController::slotPreprocessPlan() {
    CONTROLLER_DBG << "Triggered 'Preprocess Execution' from View";
    preprocessCurrentRoboyPlan();
}

void RoboyController::slotPlayPlan() {
    CONTROLLER_DBG << "Triggered 'Start Execution' from View";
    m_pMyoController->playPlanExecution();

}

void RoboyController::slotStopPlan() {
    CONTROLLER_DBG << "Triggered 'Stop Execution' from View";
    m_pMyoController->stopPlanExecution();
}

void RoboyController::slotPausePlan() {
    CONTROLLER_DBG << "Triggered 'Pause Execution' from View";
    m_pMyoController->pausePlanExecution();
}

void RoboyController::slotRecordBehavior() {
    CONTROLLER_DBG << "Triggered 'Record Behavior' from View";
    m_pMyoController->recordBehavior();
}

void RoboyController::slotStopRecording() {
    CONTROLLER_DBG << "Triggered 'Stop Recording' from View";
    m_pMyoController->stopRecording();
}

// Private Interface
void RoboyController::preprocessCurrentRoboyPlan() {
    CONTROLLER_DBG << "Start to execute current Roboy Plan.";
    CONTROLLER_DBG << "Get Metaplan from ViewController";
    RoboyBehaviorMetaplan metaplan = m_pViewController->fromController_getCurrentRoboyPlan();

    if(metaplan.listExecutions.isEmpty()) {
        CONTROLLER_WAR << "Empty Execution-List. Nothing to execute. Abort.";
        return;
    }

    CONTROLLER_DBG << "Build BehaviorPlan";
    RoboyBehaviorPlan plan(m_pModelService, metaplan);
    if(plan.isValid()){
        if(m_pMyoController->sendRoboyPlan(plan)) {
            CONTROLLER_SUC << "Plan is now executed on Roboy";
            CONTROLLER_SUC << "<3";
        } else {
            CONTROLLER_WAR << "Damn. Something went wrong sending the plan. See MyoController-Log";
        }
    } else {
        CONTROLLER_WAR << "Could not build valid plan. Abort.";
    }
}