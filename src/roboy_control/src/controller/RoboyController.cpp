//
// Created by bruh on 30.11.15.
//

#include "RoboyController.h"

RoboyController::RoboyController() {
    m_pModelService = new XmlModelService();
    m_pViewController = new ViewController(this, m_pModelService);
}

RoboyController::~RoboyController() {
    m_bTerminate = true;
    m_mutexCVView.lock();
    m_conditionView.wakeAll();
    m_mutexCVView.unlock();

    QThread::wait();

    delete m_pMyoController;
    delete m_pModelService;
    delete m_pViewController;

    if (this->isFinished())
        CONTROLLER_DBG << "Thread terminated";
}

void RoboyController::run() {
    CONTROLLER_DBG << "Controller Thread Started";
    CONTROLLER_DBG << "ID is: " << this->currentThreadId();

    m_pMyoController = new MyoController();
    msleep(1000);

    CONTROLLER_DBG << "Initialize Controllers";

    bool run = false;
    if(m_pMyoController->initializeControllers()) {
        CONTROLLER_DBG << "Controller Thread wait for events";
        run = true;
    } else {
        CONTROLLER_DBG << "Initialization failed. Exit.";
    }

    while( run ) {
        m_mutexCVView.lock();
        m_conditionView.wait(&m_mutexCVView);

        if( m_bStartExectution ) {
            CONTROLLER_DBG << "Triggered 'Start Execution' from View";
            m_bStartExectution = false;
            executeCurrentRoboyPlan();
        } else if ( m_bStopExecution ) {
            CONTROLLER_DBG << "Triggered 'Stop Execution' from View";
            m_bStopExecution = false;
        } else if ( m_bTerminate ) {
            CONTROLLER_DBG << "Received Terminate Thread. Quit.";
            run = false;
        }

        m_mutexCVView.unlock();
    }

    CONTROLLER_DBG << "Controller Thread Interrupted. Quit.";
}

// ViewController Interface
void RoboyController::fromViewController_triggerPlayPlan() {
    m_mutexCVView.lock();
    m_bStartExectution = true;
    m_conditionView.wakeAll();
    m_mutexCVView.unlock();
}

// Private Interface
void RoboyController::executeCurrentRoboyPlan() {
    CONTROLLER_DBG << "Start to execute current Roboy Plan.";
    CONTROLLER_DBG << "Get Metaplan from ViewController";
    RoboyBehaviorMetaplan metaplan = m_pViewController->fromController_getCurrentRoboyPlan();

    CONTROLLER_DBG << "Build BehaviorPlan";
    RoboyBehaviorPlan plan(m_pModelService, metaplan);

    CONTROLLER_DBG << "Get Flattended Trajectories";
    QMap<qint32, Trajectory> mapTrajectories = plan.getTrajectories();

    CONTROLLER_DBG << "Send Trajectories";
    for(qint32 id : mapTrajectories.keys()) {
        CONTROLLER_DBG << "Motor Id: " << id << mapTrajectories.value(id).toString();
//        m_pTransceiverService->sendTrajectory(id, mapTrajectories.value(id));
    }

//    while(!m_bReceivedAllControllerStates) {
//        m_mutexCVTransceiver.lock();
//        m_conditionTransceiver.wait(&m_mutexCVTransceiver);
//        m_mutexCVTransceiver.unlock();
//    }
//    TRANSCEIVER_LOG << "Received all Controller Status Updates";
//    m_bReceivedAllControllerStates = false;
}