//
// Created by bruh on 30.11.15.
//

#include "RoboyController.h"

RoboyController::RoboyController() {
    m_pModelService = new XmlModelService();
    m_pTransceiverService = new ROSMessageTransceiverService();

    m_pViewController = new ViewController(this, m_pModelService);
}

RoboyController::~RoboyController() {
    m_bTerminate = true;
    m_mutexCV.lock();
    m_conditionView.wakeAll();
    m_mutexCV.unlock();

    QThread::wait();

    if (this->isFinished())
        CONTROLLER_DBG << "Thread terminated";

    delete m_pModelService;
    delete m_pTransceiverService;
    delete m_pViewController;
}

void RoboyController::run() {

    CONTROLLER_DBG << "Controller Thread Started. Wait for events ... ";
    CONTROLLER_DBG << "ID is: " << this->currentThreadId();

    bool run = true;
    while( run ) {
        m_mutexCV.lock();
        m_conditionView.wait(&m_mutexCV);

        if( m_bStartExectution ) {
            CONTROLLER_DBG << "Triggered 'Start Execution' from View";
            m_bStartExectution = false;
        } else if ( m_bStopExecution ) {
            CONTROLLER_DBG << "Triggered 'Stop Execution' from View";
            m_bStopExecution = false;
        } else if ( m_bTerminate ) {
            CONTROLLER_DBG << "Received Terminate Thread. Quit.";
            run = false;
        }

        m_mutexCV.unlock();
    }

    CONTROLLER_DBG << "Controller Thread Interrupted. Quit.";
}

void RoboyController::fromViewController_triggerPlayPlan() {
    m_mutexCV.lock();
    m_bStartExectution = true;
    m_conditionView.wakeAll();
    m_mutexCV.unlock();
}