//
// Created by bruh on 30.11.15.
//

#include "RoboyController.h"

RoboyController::RoboyController() {
    m_pModelService = new XmlModelService();
    m_pTransceiverService = new ROSMessageTransceiverService();

    m_pViewController = new ViewController(this, m_pModelService);

    // sending initialize request once
    size_t size = 10;
    std::vector<bool> b;
    for(int i=0; i<size; i++) {
        b.push_back(true);
    }
    CONTROLLER_DBG << "Sending initialize request...";
    m_pTransceiverService->sendInitializeRequest(b);
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
            executeCurrentRoboyPlan();
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

void RoboyController::executeCurrentRoboyPlan() {
    CONTROLLER_DBG << "Start to execute current Roboy Plan.";

    RoboyBehaviorMetaplan metaplan = m_pViewController->fromController_getCurrentRoboyPlan();

    CONTROLLER_DBG << "Loaded plan with " << metaplan.listExecutions.length() << " executions.";

    RoboyBehaviorPlan plan(m_pModelService, metaplan);

    CONTROLLER_DBG << "Created Plan " << plan.getExcutionsList().count();

/*    for(int i=0; i<plan.listExecutions.length(); i++){
        // for every execution
        RoboyBehavior rb = plan.listExecutions.at(i).behavior;
        QList<u_int32_t> motors = rb.m_mapMotorWaypoints.keys();
        CONTROLLER_DBG << "Processing behavior " << rb.m_metadata.m_sBehaviorName << " for " << motors.length() << " motors.";
        for(int j=0; j< motors.length(); j++){
            // for every motor
            QList<RoboyWaypoint> waypoints = rb.m_mapMotorWaypoints.take(motors.at(j));
            CONTROLLER_DBG << "Sending request for " << waypoints.length() << " waypoints.";
            m_pTransceiverService->sendTrajectory(motors.at(j),waypoints);
        }
    }
*/
    m_pTransceiverService->sendSteeringMessage(1);

}
