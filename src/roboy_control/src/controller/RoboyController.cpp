//
// Created by bruh on 30.11.15.
//

#include "RoboyController.h"

RoboyController::RoboyController() {
    m_pModelService = new XmlModelService();
    m_pTransceiverService = new ROSMessageTransceiverService();
    m_pTransceiverService->setDelegate(this);

    m_pViewController = new ViewController(this, m_pModelService);
}

RoboyController::~RoboyController() {
    m_bTerminate = true;
    m_mutexCVView.lock();
    m_conditionView.wakeAll();
    m_mutexCVView.unlock();

    QThread::wait();

    if (this->isFinished())
        CONTROLLER_DBG << "Thread terminated";

    delete m_pModelService;
    delete m_pTransceiverService;
    delete m_pViewController;
}

void RoboyController::run() {
    CONTROLLER_DBG << "Controller Thread Started";
    CONTROLLER_DBG << "ID is: " << this->currentThreadId();
    CONTROLLER_DBG << "Initialize Controllers";

    bool run = false;
    if(initialize()) {
        CONTROLLER_DBG << "Controller Thread wait for events";
        run = true;
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

// Private Interface
bool RoboyController::initialize() {
    bool result = false;
    QList<qint8> controllerIds = RoboyControlConfiguration::instance().getControllersConfig();
    ROSController controller;
    for(qint8 id : controllerIds) {
        controller.id = id;
        controller.state = ControllerState::UNDEFINED;
        m_listControllers.append(controller);
    }

    m_pTransceiverService->sendInitializeRequest(controllerIds.toStdList());

    if(checkForCorrectInitialization()) {
        CONTROLLER_DBG << "Initialization completed successful" ;
        result = true;
    } else {
        CONTROLLER_DBG << "Initialization failed";
        CONTROLLER_DBG << "INITIALIZATION RESULT:";
        for(ROSController controller : m_listControllers) {
            CONTROLLER_DBG << " - Controller: " << controller.id << " state: " << controller.state;
        }
        result = false;
    }
    return result;
}

bool RoboyController::checkForCorrectInitialization() {
    bool invalidState = false;
    for(ROSController & controller : m_listControllers)
        if (controller.state != ControllerState::INITIALIZED)
            return false;

    return true;
}

void RoboyController::fromViewController_triggerPlayPlan() {
    m_mutexCVView.lock();
    m_bStartExectution = true;
    m_conditionView.wakeAll();
    m_mutexCVView.unlock();
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

// ITransceiverServiceDelegate - Callback-Interface Implementation
void RoboyController::receivedControllerStatusUpdate(const QList<ROSController> & controllers) {
    for(ROSController receivedController : controllers) {
        for(ROSController & localController : m_listControllers) {
            if (receivedController.id == localController.id) {
                localController.state = ControllerState::INITIALIZED;
                CONTROLLER_DBG << "Update Controller Status: " << localController.id << " state: " << localController.state;
                break;
            }
        }
    }
}