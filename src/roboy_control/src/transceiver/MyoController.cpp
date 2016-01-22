//
// Created by bruh on 1/21/16.
//

#include "MyoController.h"

MyoController::MyoController() {
    MYOCONTROLLER_DBG << "Initialize Myo-Controller";

    // Create Myo-Master Transceiver
    QString name;
    name.sprintf("myomaster");
    m_myoMasterTransceiver = new ROSMessageTransceiverService(0, name);
    m_myoMasterTransceiver->setDelegate(this);
    m_myoMasterTransceiver->start();

    // Create Controller Status Structs
    MYOCONTROLLER_DBG << "Instantialte ROSControllers:";
    QList<qint8> controllerIds = RoboyControlConfiguration::instance().getControllersConfig();
    ROSController controller;
    for(qint8 id : controllerIds) {
        controller.id = id;
        controller.state = ControllerState::UNDEFINED;
        name.clear();
        name.sprintf("motor%i", id);
        controller.transceiver = new ROSMessageTransceiverService(id, name);
        controller.transceiver->setDelegate(this);
        controller.transceiver->start();
        m_mapControllers.insert(id, controller);
        MYOCONTROLLER_DBG << "\t- " << controller.toString();
    }
}

MyoController::~MyoController() {
    delete m_myoMasterTransceiver;
    for(ROSController & controller : m_mapControllers.values())
        delete controller.transceiver;
}

// RoboyController Interface
bool MyoController::initializeControllers() {
    MYOCONTROLLER_DBG << "Send Initialize Request to Myo-Master";
    bool result = false;
    m_myoMasterTransceiver->sendInitializeRequest(m_mapControllers.keys().toStdList());

    while(!m_bInitializationComplete){
        m_mutexCVTransceiver.lock();
        m_conditionTransceiver.wait(&m_mutexCVTransceiver);
        m_mutexCVTransceiver.unlock();
    }

    if(isInitializedCorrectly()) {
        MYOCONTROLLER_DBG << "Initialization completed successful" ;
        result = true;
    } else {
        MYOCONTROLLER_DBG << "Initialization failed";
        result = false;
    }

    MYOCONTROLLER_DBG << "Controller States:";
    for(ROSController controller : m_mapControllers.values())
        MYOCONTROLLER_DBG << "\t- " << controller.toString();

    return result;
}

// ITransceiverServiceDelegate - Callback-Interface Implementation
void MyoController::receivedControllerStatusUpdate(const QList<ROSController> & controllers) {
    for(ROSController controller : controllers) {
        m_mapControllers[controller.id].state = controller.state;
        MYOCONTROLLER_DBG << "Update Controller: " << controller.toString();
    }

    m_mutexCVTransceiver.lock();
    m_bInitializationComplete = true;
    m_conditionTransceiver.wakeAll();
    m_mutexCVTransceiver.unlock();
}

void MyoController::receivedControllerStatusUpdate(const ROSController &controller) {
    // TODO
}

// private
bool MyoController::isInitializedCorrectly() {
    for(ROSController & controller : m_mapControllers.values())
        if (controller.state != ControllerState::INITIALIZED)
            return false;

    return true;
}
