//
// Created by bruh on 30.11.15.
//

#include "RoboyController.h"

RoboyController::RoboyController() {
    m_pModelService = new XmlModelService();
    // TODO: create Method 'setDataModelService()' in MainWindow
    m_pMainWindow   = new MainWindow(m_pModelService);
    m_pMainWindow->show();

    // TODO: ITransceiverService should be abstract
    m_pTransceiverService = new ROSMessageTransceiverService();
}

RoboyController::~RoboyController() {
    delete m_pMainWindow;
    delete m_pModelService;
}

void RoboyController::startExecution() {

}

void RoboyController::pauseExecution() {

}

void RoboyController::stopExecution() {

}

void RoboyController::updateBehaviorQueue() {

}
