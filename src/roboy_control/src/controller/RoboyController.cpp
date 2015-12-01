//
// Created by bruh on 30.11.15.
//

#include "RoboyController.h"

RoboyController::RoboyController() {
    m_pModelService = new XmlModelService();
    // TODO: create Method 'setDataModelService()' in MainWindow
    m_pMainWindow   = new MainWindow(m_pModelService);

    m_pMainWindow->show();
}

RoboyController::~RoboyController() {
    delete m_pMainWindow;
    delete m_pModelService;
}
void RoboyController::initializeComponents() {



}
