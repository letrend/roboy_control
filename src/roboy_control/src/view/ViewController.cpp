//
// Created by bruh on 12/4/15.
//

#include "ViewController.h"
#include "RoboyController.h"

ViewController::ViewController(RoboyController * pRoboyController, IModelService * pModelService) {
    VIEW_DBG << "Initialize View";

    m_pRoboyController = pRoboyController;
    m_pModelSerivce = pModelService;

    m_pMainWindow = new MainWindow(m_pModelSerivce, this);
    m_pMainWindow->show();
}

void ViewController::playBehaviorPlan() {
    VIEW_DBG << "Play Behavior Plan triggered.";

    m_pRoboyController->fromViewController_triggerPlayPlan();
}

