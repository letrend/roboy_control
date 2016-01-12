//
// Created by bruh on 12/4/15.
//

#include "ViewController.h"
#include "RoboyController.h"

/**
 * @brief ViewController::ViewController constructor
 * @param pRoboyController roboy controller that is in charge of coordinating all components of the program
 * @param pModelService modelService from which the RoboyBehaviors are retrieved
 */
ViewController::ViewController(RoboyController * pRoboyController, IModelService * pModelService) {
    VIEW_DBG << "Initialize View";
    VIEW_DBG << "View Thread ID is: " << QThread::currentThreadId();

    m_pRoboyController = pRoboyController;
    m_pModelSerivce = pModelService;

    m_pMainWindow = new MainWindow(m_pModelSerivce, this);
    m_pMainWindow->show();
}

/**
 * @brief ViewController::playBehaviorPlan method for triggering playing the current behavior plan
 */
void ViewController::playBehaviorPlan() {
    VIEW_DBG << "Play Behavior Plan triggered.";

    m_pRoboyController->fromViewController_triggerPlayPlan();
}

/**
 * @brief ViewController::fromController_getCurrentRoboyPlan method fore retrieving the current behavior plan
 * @return the current behavior plan
 */
RoboyBehaviorMetaplan ViewController::fromController_getCurrentRoboyPlan() {
    // TODO: Get RoboyBehaviorPlan from PlayerView
    RoboyBehaviorMetaplan plan;

    RoboyBehaviorMetadata behavior1;
    behavior1.m_sBehaviorName = "TestBehavior1";
    behavior1.m_ulBehaviorId = 1;

    RoboyBehaviorMetaExecution execution;
    execution.lId = 1;
    execution.lTimestamp = QDateTime::currentMSecsSinceEpoch();
    execution.behaviorMetadata = behavior1;

    plan.listExecutions.append(execution);

    return plan;
}
