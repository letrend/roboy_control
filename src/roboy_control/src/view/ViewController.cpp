//
// Created by bruh on 12/4/15.
//

#include "ViewController.h"
#include "RoboyController.h"

ViewController::ViewController(RoboyController * pRoboyController, IModelService * pModelService) {
    VIEW_DBG << "Initialize View";
    VIEW_DBG << "View Thread ID is: " << QThread::currentThreadId();

    m_pRoboyController = pRoboyController;
    m_pModelSerivce = pModelService;

    m_pMainWindow = new MainWindow(m_pModelSerivce, this);
    m_pMainWindow->show();
}

void ViewController::playBehaviorPlan() {
    VIEW_DBG << "Play Behavior Plan triggered.";

    m_pRoboyController->fromViewController_triggerPlayPlan();
}

RoboyBehaviorPlan ViewController::fromController_getCurrentRoboyPlan() {
    // TODO: Get RoboyBehaviorPlan from PlayerView
    RoboyBehaviorPlan plan;
    RoboyBehaviorExecution ex;
    QList<RoboyWaypoint> waypoints;
    RoboyWaypoint p1;
    p1.m_ulTimestamp = 50;
    p1.m_ulPosition = 10;
    RoboyWaypoint p2;
    p2.m_ulTimestamp = 100;
    p2.m_ulPosition = 30;
    waypoints.push_back(p1);
    waypoints.push_back(p2);
    ex.behavior.m_mapMotorWaypoints.insert(1,waypoints);
    plan.listExecutions.push_back(ex);
    return plan;
}
