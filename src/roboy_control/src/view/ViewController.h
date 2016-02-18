//
// Created by bruh on 12/4/15.
//

#ifndef ROBOYCONTROL_VIEWCONTROLLER_H
#define ROBOYCONTROL_VIEWCONTROLLER_H

#include "IModelService.h"
#include "mainWindow/MainWindow.h"

#include <QDateTime>
#include <QThread>
#include <QQmlApplicationEngine>
#include <QQmlContext>

class RoboyController;

class ViewController {

private:
    IModelService   * m_pModelSerivce;
    RoboyController * m_pRoboyController;

    MainWindow      * m_pMainWindow;
    QQmlApplicationEngine *m_pApplicationEngine;

public:
    ViewController(RoboyController * pRoboyController, IModelService * pModelService);

    void playBehaviorPlan();
    RoboyBehaviorMetaplan fromController_getCurrentRoboyPlan();
};


#endif //ROBOYCONTROL_VIEWCONTROLLER_H
