//
// Created by bruh on 12/4/15.
//

#ifndef ROBOYCONTROL_VIEWCONTROLLER_H
#define ROBOYCONTROL_VIEWCONTROLLER_H

#include "IModelService.h"
#include "MainWindow/MainWindow.h"

#include <QThread>

class RoboyController;

class ViewController {

private:
    IModelService   * m_pModelSerivce;
    RoboyController * m_pRoboyController;

    MainWindow      * m_pMainWindow;

public:
    ViewController(RoboyController * pRoboyController, IModelService * pModelService);

    void playBehaviorPlan();
};


#endif //ROBOYCONTROL_VIEWCONTROLLER_H
