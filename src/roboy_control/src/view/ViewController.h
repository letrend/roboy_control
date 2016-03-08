//
// Created by bruh on 12/4/15.
//

#ifndef ROBOYCONTROL_VIEWCONTROLLER_H
#define ROBOYCONTROL_VIEWCONTROLLER_H

#include "IModelService.h"
#include "MainWindow/MainWindow.h"

#include <QDateTime>
#include <QThread>

class RoboyController;

class ViewController : public QObject {

    Q_OBJECT

private:
    IModelService   * m_pModelSerivce;
    RoboyController * m_pRoboyController;

    MainWindow      * m_pMainWindow;

public:
    ViewController(RoboyController * pRoboyController, IModelService * pModelService);

    void playBehaviorPlan();
    RoboyBehaviorMetaplan fromController_getCurrentRoboyPlan();

    //TODO: Trigger Initialization by button
    void triggerInit();

signals:
    void signalInitialize();
    void signalPlay();
    void signalStop();
    void signalRewind();

};


#endif //ROBOYCONTROL_VIEWCONTROLLER_H
