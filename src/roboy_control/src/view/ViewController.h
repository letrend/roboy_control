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
class MainWindow;

class ViewController : public QObject {

    Q_OBJECT

private:
    IModelService           * m_pModelSerivce;
    RoboyController         * m_pRoboyController;
    MainWindow              * m_pMainWindow;
    QQmlApplicationEngine   * m_pApplicationEngine;

public:
    ViewController(RoboyController * pRoboyController, IModelService * pModelService);

    void playBehaviorPlan  ();
    void pauseBehaviorPlan ();
    void stopBehaviorPlan  ();
    void skipBehavior      ();
    void rewindBehaviorPlan();
    void triggerInit       (); //TODO: Trigger Initialization by button

    RoboyBehaviorMetaplan fromController_getCurrentRoboyPlan();

signals:
    void signalPlay      ();
    void signalPause     ();
    void signalStop      ();
    void signalSkip      ();
    void signalRewind    ();
    void signalInitialize();

};


#endif //ROBOYCONTROL_VIEWCONTROLLER_H
