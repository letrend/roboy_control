//
// Created by bruh on 12/4/15.
//

#ifndef ROBOYCONTROL_VIEWCONTROLLER_H
#define ROBOYCONTROL_VIEWCONTROLLER_H

#include <QDateTime>
#include <QQmlApplicationEngine>
#include <QQmlContext>
#include <QThread>

#include "IModelService.h"
#include "mainWindow/MainWindow.h"

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

    // PlayerView - Interface
    void triggerInit();
    void preprocessBehaviorPlan();
    void playBehaviorPlan();
    void stopBehaviorPlan();
    void pauseBehaviorPlan();

    // RecorderView - Interface
    void recordBehavior();
    void stopRecording();

    // RoboyController - Interface
    RoboyBehaviorMetaplan fromController_getCurrentRoboyPlan();

public slots:
    void signalPlayerStatusUpdated(PlayerState state);
    void signalControllerStatusUpdated(qint32 motorId, ControllerState state);

signals:
    void signalInitialize();
    void signalPreprocess();
    void signalPlay();
    void signalStop();
    void signalPause();

    void signalRecord();
    void signalStopRecording();
};


#endif //ROBOYCONTROL_VIEWCONTROLLER_H
