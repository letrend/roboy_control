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

    void preprocessBehaviorPlan();
    void playBehaviorPlan();
    void stopBehaviorPlan();
    void pauseBehaviorPlan();
    void rewindBehaviorPlan();

    void recordBehavior();
    void stopRecording();

    RoboyBehaviorMetaplan fromController_getCurrentRoboyPlan();

    //TODO: Trigger Initialization by button
    void triggerInit();

signals:
    void signalInitialize();
    void signalPreprocess();
    void signalPlay();
    void signalStop();
    void signalPause();
    void signalRewind();

    void signalRecord();
    void signalStopRecording();
};


#endif //ROBOYCONTROL_VIEWCONTROLLER_H
