//
// Created by bruh on 12/4/15.
//

#ifndef ROBOYCONTROL_VIEWCONTROLLER_H
#define ROBOYCONTROL_VIEWCONTROLLER_H

#include <QDateTime>
#include <QQmlApplicationEngine>
#include <QQmlContext>
#include <QQuickItem>
#include <QThread>

#include "IModelService.h"
#include "mainWindow/MainWindow.h"

class RoboyController;
class MainWindow;

class ViewController : public QObject {

    Q_OBJECT

public:
    ViewController(IModelService * pModelService);

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
    void slotPlayerStateUpdated() const;
    void slotRecorderStateUpdated() const;
    void slotControllerStateUpdated(qint32 motorId) const;
    void slotRecorderResultReceived() const;
    void slotDataPoolReset() const;

signals:
    void signalInitialize();
    void signalPreprocess();
    void signalPlay();
    void signalStop();
    void signalPause();

    void signalRecord();
    void signalStopRecording();

private:
    IModelService           * m_pModelSerivce;
    MainWindow              * m_pMainWindow;
    QQmlApplicationEngine   * m_pApplicationEngine;
};


#endif //ROBOYCONTROL_VIEWCONTROLLER_H
