//
// Created by bruh on 30.11.15.
//

#ifndef ROBOYCONTROL_ROBOYCONTROLLER_H
#define ROBOYCONTROL_ROBOYCONTROLLER_H

#include "IMasterCommunication.h"
#include "MyoController.h"
#include "ROSMasterCommunication.h"
#include "ViewController.h"
#include "XmlModelService.h"

#include <QMutex>
#include <QThread>
#include <QWaitCondition>

class RoboyController : public QThread {

    Q_OBJECT

private:
    IModelService       * m_pModelService   = nullptr;
    ViewController      * m_pViewController = nullptr;

    MyoController       * m_pMyoController  = nullptr;

protected:
    void run();

public:
    RoboyController();
    ~RoboyController();

// View - Interface
public slots:
    // Roboy (Tab) - Interface
    void slotInitializeRoboy();

    // Player - Interface
    void slotPreprocessPlan();

    void slotPlayPlan();
    void slotStopPlan();
    void slotPausePlan();

    // Recorder - Interface
    void slotRecordBehavior();
    void slotStopRecording();

private:
    void preprocessCurrentRoboyPlan();

};

#endif //ROBOYCONTROL_ROBOYCONTROLLER_H