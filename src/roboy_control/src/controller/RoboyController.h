//
// Created by bruh on 30.11.15.
//

#ifndef ROBOYCONTROL_ROBOYCONTROLLER_H
#define ROBOYCONTROL_ROBOYCONTROLLER_H

#include "ITransceiverService.h"
#include "ITransceiverServiceDelegate.h"
#include "MyoController.h"
#include "ROSMessageTransceiverService.h"
#include "ViewController.h"
#include "XmlModelService.h"

#include <QMutex>
#include <QThread>
#include <QWaitCondition>

class RoboyController : public QThread {

    Q_OBJECT

private:
    IModelService       * m_pModelService;
    ViewController      * m_pViewController;

    MyoController       * m_pMyoController;

protected:
    void run();

public:
    RoboyController();
    ~RoboyController();

public slots:
    void slotInitializeRoboy();
    void slotExecutePlan();
    void stopPlan();

signals:
    void updateRoboyStatus();
    void updatecontrollerStatus();

private:
    void executeCurrentRoboyPlan();

};


#endif //ROBOYCONTROL_ROBOYCONTROLLER_H
