//
// Created by bruh on 30.11.15.
//

#ifndef ROBOYCONTROL_ROBOYCONTROLLER_H
#define ROBOYCONTROL_ROBOYCONTROLLER_H

#include "ITransceiverService.h"
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
    ITransceiverService * m_pTransceiverService;

    ViewController      * m_pViewController;

    QMutex                m_mutexCV;
    QWaitCondition        m_conditionView;

    bool    m_bControllersInitialized = false;
    bool    m_bStartExectution = false;
    bool    m_bStopExecution = false;
    bool    m_bTerminate = false;

    void executeCurrentRoboyPlan();

protected:
    void run();

public:
    RoboyController();
    ~RoboyController();

    void fromViewController_triggerPlayPlan();
};


#endif //ROBOYCONTROL_ROBOYCONTROLLER_H
