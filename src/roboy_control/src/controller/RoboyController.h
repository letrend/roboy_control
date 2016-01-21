//
// Created by bruh on 30.11.15.
//

#ifndef ROBOYCONTROL_ROBOYCONTROLLER_H
#define ROBOYCONTROL_ROBOYCONTROLLER_H

#include "ITransceiverService.h"
#include "ITransceiverServiceDelegate.h"
#include "ROSMessageTransceiverService.h"
#include "ViewController.h"
#include "XmlModelService.h"

#include <QMutex>
#include <QThread>
#include <QWaitCondition>

class RoboyController : public QThread, ITransceiverServiceDelegate {

    Q_OBJECT

private:
    IModelService       * m_pModelService;
    ITransceiverService * m_pTransceiverService;

    ViewController      * m_pViewController;

    QMutex                m_mutexCVView;
    QWaitCondition        m_conditionView;

    QMutex                m_mutexCVTransceiver;
    QWaitCondition        m_conditionTransceiver;

    bool    m_bStartExectution = false;
    bool    m_bStopExecution = false;
    bool    m_bTerminate = false;

    bool    m_bInitializationComplete = false;
    bool    m_bReceivedAllControllerStates = false;

    QMutex               m_mutexData;
    QList<ROSController> m_listControllers;

protected:
    void run();

public:
    RoboyController();
    ~RoboyController();

    // ViewController - Interface
    void fromViewController_triggerPlayPlan();

    // ITransceiverServiceDelegate - Callback-Interface Implementation
    void receivedControllerStatusUpdate(const QList<ROSController> & controllers);
    void receivedControllerStatusUpdate(const ROSController & controller);

private:
    bool initialize();
    bool checkForCorrectInitialization();
    void executeCurrentRoboyPlan();

};


#endif //ROBOYCONTROL_ROBOYCONTROLLER_H
