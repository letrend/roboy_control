//
// Created by bruh on 1/21/16.
//

#ifndef ROBOY_CONTROL_MYOCONTROLLER_H
#define ROBOY_CONTROL_MYOCONTROLLER_H

#include "DataTypes.h"
#include "ITransceiverService.h"
#include "RoboyControlConfiguration.h"
#include "ROSMessageTransceiverService.h"

#include <QThread>

class MyoController : public ITransceiverServiceDelegate{

private:
    ITransceiverService             * m_myoMasterTransceiver;
    QMap<qint8, ROSController>        m_mapControllers;

    QMutex                m_mutexCVTransceiver;
    QWaitCondition        m_conditionTransceiver;

    bool    m_bInitializationComplete = false;
    bool    m_bReceivedAllControllerStates = false;


public:
    MyoController();
    ~MyoController();

    // RoboyController Interface
    bool initializeControllers();
    bool sendRoboyPlan(RoboyBehaviorPlan & roboyPlan);

    // ITransceiverServiceDelegate - Callback-Interface Implementation
    void receivedControllerStatusUpdate(const QList<ROSController> & controllers);
    void receivedControllerStatusUpdate(const ROSController & controller);

private:
    bool isInitializedCorrectly();
    bool didReceiveAllStatusUpdates();
    bool isReadyToPlay(RoboyBehaviorPlan & plan);
};


#endif //ROBOY_CONTROL_MYOCONTROLLER_H
