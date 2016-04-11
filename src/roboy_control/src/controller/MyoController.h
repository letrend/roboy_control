//
// Created by bruh on 1/21/16.
//

#ifndef ROBOY_CONTROL_MYOCONTROLLER_H
#define ROBOY_CONTROL_MYOCONTROLLER_H

#include "DataTypes.h"
#include "IMasterCommunication.h"
#include "IControllerCommunication.h"
#include "RoboyControlConfiguration.h"
#include "ROSMasterCommunication.h"

#include <QThread>

class RoboyController;

class MyoController : public QObject {

    Q_OBJECT

private:
     const RoboyController      & m_roboyController;

    IMasterCommunication * m_myoMasterTransceiver;
    QMap<qint32, ROSController *>       m_mapControllers;

    QMutex                m_mutexCVController;
    QWaitCondition        m_conditionStatusUpdated;

public:
    MyoController(const RoboyController & controller);
    ~MyoController();

    // RoboyController Interface
    bool handleEvent_initializeControllers();
    bool handleEvent_preprocessRoboyPlan(RoboyBehaviorPlan &roboyPlan);

    bool handleEvent_playPlanExecution();
    bool handleEvent_pausePlanExecution();
    bool handleEvent_stopPlanExecution();

    bool handleEvent_recordBehavior();
    bool handleEvent_stopRecording();

public slots:
    // ControllerCommunication - Interface
    void slotControllerStatusUpdated(qint32 motorId);
    void slotRecordFinished(bool result);

private:
    bool waitForControllerStatus(QList<qint32> idList, ControllerState state, quint32 timeout = 0);
    bool checkControllersForState(QList<qint32> idList, ControllerState state);
};


#endif //ROBOY_CONTROL_MYOCONTROLLER_H
