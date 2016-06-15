//
// Created by bruh on 1/21/16.
//

#ifndef ROBOY_CONTROL_MYOCONTROLLER_H
#define ROBOY_CONTROL_MYOCONTROLLER_H

#include <QObject>

class IMotorController;
class IMyoController;
class IMyoMaster;
class RoboyBehaviorPlan;
class RoboyController;

class MyoController : public QObject {

    Q_OBJECT

public:
    MyoController();
    ~MyoController();

    // RoboyController Interface
    bool handleEvent_initializeControllers() const;
    bool handleEvent_preprocessRoboyPlan(RoboyBehaviorPlan &roboyPlan) const;

    bool handleEvent_playPlanExecution() const;
    bool handleEvent_pausePlanExecution() const;
    bool handleEvent_stopPlanExecution() const;

    bool handleEvent_recordBehavior() const;
    bool handleEvent_stopRecording() const;

    /*
public slots:
    // ControllerCommunication - Interface
    void slotControllerStatusUpdated(qint32 motorId);
    void slotRecordFinished(bool result);

private:
    void initializeControllerMap();
    bool waitForControllerStatus(QList<qint32> idList, ControllerState state, quint32 timeout = 0);
    bool checkControllersForState(QList<qint32> idList, ControllerState state);
     */
};


#endif //ROBOY_CONTROL_MYOCONTROLLER_H
