//
// Created by bruh on 4/11/16.
//

#ifndef ROBOY_CONTROL_DATAPOOL_H
#define ROBOY_CONTROL_DATAPOOL_H

#include "DataTypes.h"

#include "IMotorController.h"
#include "IMyoMaster.h"

#include <QMutex>

class DataPool : public QObject {

    Q_OBJECT

private:
    QMutex          m_mutexData;

    DataPool() { }

    PlayerState     m_playerState = PlayerState::PLAYER_NOT_READY;
    RecorderState   m_recorderState = RecorderState::RECORDER_NOT_READY;

    IMyoMaster * m_myoMaster;
    QMap<qint32, IMotorController *> m_motorControllers;

    const RoboyBehaviorPlan * m_currentRoboyPlan;








    // Record Request
    qint32          m_sampleRate = 0;

    // Record Results
    bool           m_recordResult = false;
    RoboyBehavior  m_recordBehavior;

public:
    static DataPool& instance() {
        static DataPool _instance;
        return _instance;
    }

    void initializeDataPool();

    void setMotorControllerState(qint32 id, ControllerState state);
    ControllerState getMotorControllerState(qint32 id) const;

    ControlMode getMotorControlMode(qint32 id) const;

    IMyoMaster * getMyoMaster() const;
    const QList<IMotorController *> getMotorControllers() const;
    IMotorController * getMotorController(qint32 id) const;

    void setPlayerState(PlayerState state);
    PlayerState getPlayerState() const;

    void setCurrentBehaviorPlan(const RoboyBehaviorPlan * plan);

private:
    void reset();
    void updateApplicationStates(ControllerState state);
    bool checkControllersForState(const QList<qint32> idList, ControllerState state) const;

signals:
    void signalNotifyOnDataPoolReset();
    void signalNotifyOnControllerStateUpdated(qint32 motorId);
    void signalNotifyOnPlayerStateUpdated();


/*




    void setPlayerState(PlayerState state);
    void setRecorderState(RecorderState state);
    void setControllerState(qint32 id, ControllerState state);

    void setSampleRate(qint32 sampleRate);

    void setRecordResult(bool result, RoboyBehavior * pBehavior);

    PlayerState getPlayerState();
    RecorderState getRecorderState();

    ControllerState getControllerState(qint32 motorId);
    ControlMode getControlMode(qint32 motorId);

    float getSampleRate();

    bool getRecordResult();
    RoboyBehavior * getRecordedBehavior();

signals:
    void signalNotifyOnPlayerStateUpdated();
    void signalNotifyOnRecorderStateUpdated();
    void signalNotifyOnControllerStateUpdated(qint32 motorId);
    void signalNotifyOnRecordResult();
    void signalNotifyOnDataPoolReset();

*/
};

#endif //ROBOY_CONTROL_DATAPOOL_H
