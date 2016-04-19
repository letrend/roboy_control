//
// Created by bruh on 4/11/16.
//

#ifndef ROBOY_CONTROL_DATAPOOL_H
#define ROBOY_CONTROL_DATAPOOL_H

#include "DataTypes.h"
#include <QMutex>

class DataPool : public QObject {

    Q_OBJECT

private:
    static DataPool * instance;

    QMutex          m_mutexData;

    DataPool() { }

private:
    PlayerState     m_playerState = PlayerState::PLAYER_NOT_READY;
    RecorderState   m_recorderState = RecorderState::RECORDER_NOT_READY;

    QMap<qint32, ROSController> m_controllerStates;

    // Record Request
    qint32          m_sampleRate = 0;

    // Record Results
    bool           m_recordResult = false;
    RoboyBehavior  m_recordBehavior;

public:
    static DataPool * getInstance();

    void insertController(const ROSController & controller);

    void reset();

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
};

#endif //ROBOY_CONTROL_DATAPOOL_H
