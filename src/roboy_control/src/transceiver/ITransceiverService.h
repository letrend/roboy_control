//
// Created by bruh on 30.11.15.
//

#ifndef ROBOYCONTROL_ITRANSCEIVERSERVICE_H
#define ROBOYCONTROL_ITRANSCEIVERSERVICE_H

#include <QThread>
#include <QWaitCondition>
#include <QtCore/qmutex.h>

#include "DataTypes.h"
#include "LogDefines.h"
#include "ITransceiverServiceDelegate.h"

class ITransceiverService : public QThread {

    Q_OBJECT

protected:
    qint32          m_motorId;
    QString         m_name;
    ITransceiverServiceDelegate * delegate;

    QMutex           m_mutexData;
    std::list<qint8> m_initializationList;
    Trajectory       m_trajectory;
    qint8            m_steeringCommand;

    QMutex          m_mutexCV;
    QWaitCondition  m_condition;
    bool            m_bSendInitialize = false;
    bool            m_bSendTrajectory = false;
    bool            m_bSendSteering = false;
    bool            m_bTerminate = false;

public:
    ITransceiverService(qint32 motorId, QString name = QString()) {
        m_motorId = motorId;
        if(name.isEmpty())
            m_name.sprintf("%i", motorId);
        else
            m_name = name;
    }

    ~ITransceiverService() {
        m_mutexCV.lock();
        m_bTerminate = true;
        m_condition.wakeAll();
        m_mutexCV.unlock();

        QThread::wait();

        TRANSCEIVER_LOG << "Thread terminated regularly";
    }

    void run() {
        bool run = true;
        TRANSCEIVER_LOG << "Transceiver Thread started";
        TRANSCEIVER_LOG << "Wait for Events";

        while(run) {
            m_mutexCV.lock();
            m_condition.wait(&m_mutexCV);

            if(m_bSendTrajectory) {
                TRANSCEIVER_LOG << "Triggered 'Send Trajectory'";
                m_bSendTrajectory = false;
                sendTrajectory();
            } else if (m_bSendInitialize) {
                TRANSCEIVER_LOG << "Triggered 'Send Initialize'";
                m_bSendInitialize = false;
                sendInitializeRequest();
            } else if (m_bSendSteering) {
                TRANSCEIVER_LOG << "Triggered 'Send Steering Command'";
                m_bSendSteering = false;
                sendSteeringMessage();
            } else if (m_bTerminate) {
                TRANSCEIVER_LOG << "Received: Terminate Thread";
                run = false;
            }
            m_mutexCV.unlock();
        }
        TRANSCEIVER_LOG << "Thread interrupted. Exit.";
    }

    void setDelegate(ITransceiverServiceDelegate * delegate) {
        this->delegate = delegate;
    }

    void sendInitializeRequest(const std::list<qint8> initializationList) {
        m_mutexData.lock();
        m_initializationList = initializationList;
        m_mutexData.unlock();

        m_mutexCV.lock();
        m_bSendInitialize = true;
        m_condition.wakeAll();
        m_mutexCV.unlock();
    }

    void sendTrajectory(Trajectory trajectory) {
        m_mutexData.lock();
        m_trajectory = trajectory;
        m_mutexData.unlock();

        m_mutexCV.lock();
        m_bSendTrajectory = true;
        m_condition.wakeAll();
        m_mutexCV.unlock();
    }

    void sendSteeringMessage(SteeringCommand command) {
        m_mutexData.lock();
        m_steeringCommand = command;
        m_mutexData.unlock();

        m_mutexCV.lock();
        m_bSendSteering = true;
        m_condition.wakeAll();
        m_mutexCV.unlock();
    }

protected:
    virtual void sendTrajectory() = 0;
    virtual void sendInitializeRequest() = 0;
    virtual void sendSteeringMessage() = 0;
};


#endif //ROBOYCONTROL_ITRANSCEIVERSERVICE_H
