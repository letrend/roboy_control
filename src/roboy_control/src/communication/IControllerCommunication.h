//
// Created by bruh on 3/16/16.
//

#ifndef ROBOY_CONTROL_ICONTROLLERHANDLE_H
#define ROBOY_CONTROL_ICONTROLLERHANDLE_H

#include "DataTypes.h"
#include <QtCore/qmutex.h>
#include <QtCore/qwaitcondition.h>
#include <QtCore/qthread.h>
#include <LogDefines.h>

class IControllerCommunication : public QThread {

    Q_OBJECT

private:
    QMutex          m_mutexData;

    QMutex          m_mutexCV;

    QWaitCondition  m_condition;
    bool            m_bSendTrajectory = false;

    bool            m_bTerminate = false;

protected:
    ROSController * m_pController;
    qint32          m_id;
    QString         m_name;

    Trajectory      m_trajectory;

public:
    IControllerCommunication(ROSController * controller) {
        this->m_pController = controller;
        this->m_id = controller->m_id;
        QString name;
        this->m_name = name.sprintf("motor%u", m_id);
    }

    ~IControllerCommunication() {
        m_mutexCV.lock();
        m_bTerminate = true;
        m_condition.wakeAll();
        m_mutexCV.unlock();

        QThread::wait();

        TRANSCEIVER_LOG << "Thread terminated regularly";
    }

    void run() {
        bool run = true;
        TRANSCEIVER_LOG << "Started ... Wait for Events.";

        while(run) {
            m_mutexCV.lock();
            m_condition.wait(&m_mutexCV);

            if(m_bSendTrajectory) {
                TRANSCEIVER_LOG << "Triggered 'Send Trajectory'";
                m_bSendTrajectory = false;
                eventHandle_sendTrajectory();
            } else if (m_bTerminate) {
                TRANSCEIVER_LOG << "Received: Terminate Thread";
                run = false;
            }
            m_mutexCV.unlock();
        }
        TRANSCEIVER_LOG << "Thread interrupted. Exit.";

    }

    void sendTrajectory(const Trajectory & trajectory) {
        m_mutexData.lock();
        m_trajectory = trajectory;
        m_mutexData.unlock();

        m_mutexCV.lock();
        m_bSendTrajectory = true;
        m_condition.wakeAll();
        m_mutexCV.unlock();
    }

signals:
    void signalControllerStatusUpdated();

protected:
    virtual void eventHandle_sendTrajectory() = 0;
};


#endif //ROBOY_CONTROL_ICONTROLLERHANDLE_H
