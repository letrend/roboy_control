//
// Created by bruh on 30.11.15.
//

#ifndef ROBOYCONTROL_ITRANSCEIVERSERVICE_H
#define ROBOYCONTROL_ITRANSCEIVERSERVICE_H

#include "DataTypes.h"
#include "LogDefines.h"
#include <QtCore/qmutex.h>
#include <QThread>
#include <QWaitCondition>
#include "ros/ros.h"

class IMasterCommunication : public QThread {

    Q_OBJECT

protected:
    QString         m_name;

    QMutex                   m_mutexData;
    QList<ROSController *>   m_initializationList;
    SteeringCommand          m_steeringCommand;
    QList<ROSController *>   m_recordRequest;
    SteeringCommand          m_recordSteeringCommand;
    RoboyBehavior *          m_pRecordedBehavior;

    QMutex          m_mutexCV;
    QWaitCondition  m_condition;
    bool            m_bSendInitialize = false;
    bool            m_bSendSteering = false;
    bool            m_bStartRecording = false;
    bool            m_bSendRecordSteering = false;
    bool            m_bTerminate = false;

public:
    IMasterCommunication() {
        m_name = "transceiver";
    }

    ~IMasterCommunication() {
        m_mutexCV.lock();
        m_bTerminate = true;
        m_condition.wakeAll();
        m_mutexCV.unlock();

        QThread::wait();

        delete m_pRecordedBehavior;

        TRANSCEIVER_LOG << "Thread terminated regularly";
    }

    void run() {
        bool run = true;
        TRANSCEIVER_LOG << "Wait for Events";

        ros::AsyncSpinner spinner(10);
        spinner.start();

        while(run) {
            m_mutexCV.lock();
            m_condition.wait(&m_mutexCV);

            if (m_bSendInitialize) {
                TRANSCEIVER_LOG << "Triggered 'Send Initialize'";
                m_bSendInitialize = false;
                eventHandle_sendInitializeRequest();
            } else if (m_bSendSteering) {
                TRANSCEIVER_LOG << "Triggered 'Send Steering Command'";
                m_bSendSteering = false;
                eventHandle_sendSteeringMessage();
            } else if (m_bStartRecording) {
                TRANSCEIVER_LOG << "Received: Start Recording";
                m_bStartRecording = false;
                eventHandle_recordBehavior();
                TRANSCEIVER_SUC << "Return from Record Call";
            } else if (m_bSendRecordSteering) {
                TRANSCEIVER_LOG << "Triggered 'Send Record Steering Command'";
                m_bSendRecordSteering = false;
                eventHandle_sendRecordSteeringMessage();
            } else if (m_bTerminate) {
                TRANSCEIVER_LOG << "Received: Terminate Thread";
                run = false;
            }
            m_mutexCV.unlock();
        }
        TRANSCEIVER_LOG << "Thread interrupted. Exit.";
    }

    void sendInitializeRequest(const QList<ROSController *> initializationList) {
        m_mutexData.lock();
        m_initializationList = initializationList;
        m_mutexData.unlock();

        m_mutexCV.lock();
        m_bSendInitialize = true;
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

    void startRecording(const QList<ROSController *> controllers) {
        m_mutexData.lock();
        m_recordRequest = controllers;
        m_mutexData.unlock();

        m_mutexCV.lock();
        m_bStartRecording = true;
        m_condition.wakeAll();
        m_mutexCV.unlock();
    }

    void sendRecordSteeringMessage(SteeringCommand command) {
//        m_mutexData.lock();
//        m_recordSteeringCommand = command;
//        m_mutexData.unlock();
//
//        m_mutexCV.lock();
//        m_bSendRecordSteering = true;
//        m_condition.wakeAll();
//        m_mutexCV.unlock();
        m_recordSteeringCommand = command;
        eventHandle_sendRecordSteeringMessage();

    }

    RoboyBehavior * getRecordedBehavior() {
        return m_pRecordedBehavior;
    }

    virtual void startControllers(const QList<qint32> & controllers) = 0;
    virtual void stopControllers(const QList<qint32> & contollers) = 0;

protected:
    virtual void eventHandle_sendInitializeRequest() = 0;
    virtual void eventHandle_sendSteeringMessage() = 0;

    virtual void eventHandle_recordBehavior() = 0;
    virtual void eventHandle_sendRecordSteeringMessage() = 0;

signals:
    void signalRecordFinished(bool result);
};

#endif //ROBOYCONTROL_ITRANSCEIVERSERVICE_H