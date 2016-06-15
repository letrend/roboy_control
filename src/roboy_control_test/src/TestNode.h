//
// Created by bruh on 2/4/16.
//

#ifndef ROBOY_CONTROL_TESTNODE_H
#define ROBOY_CONTROL_TESTNODE_H


#include "boost/function.hpp"
#include "common_utilities/ControllerState.h"
#include "common_utilities/Initialize.h"
#include <common_utilities/Record.h>
#include "common_utilities/Steer.h"
#include "common_utilities/Status.h"
#include "ros/ros.h"
#include "ros/callback_queue.h"

#include "DataTypes.h"
#include <QMutex>
#include <QWaitCondition>

class TestNode {

private:
    QMutex          m_mutexCV;

    bool m_bInterrupted = false;

    ros::NodeHandle      m_nodeHandle;

    ros::Subscriber   m_initializeSubscriber;
    ros::ServiceServer   m_recordServer;

    ros::Subscriber      m_recordSteeringSubscriber;

    QList<MotorController> m_listControllers;

    void callbackInitialize(const common_utilities::Initialize & msg);
    bool callbackRecord(common_utilities::Record::Request & req, common_utilities::Record::Response & res);
    void callbackSteerRecord(const common_utilities::Steer & msg);

    bool startNode(qint32 id, QString nodeName, QString serviceName, MotorController & controller);

public:
    TestNode();
    ~TestNode();
};

#endif //ROBOY_CONTROL_TESTNODE_H
