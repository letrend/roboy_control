//
// Created by bruh on 2/4/16.
//

#ifndef ROBOY_CONTROL_TESTNODE_H
#define ROBOY_CONTROL_TESTNODE_H

#include "ros/ros.h"

#include "common_utilities/Initialize.h"
#include "common_utilities/ControllerState.h"
#include "common_utilities/Steer.h"
#include "common_utilities/Status.h"

#include "DataTypes.h"

class TestNode {

private:
    ros::NodeHandle      m_nodeHandle;

    QList<ROSController> m_listControllers;

    bool callbackInitialize(common_utilities::Initialize::Request & req, common_utilities::Initialize::Response & res);
    void callbackSteering(const common_utilities::Steer & msg);

    bool startNode(qint32 id, QString nodeName, QString serviceName, ROSController & controller);

public:
    TestNode();
    ~TestNode();
};

#endif //ROBOY_CONTROL_TESTNODE_H
