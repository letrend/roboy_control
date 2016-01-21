//
// Created by bruh on 12/14/15.
//

#include <QDebug>

#include "ros/ros.h"
#include "std_msgs/String.h"

#include "common_utilities/Initialize.h"
#include "common_utilities/ControllerState.h"
//#include "common_utilities/InitializeRequest.h"
//#include "common_utilities/InitializeResponse.h"
#include "common_utilities/Trajectory.h"
#include "common_utilities/Steer.h"
#include "common_utilities/Status.h"

#include "CommonDefinitions.h"

ros::Publisher publisher;
ros::Publisher publisherStatus;
ros::Subscriber subscriber;
ros::Subscriber subscriberMotor1;
ros::Subscriber subscriberSteer;

bool callbackInitialize(common_utilities::Initialize::Request & req, common_utilities::Initialize::Response & res){
    qDebug() << "Process 'initialize' request";

    qDebug() << "Build 'initialize' response";
    common_utilities::ControllerState controller;
    for(qint8 id : req.idList) {
        controller.id = id;
        controller.state = 1;
        res.controllers.push_back(controller);
        qDebug() << "\t- Update Controller State: [id:" << id << "][state:" << controller.state << "]";
    }

    return true;
}

bool callbackMotor1(common_utilities::Trajectory::Request & req, common_utilities::Trajectory::Response & res){
    qDebug() << "Received Service Call: TRAJECTORY on topic 'motor1'";
    qDebug() << "TRAJECTORY: [samplerate:" << req.samplerate << "][controlmode:" << req.controlmode << "]";
    qint32 index = 0;
    for(qint32 wp : req.waypoints) {
        qDebug() << " - WAYPOINT" << ++index << ": [value:" << wp << "]";
    }

    qDebug() << "Send Service Response on topic 'motor1'";
    qDebug() << "\t- Update Controller State: [id:" << 1 << "][state:" << 1 << "]";
    res.state.id = 1;
    res.state.state = 1;
}

void callbackSteer(const common_utilities::Steer& msg){
    qDebug() << "Heard steering message: " << msg.steeringaction;
}

int main(int argc, char ** argv) {
    qDebug() << "Test node started ...";

    ros::init(argc, argv, "roboy_control_test_node");
    ros::NodeHandle m_nodeHandle;
    ros::NodeHandle m_nodeHandle2;

    ros::ServiceServer initializeServer = m_nodeHandle.advertiseService("initialize", callbackInitialize);
    ros::ServiceServer motor1Server = m_nodeHandle2.advertiseService("motor1", callbackMotor1);

//    subscriberMotor1 = m_nodeHandle.subscribe("motor1", 1000, callbackMotor1);
//    subscriberSteer = m_nodeHandle.subscribe("steeringaction", 1000, callbackSteer);
//    publisherStatus = m_nodeHandle2.advertise<common_utilities::ControllerState>("motor_status", 1000);

    ros::spin();
}
