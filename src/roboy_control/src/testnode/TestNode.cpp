//
// Created by bruh on 12/14/15.
//

#include <QDebug>

#include "ros/ros.h"
#include "std_msgs/String.h"

#include "common_utilities/InitializeRequest.h"
#include "common_utilities/InitializeResponse.h"
#include "common_utilities/Trajectory.h"
#include "common_utilities/Steer.h"
#include "common_utilities/Status.h"

#include "CommonDefinitions.h"

ros::Publisher publisher;
ros::Publisher publisherStatus;
ros::Subscriber subscriber;
ros::Subscriber subscriberMotor1;
ros::Subscriber subscriberSteer;

void callback(const common_utilities::InitializeRequest& msg){
    qDebug() << "Heard InitializeRequest.";

    common_utilities::InitializeResponse response;

    common_utilities::ControllerState controller;
    for(qint8 id : msg.idList) {
        controller.id = id;
        controller.state = 1;
        response.controllers.push_back(controller);
    }
    qDebug() << "Prepared response.";

    ros::Rate rollRate(10);
    while(publisher.getNumSubscribers() == 0) {
        rollRate.sleep();
    }
    qDebug() << "Initialize response subscribers: " << publisher.getNumSubscribers();

    qDebug() << "Publishing InitializeResponse.";
    publisher.publish(response);
}

void callbackMotor1(const common_utilities::Trajectory& msg){
    qDebug() << "Received Message: TRAJECTORY on topic 'motor1'";
    qDebug() << "TRAJECTORY: [samplerate:" << msg.samplerate << "][controlmode:" << msg.controlmode << "]";
    qint32 index = 0;
    for(qint32 wp : msg.waypoints) {
        qDebug() << " - WAYPOINT" << ++index << ": [value:" << wp << "]";
    }

    qDebug() << "Send Message on topic 'motor_status': 'ControllerState'";
    common_utilities::ControllerState response;
    response.id = 1;
    response.state = ControllerState::TRAJECTORY_READY;
    publisherStatus.publish(response);
}

void callbackSteer(const common_utilities::Steer& msg){
    qDebug() << "Heard steering message: " << msg.steeringaction;
}

int main(int argc, char ** argv) {
    qDebug() << "Test node started ...";

    ros::init(argc, argv, "roboy_control_test_node");
    ros::NodeHandle m_nodeHandle;
    ros::NodeHandle m_nodeHandle2;
    subscriber = m_nodeHandle.subscribe("initialize_request", 1000, callback);
    subscriberMotor1 = m_nodeHandle.subscribe("motor1", 1000, callbackMotor1);
    subscriberSteer = m_nodeHandle.subscribe("steeringaction", 1000, callbackSteer);
    publisher = m_nodeHandle2.advertise<common_utilities::InitializeResponse>("initialize_response", 1000);
    publisherStatus = m_nodeHandle2.advertise<common_utilities::ControllerState>("motor_status", 1000);

    ros::spin();
}
