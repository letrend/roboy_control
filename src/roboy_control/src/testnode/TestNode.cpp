//
// Created by bruh on 12/14/15.
//

#include <QDebug>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "roboy_control/InitializeRequest.h"
#include "roboy_control/InitializeResponse.h"
#include "roboy_control/Trajectory.h"
#include "roboy_control/Steer.h"
#include "roboy_control/Status.h"

ros::Publisher publisher;
ros::Publisher publisherMotor1;
ros::Subscriber subscriber;
ros::Subscriber subscriberMotor1;
ros::Subscriber subscriberSteer;

void callback(const roboy_control::InitializeRequest& msg){
    qDebug() << "Heard InitializeRequest.";


    roboy_control::InitializeResponse response;

    for(bool b : msg.enable) {
        response.status.push_back(1);
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

void callbackMotor1(const roboy_control::Trajectory& msg){
    qDebug() << "Heard Trajectory message.";
    roboy_control::Status response;

    response.statusCode = 1;
    qDebug() << "Prepared response.";

    ros::Rate rollRate(10);
    while(publisherMotor1.getNumSubscribers() == 0) {
        rollRate.sleep();
    }
    qDebug() << "Motor status subscribers: " << publisherMotor1.getNumSubscribers();

    qDebug() << "Publishing MotorStatus.";
    publisherMotor1.publish(response);
}

void callbackSteer(const roboy_control::Steer& msg){
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
    publisher = m_nodeHandle2.advertise<roboy_control::InitializeResponse>("initialize_response", 1000);
    publisherMotor1 = m_nodeHandle2.advertise<roboy_control::Status>("motor_status1", 1000);

    ros::spin();
}
