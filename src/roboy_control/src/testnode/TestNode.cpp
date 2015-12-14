//
// Created by bruh on 12/14/15.
//

#include <QDebug>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "roboy_control/InitializeRequest.h"


void callback(const roboy_control::InitializeRequest& msg){
    qDebug() << "I heard InitializeRequest";
}
int main(int argc, char ** argv) {
    qDebug() << "Test node started ...";

    ros::init(argc, argv, "roboy_control_test_node");
    ros::NodeHandle n;

    ros::Subscriber subscriber = n.subscribe("initialize_request", 1000, callback);

    ros::spin();
}
