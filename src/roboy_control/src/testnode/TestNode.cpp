//
// Created by bruh on 12/14/15.
//

#include <QDebug>
#include "ros/ros.h"
#include "std_msgs/String.h"


void callback(const std_msgs::String::ConstPtr& msg){
    qDebug() << "I heard: " << msg->data.c_str();
}
int main(int argc, char ** argv) {
    qDebug() << "Test node started ...";

    ros::init(argc, argv, "roboy_control_test_node");
    ros::NodeHandle n;

    ros::Subscriber subscriber = n.subscribe("motor", 1000, callback);

    ros::spin();
}
