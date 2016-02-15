//
// Created by bruh on 2/4/16.
//

#include <QDebug>
#include "ros/ros.h"
#include "TestNode.h"

int main(int argc, char ** argv) {
    qDebug() << "Test node started ...";

    ros::init(argc, argv, "roboy_control_test_node");
    ros::NodeHandle m_nodeHandle;

    TestNode testNode;

    ros::spin();
}
