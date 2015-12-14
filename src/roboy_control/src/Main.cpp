//
// Created by bruh on 17.11.15.
//

#include <QApplication>
#include "controller/RoboyController.h"
#include "transceiver/ROSMessageTransceiverService.h"
#include "ros/ros.h"

int main(int argc, char ** argv) {
    ros::init(argc, argv, "roboy_control");
    ros::NodeHandle n;

    QApplication app(argc, argv);

    RoboyController controller;
    controller.start();

//    RoboyBehavior newBehavior;
//    newBehavior.m_metadata.m_sBehaviorName = "GreetBehavior";
//    newBehavior.m_metadata.m_ulBehaviorId = 2;
//
//    ROSMessageTransceiverService t;
//    ros::Subscriber sub = n.subscribe("motor", 1000, callback);
//    ros::spinOnce();
//    QList<RoboyWaypoint> list;
//    RoboyWaypoint waypoint;
//    waypoint.m_ulId = 1;
//    waypoint.m_ulPosition = 1000;
//    list.append(waypoint);
//    t.sendTrajectory(1, list);

    return app.exec();
}