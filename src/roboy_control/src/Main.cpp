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

    return app.exec();
}