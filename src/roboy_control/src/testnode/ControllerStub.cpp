//
// Created by bruh on 1/24/16.
//

#include <QDebug>

#include "common_utilities/Trajectory.h"
#include "DataTypes.h"

#include "ros/ros.h"

QString nodeName;
ControllerState state;

bool callbackMotor(common_utilities::Trajectory::Request & req, common_utilities::Trajectory::Response & res);

int main(int argc, char ** argv) {
    ros::init(argc, argv, "controllerStub");

    if(argc != 2) {
        qDebug() << "Invalid number of arguments";
        exit(1);
    }

    qDebug() << "Started ControllerStub-Node: " << argv[1];
    ros::NodeHandle n;

    state = ControllerState::INITIALIZED;

    nodeName = argv[1];

    qDebug() << "[" << nodeName << "] Advertise Service: " << nodeName;
    n.advertiseService(nodeName.toStdString(), callbackMotor);

    ros::spin();
}

bool callbackMotor(common_utilities::Trajectory::Request & req, common_utilities::Trajectory::Response & res){
    qDebug() << "[" << nodeName << "] " << "Received Service Call: TRAJECTORY for motor ";
    qDebug() << "[" << nodeName << "] " << "TRAJECTORY: [samplerate:" << req.samplerate << "][controlmode:" << req.controlmode << "]";
    qint32 index = 0;
    for(qint32 wp : req.waypoints) {
        qDebug() << " - WAYPOINT" << ++index << ": [value:" << wp << "]";
    }

    qDebug() << "[" << nodeName << "] " << "Send Service Response on topic 'motor1'";
    res.state.id = 1;
    res.state.state = ControllerState::TRAJECTORY_READY ;
    qDebug() << "\t- Update Controller State: [id:" << res.state.id << "][state:" << res.state.state << "]";

    return true;
}
