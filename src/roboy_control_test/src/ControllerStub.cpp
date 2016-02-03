//
// Created by bruh on 1/24/16.
//

#include "DataTypes.h"

#include <QDebug>

#include "common_utilities/Trajectory.h"

#include "ros/ros.h"

QString nodeName;
STATUS state;

bool callbackMotor(common_utilities::Trajectory::Request & req, common_utilities::Trajectory::Response & res);

int main(int argc, char ** argv) {

    ros::init(argc, argv, "controller_stub_1");
    ros::NodeHandle n;

    qDebug() << "Started ControllerStub-Node: " << argv[1];
    nodeName = argv[1];

    state = STATUS::INITIALIZED;

    qDebug() << "[" << nodeName << "] Advertise Service: " << nodeName;
    ros::ServiceServer trajectoryServer =  n.advertiseService("roboy/trajectory_motor1", callbackMotor);

    ros::spin();
}

bool callbackMotor(common_utilities::Trajectory::Request & req, common_utilities::Trajectory::Response & res){
    qDebug() << "[" << nodeName << "] " << "Received Service Call: TRAJECTORY for motor ";
    qDebug() << "[" << nodeName << "] " << "TRAJECTORY: [samplerate:" << req.samplerate << "]";
    qint32 index = 0;
    for(qint32 wp : req.waypoints) {
        qDebug() << " - WAYPOINT" << ++index << ": [value:" << wp << "]";
    }

    qDebug() << "[" << nodeName << "] " << "Send Service Response on topic 'motor1'";
    res.state.id = 1;
    res.state.state = STATUS::TRAJECTORY_READY ;
    qDebug() << "\t- Update Controller State: [id:" << res.state.id << "][state:" << res.state.state << "]";

    return true;
}
