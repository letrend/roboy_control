//
// Created by bruh on 1/24/16.
//

#include "DataTypes.h"

#include <QDebug>

#include "common_utilities/Steer.h"
#include "common_utilities/Trajectory.h"

#include "ros/ros.h"

QString nodeName;
QString serviceName;
qint32 id;
STATUS state;

bool callbackMotor(common_utilities::Trajectory::Request & req, common_utilities::Trajectory::Response & res);
bool callbackSteering(common_utilities::Steer & msg);

int main(int argc, char ** argv) {

    id = atoi(argv[1]);
    nodeName = argv[2];
    serviceName = argv[3];
    qDebug() << "Start ControllerStub-Node: " << nodeName;
    qDebug() << "Service Name: " << serviceName;

    ros::init(argc, argv, nodeName.toStdString());
    ros::NodeHandle n;

    state = STATUS::INITIALIZED;

    ros::ServiceServer trajectoryServer =  n.advertiseService(serviceName.toStdString(), callbackMotor);
    ros::Subscriber steeringSubscriber = n.subscribe("roboy/steering", 1000, callbackSteering);

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
    res.state.id = id;
    res.state.state = STATUS::TRAJECTORY_READY ;
    qDebug() << "\t- Update Controller State: [id:" << res.state.id << "][state:" << res.state.state << "]";

    return true;
}

bool callbackSteering(common_utilities::Steer & msg) {
    return true;
}