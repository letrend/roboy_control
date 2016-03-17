//
// Created by bruh on 1/24/16.
//

#include <common_utilities/ControllerState.h>
#include "DataTypes.h"

#include "common_utilities/Steer.h"
#include "common_utilities/Trajectory.h"

#include "ros/ros.h"

QString nodeName;
QString serviceName;
qint32 id;
STATUS state;

bool callbackMotor(common_utilities::Trajectory::Request & req, common_utilities::Trajectory::Response & res);
//void callbackSteering(common_utilities::Steer::ConstPtr & msg);

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

    //ros::Subscriber steeringSubscriber = n.subscribe("roboy/steering", 1000, callbackSteering);

    ros::AsyncSpinner spinner(1);
    spinner.start();

    QString statusTopic;
    statusTopic.sprintf("/roboy/status_motor%u", id);
    ros::Publisher statusPublisher = n.advertise<common_utilities::ControllerState>(statusTopic.toStdString(), 1000);

    ros::Duration duration(1,0);
    while(1) {
        qDebug() << "Advertixe 'ControllerState' Message";
        common_utilities::ControllerState stateMessage;
        stateMessage.id = id;
        stateMessage.state = STATUS::INITIALIZED;
        statusPublisher.publish(stateMessage);
        duration.sleep();
    }
}

bool callbackMotor(common_utilities::Trajectory::Request & req, common_utilities::Trajectory::Response & res){
    qDebug() << "[" << nodeName << "] " << "Received Service Call: TRAJECTORY for motor ";
    qDebug() << "[" << nodeName << "] " << "TRAJECTORY: [samplerate:" << req.samplerate << "]";
    qint32 index = 0;
    for(qint32 wp : req.waypoints) {
        qDebug() << " - WAYPOINT" << ++index << ": [value:" << wp << "]";
    }

    qDebug() << "[" << nodeName << "] " << "Send Service Response on topic 'motor1'";
    res.state = STATUS::TRAJECTORY_READY ;
    qDebug() << "\t- Update Controller State: [id:" << id << "][state:" << res.state << "]";

    return true;
}

//void callbackSteering(common_utilities::Steer::ConstPtr & msg) {
//    qDebug() << "Received Steering Message";
//}
