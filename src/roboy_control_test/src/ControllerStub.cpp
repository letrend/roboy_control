//
// Created by bruh on 1/24/16.
//

#include "DataTypes.h"
#include "common_utilities/Steer.h"
#include <common_utilities/ControllerState.h>
#include "common_utilities/Trajectory.h"

#include "ros/ros.h"

QString nodeName;
QString serviceName;
qint32 id;
ControllerState state;

ros::ServiceServer trajectoryServer;
ros::Subscriber steeringSubscriber;
ros::Publisher statusPublisher;

bool callbackMotor(common_utilities::Trajectory::Request & req, common_utilities::Trajectory::Response & res);
void callbackSteering(const common_utilities::Steer::ConstPtr & msg);

void setState(ControllerState cs);

int main(int argc, char ** argv) {

    id = atoi(argv[1]);
    nodeName = argv[2];
    serviceName = argv[3];
    qDebug() << "Start ControllerStub-Node: " << nodeName;
    qDebug() << "Service Name: " << serviceName;

    ros::init(argc, argv, nodeName.toStdString());
    ros::NodeHandle n;

    ros::AsyncSpinner spinner(1);
    spinner.start();

    QString statusTopic;
    statusTopic.sprintf("/roboy/status_motor%u", id);

    trajectoryServer =  n.advertiseService(serviceName.toStdString(), callbackMotor);
    steeringSubscriber = n.subscribe("roboy/steer", 1000, callbackSteering);
    statusPublisher = n.advertise<common_utilities::ControllerState>(statusTopic.toStdString(), 1000);

    ros::Duration duration(1);
    while(statusPublisher.getNumSubscribers() == 0)
        duration.sleep();

    setState(ControllerState::INITIALIZED);

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
    setState(ControllerState::TRAJECTORY_READY);
    qDebug() << "\t- Update Controller State: [id:" << id << "][state:" << res.state << "]";

    return true;
}

void callbackSteering(const common_utilities::Steer::ConstPtr & msg) {
    qDebug() << "Received Steering Message";
    if(msg->steeringCommand == SteeringCommand::PLAY_TRAJECTORY){
        setState(ControllerState::TRAJECTORY_PLAYING);
    } else if (msg->steeringCommand == SteeringCommand::STOP_TRAJECTORY) {
        setState(ControllerState::TRAJECTORY_READY);
    } else if (msg->steeringCommand == SteeringCommand::PAUSE_TRAJECTORY) {
        setState(ControllerState::TRAJECTORY_PAUSED);
    }
}

void setState(ControllerState cs) {
    state = cs;

    common_utilities::ControllerState stateMessage;
    stateMessage.id = id;
    stateMessage.state = cs;

    statusPublisher.publish(stateMessage);
}