//
// Created by bruh on 3/16/16.
//

#ifndef ROBOY_CONTROL_ROSCONTROLLERHANDLE_H
#define ROBOY_CONTROL_ROSCONTROLLERHANDLE_H

#include "IMotorController.h"

#include <common_utilities/ControllerState.h>

#include <ros/ros.h>

class ROSMotorController : public IMotorController {

private:
    ros::NodeHandle m_nodeHandle;
    ros::Subscriber m_statusSubscriber;
    ros::Publisher  m_trajectoryPublisher;

public:
    ROSMotorController(qint32 id, const ControlMode controlMode);
    ~ROSMotorController() {}

    void sendTrajectory(const Trajectory & trajectory) const;

private:
    void callbackStatus(const common_utilities::ControllerStateConstPtr & status) const;
};


#endif //ROBOY_CONTROL_ROSCONTROLLERHANDLE_H
