//
// Created by bruh on 3/16/16.
//

#ifndef ROBOY_CONTROL_ROSCONTROLLERHANDLE_H
#define ROBOY_CONTROL_ROSCONTROLLERHANDLE_H

#include <CommonDefinitions.h>
#include <common_utilities/ControllerState.h>
#include <common_utilities/SetTrajectory.h>
#include <ros/node_handle.h>
#include <ros/service_client.h>
#include "IControllerCommunication.h"

class ROSControllerCommunication : public IControllerCommunication {

private:
    ros::NodeHandle      m_nodeHandle;
    ros::Subscriber      m_statusSubscriber;
    ros::ServiceClient   m_trajectoryService;

public:
    ROSControllerCommunication(ROSController * controller);
    ~ROSControllerCommunication() {}

    void eventHandle_sendTrajectory();

private:
    void listenOnControllerStatus();
    void callbackStatus(const common_utilities::ControllerStateConstPtr & status);
};


#endif //ROBOY_CONTROL_ROSCONTROLLERHANDLE_H
