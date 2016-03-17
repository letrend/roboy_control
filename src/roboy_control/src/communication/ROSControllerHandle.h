//
// Created by bruh on 3/16/16.
//

#ifndef ROBOY_CONTROL_ROSCONTROLLERHANDLE_H
#define ROBOY_CONTROL_ROSCONTROLLERHANDLE_H

#include "common_utilities/ControllerState.h"

#include "DataTypes.h"
#include "IControllerHandle.h"

class ROSControllerHandle : public IControllerHandle {

private:

public:
    ROSControllerHandle(qint8 controllerId, ControlMode controlMode);

    bool sendTrajectory(const Trajectory & trajectory);

private:
    void listenOnControllerStatus();
    void callbackStatus(const common_utilities::ControllerStateConstPtr & status);
};


#endif //ROBOY_CONTROL_ROSCONTROLLERHANDLE_H
