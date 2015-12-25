//
// Created by bruh on 30.11.15.
//

#ifndef ROBOYCONTROL_ITRANSCEIVERSERVICE_H
#define ROBOYCONTROL_ITRANSCEIVERSERVICE_H

#include "../DataTypes.h"
#include "roboy_control/InitializeResponse.h"
#include "roboy_control/Status.h"
#include <stdint.h>

class ITransceiverService {

public:
    // MyoMaster Interface
    virtual void sendInitializeRequest(std::vector<bool> enable) = 0;
    virtual void receiveInitializeResponse(const roboy_control::InitializeResponse& msg) = 0;

    // MotorController Interface
    virtual void sendTrajectory(u_int32_t motorId, QList<RoboyWaypoint> & waypoints) = 0;
    virtual void receiveControllerStatus(const roboy_control::Status &msg) = 0;

    virtual void sendSteeringMessage(uint8_t steeringaction) = 0;
};


#endif //ROBOYCONTROL_ITRANSCEIVERSERVICE_H
