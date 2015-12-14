//
// Created by bruh on 30.11.15.
//

#ifndef ROBOYCONTROL_ITRANSCEIVERSERVICE_H
#define ROBOYCONTROL_ITRANSCEIVERSERVICE_H

#include "../DataTypes.h"

class ITransceiverService {

public:
    // MyoMaster Interface
    virtual void sendInitializeRequest(std::vector<bool> enable) = 0;
    virtual void receiveInitializeResponse() = 0;

    // MotorController Interface
    virtual void sendTrajectory(u_int32_t motorId, QList<RoboyWaypoint> & waypoints) = 0;
    virtual void receiveControllerStatus(u_int32_t motorId) = 0;

    virtual void sendSteeringMessage() = 0;
};


#endif //ROBOYCONTROL_ITRANSCEIVERSERVICE_H
