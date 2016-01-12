//
// Created by bruh on 30.11.15.
//

#ifndef ROBOYCONTROL_ITRANSCEIVERSERVICE_H
#define ROBOYCONTROL_ITRANSCEIVERSERVICE_H

#include "DataTypes.h"
#include "ITransceiverServiceDelegate.h"
#include "common_utilities/InitializeResponse.h"
#include "common_utilities/Status.h"
#include <stdint.h>

class ITransceiverService {
protected:
    ITransceiverServiceDelegate * delegate;

public:
    void setDelegate(ITransceiverServiceDelegate * delegate) {
        this->delegate = delegate;
    }

    // MyoMaster Interface
    virtual void sendInitializeRequest(const std::list<qint8> initializationList) = 0;

    // MotorController Interface
    virtual void sendTrajectory(quint32 motorId, const Trajectory trajectory) = 0;
    virtual void sendSteeringMessage(uint8_t steeringaction) = 0;
};


#endif //ROBOYCONTROL_ITRANSCEIVERSERVICE_H
